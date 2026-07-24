#include "xil_printf.h"
#include "xil_io.h"
#include "xparameters.h"
#include "xil_cache.h"
#include "xtmrctr.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// =============================================================
//  MEMORY MAP
// =============================================================
#define DDR_BASE_ADDR         0x80000000
#define QUERY_BASE_ADDR       0x80800000
#define MAIN_POINTS_ADDR      0x81000000
#define UNIQUE_BINS_ADDR      0x81800000

// 8MB target staging split 4MB per channel
// ch0: 0x82000000 .. 0x82400000
// ch1: 0x82400000 .. 0x82800000
#define TARGET_STAGING_ADDR   0x82000000

// 1MB staging IDs split 512KB per channel (131072 ints each)
// ch0: 0x82800000 .. 0x82880000
// ch1: 0x82880000 .. 0x82900000
#define STAGING_IDS_ADDR      0x82800000

// 1MB anchor staging split 64KB per channel
// ch0: 0x82900000 .. 0x82910000
// ch1: 0x82910000 .. 0x82920000
#define ANCHOR_STAGING_ADDR   0x82900000

#define COORD_INFO_ADDR       0x82A00000
#define SORT_KEY_ADDR         0x82C00000
#define QUERY_SORT_ADDR       0x83000000
#define POINT_EPOCH_ADDR      0x83400000

// 62MB result space split 31MB per channel
// ch0: 0x84000000 .. 0x85F00000
// ch1: 0x85F00000 .. 0x87E00000
#define RESULTS_ADDR          0x84000000
#define MATCHES_ADDR          0x87E00000

// =============================================================
//  HARDWARE ADDRESSES
// =============================================================
#define DMA_0_BASE            0x41E00000
#define IP_0_BASE             0x00008000
#define DMA_1_BASE            0x41E10000
#define IP_1_BASE             0x00010000

// =============================================================
//  CONSTANTS
// =============================================================
#define MAX_RANGE             10.0f      // <-- change this for different radii
#define MAX_BRAM_CAPACITY     2500       // must match HLS kernel
#define MAX_STAGING_SLOTS     131072     // per channel: 4MB / 32 bytes
#define MAX_MATCHES           2000000000
#define WATCHDOG_LIMIT        20000000
#define UNROLL_FACTOR         2          // must match HLS kernel
#define TIMER_FREQ_HZ         200000000ULL
#define MAX_ANCHORS_PER_TX    2048

// =============================================================
//  DATA TYPES
// =============================================================
typedef struct { float d[6]; float pad1; float pad2; }      RawPointPacket;
typedef struct { float d[6]; int bin_id; int original_id; } BinPointPacket;
typedef struct { int bin_id; int offset; int count; }        BinInfo;
typedef struct {
   float distances[UNROLL_FACTOR];
   int   target_id;
   int   anchor_b_idx;
   int   pad1, pad2, pad3, pad4;
} DistPacket;
typedef struct { int bin_id; int query_idx; } QueryBinKey;
typedef struct { int bin_id; int orig_idx; }  SortKey;
typedef struct { int cord_id; float cord_min; float cord_max; float cord_spread; } CordInfo;
typedef struct { int anchor_id; int target_id; float distance; } MatchRecord;

#define BIN3D(bx,by,bz,nby,nbz) ((bx)*((nby)*(nbz))+(by)*(nbz)+(bz))

// =============================================================
//  HELPERS
// =============================================================
u64 get_timer_value(XTmrCtr *t) {
   u32 u1 = XTmrCtr_GetValue(t, 1);
   u32 lo = XTmrCtr_GetValue(t, 0);
   u32 u2 = XTmrCtr_GetValue(t, 1);
   if (u1 != u2) lo = XTmrCtr_GetValue(t, 0);
   return (((u64)u2) << 32) | (u64)lo;
}

int binary_search_bins(BinInfo *arr, int count, int target_id) {
   int lo = 0, hi = count - 1;
   while (lo <= hi) {
       int mid = lo + ((hi - lo) >> 1);
       if      (arr[mid].bin_id == target_id) return mid;
       else if (arr[mid].bin_id  < target_id) lo = mid + 1;
       else                                    hi = mid - 1;
   }
   return -1;
}

static int g_sort_dim = 4;
int binary_search_bins_points(BinPointPacket *arr, int nb_offset, int nb_count,
       float lowest_valid, float highest_valid, int is_start, int sort_dim) {
   int lo = 0, hi = nb_count - 1;
   int ans = is_start ? nb_count : -1;
   while (lo <= hi) {
       int mid = lo + ((hi - lo) >> 1);
       if (is_start) {
           if (arr[nb_offset+mid].d[sort_dim] >= lowest_valid) { ans = mid; hi = mid-1; }
           else lo = mid+1;
       } else {
           if (arr[nb_offset+mid].d[sort_dim] <= highest_valid) { ans = mid; lo = mid+1; }
           else hi = mid-1;
       }
   }
   return ans;
}

int compare_d4(const void *a, const void *b) {
   BinPointPacket *p1 = (BinPointPacket *)a;
   BinPointPacket *p2 = (BinPointPacket *)b;
   if (p1->d[g_sort_dim] < p2->d[g_sort_dim]) return -1;
   if (p1->d[g_sort_dim] > p2->d[g_sort_dim]) return  1;
   return 0;
}

int compare_query_bins(const void *a, const void *b) {
   QueryBinKey *k1 = (QueryBinKey *)a;
   QueryBinKey *k2 = (QueryBinKey *)b;
   return k1->bin_id - k2->bin_id;
}

static inline void dma_tx_wait(u32 dma_base, const char *label) {
   int wd = 0;
   while (!(Xil_In32(dma_base + 0x04) & 0x0002)) {
       if (wd++ > WATCHDOG_LIMIT) { xil_printf("[FATAL] DMA TX: %s\n\r", label); while(1); }
   }
}

static inline void dma_rx_wait(u32 dma_base, const char *label) {
   int wd = 0;
   while (!(Xil_In32(dma_base + 0x34) & 0x0002)) {
       if (wd++ > WATCHDOG_LIMIT) { xil_printf("[FATAL] DMA RX: %s\n\r", label); while(1); }
   }
}

static inline void ip_wait_idle(u32 ip_base, const char *label) {
   int wd = 0;
   while (!(Xil_In32(ip_base + 0x00) & 0x04)) {
       if (wd++ > WATCHDOG_LIMIT) { xil_printf("[FATAL] IP stuck: %s\n\r", label); while(1); }
   }
}

// =============================================================
//  MAIN
// =============================================================
int main() {
   XTmrCtr TimerInstance;
   XTmrCtr_Initialize(&TimerInstance, XPAR_TMRCTR_0_DEVICE_ID);
   XTmrCtr_SetOptions(&TimerInstance, 0, XTC_CASCADE_MODE_OPTION);
   XTmrCtr_Start(&TimerInstance, 0);

   int *file_header   = (int *)DDR_BASE_ADDR;
   int  num_dims      = file_header[0];
   int  total_points  = file_header[1];

   xil_printf("MAX_RANGE=%d\n\r", (int)MAX_RANGE);

   int *query_header  = (int *)QUERY_BASE_ADDR;
   int  query_count   = query_header[1];
   RawPointPacket *query_coords = (RawPointPacket *)(QUERY_BASE_ADDR + 8);

   RawPointPacket *all_coords       = (RawPointPacket *)(DDR_BASE_ADDR + 8);
   BinPointPacket *main_points_array= (BinPointPacket *)MAIN_POINTS_ADDR;
   int unique_bin_count = 0;
   unsigned long long total_calcs   = 0;

   CordInfo *coord_vals = (CordInfo *)COORD_INFO_ADDR;

   // =========================================================
   //  P1: SPATIAL ANALYSIS
   // =========================================================
   for (int dim = 0; dim < 6; dim++) {
       coord_vals[dim].cord_id  = dim;
       coord_vals[dim].cord_min =  999999.0f;
       coord_vals[dim].cord_max = -999999.0f;
   }
   for (int i = 0; i < total_points; i++) {
       main_points_array[i].bin_id      = 0;
       main_points_array[i].original_id = i;
       for (int dim = 0; dim < 6; dim++) {
           float d = all_coords[i].d[dim];
           main_points_array[i].d[dim] = d;
           if (d < coord_vals[dim].cord_min) coord_vals[dim].cord_min = d;
           if (d > coord_vals[dim].cord_max) coord_vals[dim].cord_max = d;
       }
   }
   for (int dim = 0; dim < 6; dim++)
       coord_vals[dim].cord_spread = (coord_vals[dim].cord_max - coord_vals[dim].cord_min) + 0.0001f;

   int top1_id=-1, top2_id=-1, top3_id=-1, top4_id=-1;
   float top1_s=-1,top2_s=-1,top3_s=-1,top4_s=-1;
   for (int j = 0; j < num_dims; j++) {
       float cs = coord_vals[j].cord_spread; int ci = coord_vals[j].cord_id;
       if      (cs > top1_s) { top4_s=top3_s;top4_id=top3_id;top3_s=top2_s;top3_id=top2_id;top2_s=top1_s;top2_id=top1_id;top1_s=cs;top1_id=ci; }
       else if (cs > top2_s) { top4_s=top3_s;top4_id=top3_id;top3_s=top2_s;top3_id=top2_id;top2_s=cs;top2_id=ci; }
       else if (cs > top3_s) { top4_s=top3_s;top4_id=top3_id;top3_s=cs;top3_id=ci; }
       else if (cs > top4_s) { top4_s=cs;top4_id=ci; }
   }
   xil_printf("top1=%d top2=%d top3=%d top4=%d\n\r", top1_id, top2_id, top3_id, top4_id);

   float spread_x = top1_s, spread_y = top2_s, spread_z = top3_s;
   float max_range = MAX_RANGE;
   union { float f; u32 u; } f2u; f2u.f = max_range;
   u32   max_range_bits = f2u.u;
   float max_range_sq   = max_range * max_range;
   float voxel_size     = MAX_RANGE;

   int nbx = (int)(spread_x/voxel_size); if(nbx<1) nbx=1;
   int nby = (num_dims>1)?(int)(spread_y/voxel_size):1; if(nby<1) nby=1;
   int nbz = (num_dims>2)?(int)(spread_z/voxel_size):1; if(nbz<1) nbz=1;
   long long total_voxels = (long long)nbx*nby*nbz;
   if (total_voxels > 2000000000LL) {
       float scale = 1.0f;
       while (total_voxels > 2000000000LL) {
           scale *= 1.5f;
           nbx=(int)(spread_x/(voxel_size*scale)); if(nbx<1) nbx=1;
           nby=(num_dims>1)?(int)(spread_y/(voxel_size*scale)):1; if(nby<1) nby=1;
           nbz=(num_dims>2)?(int)(spread_z/(voxel_size*scale)):1; if(nbz<1) nbz=1;
           total_voxels=(long long)nbx*nby*nbz;
       }
       voxel_size *= scale;
   }
   float inv_bx = (float)nbx/spread_x;
   float inv_by = (float)nby/spread_y;
   float inv_bz = (float)nbz/spread_z;

   // =========================================================
   //  P2-P3: BIN ASSIGNMENT
   // =========================================================
   for (int i = 0; i < total_points; i++) {
       int bx=(int)((main_points_array[i].d[top1_id]-coord_vals[top1_id].cord_min)*inv_bx);
       if(bx>=nbx) bx=nbx-1; if(bx<0) bx=0;
       int by=(num_dims>1)?(int)((main_points_array[i].d[top2_id]-coord_vals[top2_id].cord_min)*inv_by):0;
       if(by>=nby) by=nby-1; if(by<0) by=0;
       int bz=(num_dims>2)?(int)((main_points_array[i].d[top3_id]-coord_vals[top3_id].cord_min)*inv_bz):0;
       if(bz>=nbz) bz=nbz-1; if(bz<0) bz=0;
       main_points_array[i].bin_id = BIN3D(bx,by,bz,nby,nbz);
   }

   // =========================================================
   //  P4: RADIX SORT + SECONDARY SORT BY TOP4 DIM
   // =========================================================
   xil_printf("[P4] Radix sorting...\n\r");
   SortKey *sort_keys = (SortKey *)SORT_KEY_ADDR;
   SortKey *temp_keys = sort_keys + total_points;
   int *cnt = (int *)(COORD_INFO_ADDR + 256);

   for (int i = 0; i < total_points; i++) { sort_keys[i].bin_id=main_points_array[i].bin_id; sort_keys[i].orig_idx=i; }
   memset(cnt, 0, 65536*sizeof(int));
   for (int i = 0; i < total_points; i++) cnt[sort_keys[i].bin_id & 0xFFFF]++;
   for (int i = 1; i < 65536; i++) cnt[i]+=cnt[i-1];
   for (int i = total_points-1; i >= 0; i--) temp_keys[--cnt[sort_keys[i].bin_id & 0xFFFF]]=sort_keys[i];
   memset(cnt, 0, 65536*sizeof(int));
   for (int i = 0; i < total_points; i++) cnt[(temp_keys[i].bin_id>>16)&0xFFFF]++;
   for (int i = 1; i < 65536; i++) cnt[i]+=cnt[i-1];
   for (int i = total_points-1; i >= 0; i--) sort_keys[--cnt[(temp_keys[i].bin_id>>16)&0xFFFF]]=temp_keys[i];

   BinPointPacket *temp_pts = (BinPointPacket *)RESULTS_ADDR;
   for (int i = 0; i < total_points; i++) temp_pts[i]=main_points_array[sort_keys[i].orig_idx];
   for (int i = 0; i < total_points; i++) main_points_array[i]=temp_pts[i];

   int bin_start=0; int current_b=main_points_array[0].bin_id;
   g_sort_dim = top4_id;
   for (int i = 1; i <= total_points; i++) {
       int b = (i<total_points)?main_points_array[i].bin_id:-1;
       if (b != current_b) {
           int bin_len=i-bin_start;
           if (bin_len>1) qsort(&main_points_array[bin_start],bin_len,sizeof(BinPointPacket),compare_d4);
           bin_start=i; current_b=b;
       }
   }
   xil_printf("[P4] Done.\n\r");

   // =========================================================
   //  P5: EXTRACT UNIQUE BINS
   // =========================================================
   BinInfo *unique_bins = (BinInfo *)UNIQUE_BINS_ADDR;
   int current_bin = -1;
   for (int i = 0; i < total_points; i++) {
       int b = main_points_array[i].bin_id;
       if (b != current_bin) {
           current_bin=b;
           unique_bins[unique_bin_count].bin_id=b;
           unique_bins[unique_bin_count].offset=i;
           unique_bins[unique_bin_count].count=1;
           unique_bin_count++;
       } else {
           unique_bins[unique_bin_count-1].count++;
       }
   }
   xil_printf("[P5] Found %d bins.\n\r", unique_bin_count);

   // =========================================================
   //  P6: DUAL-IP HARDWARE RANGE JOIN
   // =========================================================
   xil_printf("[P6 ..\n\r");
   u64 time_start_hw = get_timer_value(&TimerInstance);

   // DMA soft reset both channels
   Xil_Out32(DMA_0_BASE+0x00, 0x0004); Xil_Out32(DMA_0_BASE+0x30, 0x0004);
   while ((Xil_In32(DMA_0_BASE+0x00)&0x0004)||(Xil_In32(DMA_0_BASE+0x30)&0x0004));
   Xil_Out32(DMA_1_BASE+0x00, 0x0004); Xil_Out32(DMA_1_BASE+0x30, 0x0004);
   while ((Xil_In32(DMA_1_BASE+0x00)&0x0004)||(Xil_In32(DMA_1_BASE+0x30)&0x0004));
   Xil_Out32(DMA_0_BASE+0x04, 0x1000); Xil_Out32(DMA_0_BASE+0x34, 0x1000);
   Xil_Out32(DMA_0_BASE+0x00, 0x0001); Xil_Out32(DMA_0_BASE+0x30, 0x0001);
   Xil_Out32(DMA_1_BASE+0x04, 0x1000); Xil_Out32(DMA_1_BASE+0x34, 0x1000);
   Xil_Out32(DMA_1_BASE+0x00, 0x0001); Xil_Out32(DMA_1_BASE+0x30, 0x0001);

   // -----------------------------------------------------------
   //  FIX 1: Split 8MB target staging — 4MB per channel
   //  ch0: TARGET_STAGING_ADDR + 0         = 131072 slots
   //  ch1: TARGET_STAGING_ADDR + 0x400000  = 131072 slots
   // -----------------------------------------------------------
   RawPointPacket *target_staging_0 = (RawPointPacket *)TARGET_STAGING_ADDR;
   RawPointPacket *target_staging_1 = (RawPointPacket *)(TARGET_STAGING_ADDR + 0x400000);

   // -----------------------------------------------------------
   //  FIX 2: Split staging IDs — 512KB per channel (131072 ints)
   //  ch0: STAGING_IDS_ADDR              (0x82800000)
   //  ch1: STAGING_IDS_ADDR + 0x80000    (0x82880000)
   // -----------------------------------------------------------
   int *staging_ids_0 = (int *)STAGING_IDS_ADDR;
   int *staging_ids_1 = (int *)(STAGING_IDS_ADDR + MAX_STAGING_SLOTS * sizeof(int));

   // Anchor staging — 64KB per channel (2048 slots)
   RawPointPacket *anchor_staging_0 = (RawPointPacket *)ANCHOR_STAGING_ADDR;
   RawPointPacket *anchor_staging_1 = (RawPointPacket *)(ANCHOR_STAGING_ADDR
                                       + MAX_ANCHORS_PER_TX * sizeof(RawPointPacket));

   // Result buffers — 31MB per channel
   DistPacket *all_results_0 = (DistPacket *)RESULTS_ADDR;
   int rx_buf_half = (int)((MATCHES_ADDR - RESULTS_ADDR) / 2);
   DistPacket *all_results_1 = (DistPacket *)(RESULTS_ADDR + rx_buf_half);

   // Query bin assignment and sort
   QueryBinKey *query_bins = (QueryBinKey *)QUERY_SORT_ADDR;
   for (int qi = 0; qi < query_count; qi++) {
       int qbx=(int)((query_coords[qi].d[top1_id]-coord_vals[top1_id].cord_min)*inv_bx);
       int qby=(int)((query_coords[qi].d[top2_id]-coord_vals[top2_id].cord_min)*inv_by);
       int qbz=(int)((query_coords[qi].d[top3_id]-coord_vals[top3_id].cord_min)*inv_bz);
       if(qbx>=nbx) qbx=nbx-1; if(qbx<0) qbx=0;
       if(qby>=nby) qby=nby-1; if(qby<0) qby=0;
       if(qbz>=nbz) qbz=nbz-1; if(qbz<0) qbz=0;
       query_bins[qi].bin_id    = BIN3D(qbx,qby,qbz,nby,nbz);
       query_bins[qi].query_idx = qi;
   }
   qsort(query_bins, query_count, sizeof(QueryBinKey), compare_query_bins);

   int match_count = 0;
   u32 *point_epoch = (u32 *)POINT_EPOCH_ADDR;
   memset(point_epoch, 0, total_points * sizeof(u32));
   u32 current_epoch = 1;
   int qi_group_start = 0;

   while (qi_group_start < query_count) {

       int batches_to_run = 0;
       int batch_target_counts[2] = { 0, 0 };
       int batch_query_counts[2]  = { 0, 0 };
       int qi_group_starts[2]     = { qi_group_start, 0 };
       int qi_group_ends[2]       = { 0, 0 };

       // =======================================================
       //  STEP 1: Build up to 2 independent batches
       //
       //  FIX 3: Increment epoch PER CHANNEL so ch0 and ch1
       //  have independent deduplication. Without this, ch1
       //  skips targets already staged by ch0 → missing matches.
       //
       //  FIX 4: Overflow threshold raised from MAX_BRAM_CAPACITY
       //  (2500) to MAX_STAGING_SLOTS (131072) so large-radius
       //  neighborhoods (>2500 targets) are staged correctly.
       //  The targets_remaining loop below sends them to HW
       //  in MAX_BRAM_CAPACITY chunks.
       // =======================================================
       for (int ch = 0; ch < 2; ch++) {
           if (qi_group_start >= query_count) break;

           // FIX 3: one epoch increment per channel
           current_epoch++;

           qi_group_starts[ch]  = qi_group_start;
           int qi_current_group = qi_group_start;
           int local_target_count = 0;
           int local_query_count  = 0;

           RawPointPacket *t_stg  = (ch == 0) ? target_staging_0 : target_staging_1;
           int            *s_ids  = (ch == 0) ? staging_ids_0    : staging_ids_1;

           while (qi_current_group < query_count) {
               int cur_bin    = query_bins[qi_current_group].bin_id;
               int qi_grp_end = qi_current_group + 1;
               while (qi_grp_end < query_count &&
                      query_bins[qi_grp_end].bin_id == cur_bin) qi_grp_end++;
               int group_size = qi_grp_end - qi_current_group;

               int qbz = cur_bin % nbz;
               int qby = (cur_bin / nbz) % nby;
               int qbx = cur_bin / (nby * nbz);

               float q_d4_min = query_coords[query_bins[qi_current_group].query_idx].d[top4_id];
               float q_d4_max = q_d4_min;
               for (int g = 1; g < group_size; g++) {
                   float d4 = query_coords[query_bins[qi_current_group+g].query_idx].d[top4_id];
                   if (d4 < q_d4_min) q_d4_min = d4;
                   if (d4 > q_d4_max) q_d4_max = d4;
               }
               float lo_valid = q_d4_min - MAX_RANGE;
               float hi_valid = q_d4_max + MAX_RANGE;

               int new_targets = 0;
               for (int dx = -1; dx <= 1; dx++) {
               for (int dy = -1; dy <= 1; dy++) {
               for (int dz = -1; dz <= 1; dz++) {
                   int nx=qbx+dx, ny2=qby+dy, nz=qbz+dz;
                   if (nx<0||nx>=nbx||ny2<0||ny2>=nby||nz<0||nz>=nbz) continue;
                   int nb     = BIN3D(nx,ny2,nz,nby,nbz);
                   int nb_idx = binary_search_bins(unique_bins, unique_bin_count, nb);
                   if (nb_idx == -1) continue;
                   int nb_off = unique_bins[nb_idx].offset;
                   int nb_cnt = unique_bins[nb_idx].count;
                   int sp = binary_search_bins_points(main_points_array,nb_off,nb_cnt,lo_valid,hi_valid,1,top4_id);
                   int ep = binary_search_bins_points(main_points_array,nb_off,nb_cnt,lo_valid,hi_valid,0,top4_id);
                   if (ep >= sp && sp < nb_cnt && ep >= 0) {
                       for (int p = sp; p <= ep; p++) {
                           int orig_id = main_points_array[nb_off+p].original_id;
                           if (point_epoch[orig_id] != current_epoch) {
                               point_epoch[orig_id] = current_epoch;
                               t_stg[local_target_count + new_targets] =
                                   *(RawPointPacket *)&main_points_array[nb_off+p];
                               s_ids[local_target_count + new_targets] = orig_id;
                               new_targets++;
                           }
                       }
                   }
               }}}

               // FIX 4: overflow guard against the LARGE staging buffer
               if (local_target_count + new_targets > MAX_STAGING_SLOTS && local_query_count > 0) {
                   for (int i = 0; i < new_targets; i++)
                       point_epoch[s_ids[local_target_count + i]] = 0;
                   break;
               }

               local_target_count += new_targets;
               local_query_count  += group_size;
               qi_current_group    = qi_grp_end;

               if (local_target_count >= MAX_STAGING_SLOTS) break;
           }

           if (local_target_count == 0) {
               qi_group_start = qi_current_group;
               break;
           }

           batch_target_counts[ch] = local_target_count;
           batch_query_counts[ch]  = local_query_count;
           qi_group_ends[ch]       = qi_current_group;
           qi_group_start          = qi_current_group;
           batches_to_run++;
       }

       if (batches_to_run == 0) continue;

       // =======================================================
       //  STEP 2: HW DISPATCH — one channel at a time with
       //  targets_remaining loop.
       //
       //  FIX 5: targets_remaining loop sends targets to HW in
       //  MAX_BRAM_CAPACITY slices. This fixes the IP BRAM
       //  overflow when a single query group has > 2500 targets
       //  (which happens with larger search radii).
       //
       //  FIX 6: max_safe_anchors computed from rx_buf_half and
       //  the ACTUAL current_bite (not an undefined variable).
       //
       //  FIX 7: staging ID lookup uses target_offset so that
       //  res.target_id (which is slice-relative 0..current_bite-1)
       //  correctly indexes into the full staging IDs array.
       // =======================================================
       for (int ch = 0; ch < batches_to_run; ch++) {
           u32 active_ip  = (ch == 0) ? IP_0_BASE  : IP_1_BASE;
           u32 active_dma = (ch == 0) ? DMA_0_BASE : DMA_1_BASE;
           RawPointPacket *t_stg  = (ch == 0) ? target_staging_0 : target_staging_1;
           int            *s_ids  = (ch == 0) ? staging_ids_0    : staging_ids_1;
           RawPointPacket *a_stg  = (ch == 0) ? anchor_staging_0 : anchor_staging_1;
           DistPacket     *res_p  = (ch == 0) ? all_results_0    : all_results_1;

           int targets_remaining = batch_target_counts[ch];
           int target_offset     = 0;

           while (targets_remaining > 0) {
               int current_bite = (targets_remaining > MAX_BRAM_CAPACITY)
                                  ? MAX_BRAM_CAPACITY : targets_remaining;

               // --- MODE 0: load this target slice into BRAM ---
               Xil_DCacheFlushRange((UINTPTR)&t_stg[target_offset],
                                    current_bite * sizeof(RawPointPacket));
               Xil_Out32(active_ip + 0x10, current_bite);
               Xil_Out32(active_ip + 0x18, num_dims);
               Xil_Out32(active_ip + 0x20, max_range_bits);
               Xil_Out32(active_ip + 0x30, 0);
               Xil_Out32(active_ip + 0x28, 0x00);
               Xil_Out32(active_ip + 0x00, 0x01);
               Xil_Out32(active_dma + 0x00, 0x0001);
               Xil_Out32(active_dma + 0x18, (UINTPTR)&t_stg[target_offset]);
               Xil_Out32(active_dma + 0x28, current_bite * sizeof(RawPointPacket));
               dma_tx_wait(active_dma, "M0 target");
               ip_wait_idle(active_ip,  "M0 IP");

               // FIX 6: compute max_safe from actual current_bite and per-channel buffer
               int max_safe = ((rx_buf_half / ((current_bite+1)*(int)sizeof(DistPacket)))
                               * UNROLL_FACTOR);
               if (max_safe > MAX_ANCHORS_PER_TX) max_safe = MAX_ANCHORS_PER_TX;
               if (max_safe < UNROLL_FACTOR)       max_safe = UNROLL_FACTOR;

               int current_a_bite  = (batch_query_counts[ch] > max_safe)
                                     ? max_safe : batch_query_counts[ch];
               int padded_a_count  = ((current_a_bite + UNROLL_FACTOR - 1)
                                      / UNROLL_FACTOR) * UNROLL_FACTOR;

               for (int a = 0; a < padded_a_count; a++) {
                   int la = (a < batch_query_counts[ch]) ? a : batch_query_counts[ch]-1;
                   a_stg[a] = query_coords[query_bins[qi_group_starts[ch]+la].query_idx];
               }
               int tx_bytes    = padded_a_count * sizeof(RawPointPacket);
               Xil_DCacheFlushRange((UINTPTR)a_stg, tx_bytes);

               int num_batches   = padded_a_count / UNROLL_FACTOR;
               int expected_rx   = (num_batches * current_bite + 1) * (int)sizeof(DistPacket);
               total_calcs      += (unsigned long long)current_a_bite * current_bite;

               // --- MODE 1: stream anchors, arm RX first ---
               Xil_Out32(active_ip + 0x10, current_bite);
               Xil_Out32(active_ip + 0x18, num_dims);
               Xil_Out32(active_ip + 0x20, max_range_bits);
               Xil_Out32(active_ip + 0x30, padded_a_count);
               Xil_Out32(active_ip + 0x28, 0x01);
               Xil_Out32(active_ip + 0x00, 0x01);
               Xil_Out32(active_dma + 0x30, 0x0001);
               Xil_Out32(active_dma + 0x48, (UINTPTR)res_p);
               Xil_Out32(active_dma + 0x58, expected_rx);
               Xil_Out32(active_dma + 0x00, 0x0001);
               Xil_Out32(active_dma + 0x18, (UINTPTR)a_stg);
               Xil_Out32(active_dma + 0x28, tx_bytes);
               dma_tx_wait(active_dma, "M1 TX");
               dma_rx_wait(active_dma, "M1 RX");

               // --- Parse results ---
               int actual_rx = Xil_In32(active_dma + 0x58);
               Xil_DCacheInvalidateRange((UINTPTR)res_p, actual_rx);
               int rx_pkts = actual_rx / (int)sizeof(DistPacket);

               for (int p = 0; p < rx_pkts; p++) {
                   DistPacket res = res_p[p];
                   if (res.target_id < 0) break;

                   int b     = res.anchor_b_idx;
                   int a_idx = b * UNROLL_FACTOR;

                   for (int a = 0; a < UNROLL_FACTOR; a++) {
                       if (a_idx + a >= current_a_bite) continue;
                       float ds = res.distances[a];
                       if (ds < 0.0f || ds > max_range_sq) continue;
                       if (match_count < MAX_MATCHES) {
                           // FIX 7: add target_offset so ID lookup is slice-relative
                           // int orig_target = s_ids[target_offset + res.target_id];
                           // int orig_query  = query_bins[qi_group_starts[ch]+a_idx+a].query_idx;
                           match_count++;
                       }
                   }
               }

               targets_remaining -= current_bite;
               target_offset     += current_bite;
           }
       }
   }

   // =========================================================
   //  RESULTS
   // =========================================================
   u64 time_end_hw = get_timer_value(&TimerInstance);
   xil_printf(" M: %d\n\r", match_count);

   unsigned int calcs_hi = (unsigned int)(total_calcs >> 32);
   unsigned int calcs_lo = (unsigned int)(total_calcs & 0xFFFFFFFFUL);
   if (calcs_hi > 0) xil_printf(" C   %u%09u\n\r", calcs_hi, calcs_lo);
   else              xil_printf(" C   %u\n\r",      calcs_lo);

   u64 hw_ticks = time_end_hw - time_start_hw;
   u64 hw_sec   = hw_ticks / TIMER_FREQ_HZ;
   u32 hw_ms    = (u32)((hw_ticks % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ/1000));
   u32 hw_min   = (u32)(hw_sec / 60);
   u32 hw_s     = (u32)(hw_sec % 60);
   xil_printf("HW: %lu min %lu.%03lu sec\n\r", hw_min, hw_s, hw_ms);

   while(1);
   return 0;
}