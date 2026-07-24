#include "xil_printf.h"
#include "xil_io.h"
#include "xparameters.h"
#include "xil_cache.h"
#include "xtmrctr.h"
#include <math.h>
#include <stdlib.h>

// =============================================================
//  MEMORY MAP (8MB ANTI-CORRUPTION SIZING FOR RADIUS 20)
//  STRICTLY PACKED FOR 128MB DDR2
// =============================================================
// =============================================================
//  BULLETPROOF MEMORY MAP (Strictly Packed for 128MB DDR)
// =============================================================
#define DDR_BASE_ADDR         0x80000000 // Main Raw (5 MB)
#define QUERY_BASE_ADDR       0x80800000 // Query Raw (1 MB)
#define MAIN_POINTS_ADDR      0x81000000 // Main Array (7 MB)
#define UNIQUE_BINS_ADDR      0x81800000 // Bin Array (1 MB)

// Staging Buffers
#define TARGET_STAGING_ADDR   0x82000000 // Target Buffer (8 MB)
#define STAGING_IDS_ADDR      0x82800000 // Target IDs (1 MB)
#define ANCHOR_STAGING_ADDR   0x82900000 // Anchor Buffer (1 MB)
#define COORD_INFO_ADDR       0x82A00000 // Grid/Radix Info (1 MB)

// Sorting Keys
#define SORT_KEY_ADDR         0x82C00000 // Main Sort Keys (3 MB)
#define QUERY_SORT_ADDR       0x83000000 // Query Sort Keys (1 MB)
// Add this below your existing memory map #defines
#define POINT_EPOCH_ADDR      0x83400000 // Epoch array for O(1) deduplication (4MB)
// DMA Firehose Space
#define RESULTS_ADDR          0x84000000 // DMA RX Start (62 MB Safe Space)
#define MATCHES_ADDR          0x87E00000 // DMA RX Ceiling Limit

#define DMA_BASE              0x41E00000
#define IP_BASE               0x00008000

#define IP_CTRL              (IP_BASE + 0x00)
#define IP_NUM_POINTS        (IP_BASE + 0x10)
#define IP_NUM_DIMS          (IP_BASE + 0x18)
#define IP_MAX_RANGE         (IP_BASE + 0x20)
#define IP_MODE              (IP_BASE + 0x28)
#define IP_NUM_ANCHORS       (IP_BASE + 0x30)

// =============================================================
//  CONSTANTS
// =============================================================
#define MAX_RANGE             15.0f
#define MAX_BRAM_CAPACITY     5000
#define MAX_MATCHES           2000000000
#define WATCHDOG_LIMIT        20000000
#define UNROLL_FACTOR         4
#define TIMER_FREQ_HZ         200000000ULL

// UNLEASH THE DMA FIREHOSE
#define MAX_ANCHORS_PER_TX    2048

// =============================================================
//  DATA TYPES
// =============================================================
typedef struct { float d[8]; } RawPointPacket;
typedef struct { float d[8]; int bin_id; int original_id; } BinPointPacket;
typedef struct { int bin_id; int offset; int count; } BinInfo;

typedef struct {
    float distances[UNROLL_FACTOR];
    int target_id;
    int anchor_b_idx;
    int pad1, pad2;
} DistPacket;
typedef struct { int bin_id; int query_idx; } QueryBinKey;


typedef struct { int bin_id; int orig_idx; } SortKey;
typedef struct { int cord_id; float cord_min; float cord_max; float cord_spread; } CordInfo;
typedef struct { int anchor_id; int target_id; float distance; } MatchRecord;

#define BIN3D(bx, by, bz, nby, nbz) ((bx)*((nby)*(nbz)) + (by)*(nbz) + (bz))

// =============================================================
//  HELPERS
// =============================================================

u64 get_timer_value(XTmrCtr *TimerInstancePtr) {
    u32 upper1 = XTmrCtr_GetValue(TimerInstancePtr, 1);
    u32 lower = XTmrCtr_GetValue(TimerInstancePtr, 0);
    u32 upper2 = XTmrCtr_GetValue(TimerInstancePtr, 1);
    if (upper1 != upper2)
        lower = XTmrCtr_GetValue(TimerInstancePtr, 0);
    return (((u64) upper2) << 32) | (u64) lower;
}

int binary_search_bins(BinInfo *arr, int count, int target_id) {
    int low = 0, high = count - 1;
    while (low <= high) {
    	   int mid = low + ((high - low) >> 1);
        if (arr[mid].bin_id == target_id) return mid;
        else if (arr[mid].bin_id < target_id) low = mid + 1;
        else high = mid - 1;
    }
    return -1;
}
static int g_sort_dim = 4;
int binary_search_bins_points(BinPointPacket *arr, int nb_offset, int nb_count,
        float lowest_valid, float highest_valid, int is_start, int sort_dim) {
    int low = 0, high = nb_count - 1;
    int ans = is_start ? nb_count : -1;
    while (low <= high) {
    	   int mid = low + ((high - low) >> 1);
        if (is_start) {
            if (arr[nb_offset + mid].d[sort_dim] >= lowest_valid) { ans = mid; high = mid - 1; }
            else { low = mid + 1; }
        } else {
            if (arr[nb_offset + mid].d[sort_dim] <= highest_valid) { ans = mid; low = mid + 1; }
            else { high = mid - 1; }
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

static inline void dma_tx_wait(const char *label) {
    int wd = 0;
    while (!(Xil_In32(DMA_BASE + 0x04) & 0x0002)) {
        if (wd++ > WATCHDOG_LIMIT) {
//        	 xil_printf("[FATAL] DMA TX Jam: %s\n\r", label);
        	while (1); }
    }
}

static inline void dma_rx_wait(const char *label) {
    int wd = 0;
    while (!(Xil_In32(DMA_BASE + 0x34) & 0x0002)) {
        if (wd++ > WATCHDOG_LIMIT) {
//        	   xil_printf("[FATAL] DMA RX Halt: %s\n\r", label);
        	while (1); }
    }
}

static inline void ip_wait_idle(const char *label) {
    int wd = 0;
    while (!(Xil_In32(IP_CTRL) & 0x04)) {
        if (wd++ > WATCHDOG_LIMIT) {
//        	   xil_printf("[FATAL] IP stuck: %s\n\r", label);
        	while (1); }
    }
}

// =============================================================
//  MAIN
// =============================================================
int main() {
    XTmrCtr TimerInstance;
    if (XTmrCtr_Initialize(&TimerInstance, XPAR_TMRCTR_0_DEVICE_ID) != XST_SUCCESS) {
//        xil_printf("Timer init failed!\n\r");
    }
    XTmrCtr_SetOptions(&TimerInstance, 0, XTC_CASCADE_MODE_OPTION);
    XTmrCtr_Start(&TimerInstance, 0);

    u64 time_start_total = get_timer_value(&TimerInstance);

    int *file_header = (int *) DDR_BASE_ADDR;
    int num_dims = file_header[0];
    int total_points = file_header[1];

//    xil_printf("Dims=%d  Points=%d\n\r", num_dims, total_points);
//    xil_printf("MAX_RANGE=%d \n\r", (int) MAX_RANGE);

    int *query_header  = (int *)QUERY_BASE_ADDR;
//    int  query_dims    = query_header[0];
    int  query_count   = query_header[1];
    RawPointPacket *query_coords = (RawPointPacket *)(QUERY_BASE_ADDR + 8);

//    xil_printf("Query: Dims=%d  Points=%d\n\r", query_dims, query_count);

    RawPointPacket *all_coords = (RawPointPacket *) (DDR_BASE_ADDR + 8);
    BinPointPacket *main_points_array = (BinPointPacket *) MAIN_POINTS_ADDR;
    int unique_bin_count = 0;
    unsigned long long total_calcs = 0;

    CordInfo *coord_vals = (CordInfo *) COORD_INFO_ADDR;

    for (int dim = 0; dim < 8; dim++) {
        coord_vals[dim].cord_id = dim;
        coord_vals[dim].cord_min = 999999.0f;
        coord_vals[dim].cord_max = -999999.0f;
    }

    for (int i = 0; i < total_points; i++) {
        main_points_array[i].bin_id = 0;
        main_points_array[i].original_id = i;
        for (int dim = 0; dim < 8; dim++) {
            float d = all_coords[i].d[dim];
            main_points_array[i].d[dim] = d;
            if (d < coord_vals[dim].cord_min) coord_vals[dim].cord_min = d;
            if (d > coord_vals[dim].cord_max) coord_vals[dim].cord_max = d;
        }
    }

    for (int dim = 0; dim < 8; dim++) {
        coord_vals[dim].cord_spread = (coord_vals[dim].cord_max - coord_vals[dim].cord_min) + 0.0001f;
    }

    int top1_id = -1, top2_id = -1, top3_id = -1, top4_id = -1;
    float top1_spread = -1.0f, top2_spread = -1.0f, top3_spread = -1.0f, top4_spread = -1.0f;
    for (int j = 0; j < num_dims; j++) {
        float current_spread = coord_vals[j].cord_spread;
        int current_id = coord_vals[j].cord_id;
        if (current_spread > top1_spread) {
         	top4_spread = top3_spread; top4_id = top3_id;
            top3_spread = top2_spread; top3_id = top2_id;
            top2_spread = top1_spread; top2_id = top1_id;
            top1_spread = current_spread; top1_id = current_id;
        } else if (current_spread > top2_spread) {
        	top4_spread = top3_spread; top4_id = top3_id;
            top3_spread = top2_spread; top3_id = top2_id;
            top2_spread = current_spread; top2_id = current_id;
        } else if (current_spread > top3_spread) {
        	top4_spread = top3_spread; top4_id = top3_id;
            top3_spread = current_spread; top3_id = current_id;
        }
        else if(current_spread > top4_spread){
        	top4_spread = current_spread; top4_id = current_id;
        }
    }
    xil_printf("top1=%d top2=%d top3=%d top4=%d\n\r",
               top1_id, top2_id, top3_id, top4_id);
    float spread_x = top1_spread;
    float spread_y = top2_spread;
    float spread_z = top3_spread;

    float max_range = MAX_RANGE;
    union { float f; u32 u; } f2u;
    f2u.f = max_range;
    u32 max_range_bits = f2u.u;
    float max_range_sq = max_range * max_range;

    float voxel_size = MAX_RANGE;

    int nbx = (int)(spread_x / voxel_size); if (nbx < 1) nbx = 1;
    int nby = (num_dims > 1) ? (int)(spread_y / voxel_size) : 1; if (nby < 1) nby = 1;
    int nbz = (num_dims > 2) ? (int)(spread_z / voxel_size) : 1; if (nbz < 1) nbz = 1;

    long long total_voxels = (long long)nbx * nby * nbz;

    if (total_voxels > 2000000000LL) {
        float scale = 1.0f;
        while (total_voxels > 2000000000LL) {
            scale *= 1.5f;
            nbx = (int)(spread_x / (voxel_size * scale)); if (nbx < 1) nbx = 1;
            nby = (num_dims > 1) ? (int)(spread_y / (voxel_size * scale)) : 1; if (nby < 1) nby = 1;
            nbz = (num_dims > 2) ? (int)(spread_z / (voxel_size * scale)) : 1; if (nbz < 1) nbz = 1;
            total_voxels = (long long)nbx * nby * nbz;
        }
        voxel_size = voxel_size * scale;
    }

    float inv_bx = (float) nbx / spread_x;
    float inv_by = (float) nby / spread_y;
    float inv_bz = (float) nbz / spread_z;

//    xil_printf("[P2] Grid: %d x %d x %d\n\r", nbx, nby, nbz);
//    xil_printf("[P3] Assigning bins...\n\r");

    for (int i = 0; i < total_points; i++) {
        int bx = (int) ((main_points_array[i].d[top1_id] - coord_vals[top1_id].cord_min) * inv_bx);
        if (bx >= nbx) bx = nbx - 1; if (bx < 0) bx = 0;
        int by = (num_dims > 1) ? (int) ((main_points_array[i].d[top2_id] - coord_vals[top2_id].cord_min) * inv_by) : 0;
        if (by >= nby) by = nby - 1; if (by < 0) by = 0;
        int bz = (num_dims > 2) ? (int) ((main_points_array[i].d[top3_id] - coord_vals[top3_id].cord_min) * inv_bz) : 0;
        if (bz >= nbz) bz = nbz - 1; if (bz < 0) bz = 0;
        main_points_array[i].bin_id = BIN3D(bx, by, bz, nby, nbz);
    }

//    xil_printf("[P4] Sorting...\n\r");
//    qsort(main_points_array, total_points, sizeof(BinPointPacket), compare_bins);

//    xil_printf("[P4] Radix sorting...\n\r");

    SortKey *sort_keys = (SortKey *)SORT_KEY_ADDR;
    SortKey *temp_keys = sort_keys + total_points;  // immediately after sort_keys
    int     *cnt       = (int *)(COORD_INFO_ADDR + 256);  // 256KB count array

    // Build sort key array
    for (int i = 0; i < total_points; i++) {
        sort_keys[i].bin_id   = main_points_array[i].bin_id;
        sort_keys[i].orig_idx = i;
    }

    // Pass 1: sort by lower 16 bits of bin_id
    memset(cnt, 0, 65536 * sizeof(int));
    for (int i = 0; i < total_points; i++)
        cnt[sort_keys[i].bin_id & 0xFFFF]++;
    for (int i = 1; i < 65536; i++)
        cnt[i] += cnt[i-1];
    for (int i = total_points - 1; i >= 0; i--)
        temp_keys[--cnt[sort_keys[i].bin_id & 0xFFFF]] = sort_keys[i];

    // Pass 2: sort by upper 16 bits of bin_id
    memset(cnt, 0, 65536 * sizeof(int));
    for (int i = 0; i < total_points; i++)
        cnt[(temp_keys[i].bin_id >> 16) & 0xFFFF]++;
    for (int i = 1; i < 65536; i++)
        cnt[i] += cnt[i-1];
    for (int i = total_points - 1; i >= 0; i--)
        sort_keys[--cnt[(temp_keys[i].bin_id >> 16) & 0xFFFF]] = temp_keys[i];

    // Reorder main_points_array using sorted indices
    // Reuse RESULTS_ADDR as temp buffer (free during pre-HW phase)
    BinPointPacket *temp_pts = (BinPointPacket *)RESULTS_ADDR;
    for (int i = 0; i < total_points; i++)
        temp_pts[i] = main_points_array[sort_keys[i].orig_idx];
    for (int i = 0; i < total_points; i++)
        main_points_array[i] = temp_pts[i];

    int bin_start = 0;
    int current_b = main_points_array[0].bin_id;
    g_sort_dim = top4_id;
    for (int i = 1; i <= total_points; i++) {
        int b = (i < total_points) ? main_points_array[i].bin_id : -1;
        if (b != current_b) {
            // Sort this bin's slice by d[4]
            int bin_len = i - bin_start;
            if (bin_len > 1) {
                qsort(&main_points_array[bin_start], bin_len,
                      sizeof(BinPointPacket), compare_d4);
            }
            bin_start = i;
            current_b = b;
        }
    }

//    xil_printf("[P4] Done.\n\r");

    BinInfo *unique_bins = (BinInfo *) UNIQUE_BINS_ADDR;
    int current_bin = -1;
    for (int i = 0; i < total_points; i++) {
        int b = main_points_array[i].bin_id;
        if (b != current_bin) {
            current_bin = b;
            unique_bins[unique_bin_count].bin_id = b;
            unique_bins[unique_bin_count].offset = i;
            unique_bins[unique_bin_count].count = 1;
            unique_bin_count++;
        } else {
            unique_bins[unique_bin_count - 1].count++;
        }
    }
//    xil_printf("[P5] Found %d bins.\n\r", unique_bin_count);
//    int avg_count = total_points / (unique_bin_count + 1);
//
//    int cpu_threshold;
//    if      (avg_count <= 2)  cpu_threshold = 300;
//    else if (avg_count <= 15) cpu_threshold = 20;
//    else                      cpu_threshold = 4;
//
//    xil_printf("avg_count=%d  cpu_threshold=%d\n\r",
//               avg_count, cpu_threshold);

    // ==========================================================
        //  P6 HARDWARE LOOP — RANGE JOIN (COMPLETE WITH DEBUG)
        //  One query point per iteration, dedup targets, no dedup
        //  between query and main (different sets)
        // ==========================================================

    // =============================================================
    //  ADD THIS DEFINE TO YOUR MEMORY MAP SECTION:
    //  Query sort keys: 20k × 8 bytes = 160KB
    //  Place after SORT_KEY_ADDR which ends at ~0x87700000 + 2.4MB
    // =============================================================
    //  #define QUERY_BASE_ADDR    0x87700000   // already defined
    //  #define QUERY_SORT_ADDR    0x87780000   // 20k × 8B = 160KB

    // =============================================================
    //  ADD THIS TYPEDEF before main() alongside your other typedefs:
    // =============================================================
    //  typedef struct { int bin_id; int query_idx; } QueryBinKey;

    // =============================================================
    //  P6 HARDWARE LOOP — RANGE JOIN WITH QUERY BINNING
    //
    //  CONCEPT:
    //  Instead of looping over 20,000 query points one-by-one,
    //  we first assign each query point to its main-dataset bin,
    //  sort query points by bin_id, then process groups of query
    //  points that share the same neighbourhood together.
    //
    //  This means:
    //  - collect targets ONCE per unique query bin (not per query point)
    //  - mode0 fires ONCE per unique query bin
    //  - mode1 sends all query points in that bin as anchors in batches
    //
    //  SPEEDUP:
    //  Current:   20,000 mode0 calls
    //  After:     ~15,000 mode0 calls (groups of 2+ share targets)
    //  Dense/clustered query data: potentially 5-10x fewer mode0 calls
    // =============================================================
//    xil_printf("[P6 ..\n\r");
            u64 time_start_hw = get_timer_value(&TimerInstance);

            Xil_Out32(DMA_BASE + 0x04, 0x1000);
            Xil_Out32(DMA_BASE + 0x34, 0x1000);
            Xil_Out32(DMA_BASE + 0x00, 0x0001);
            Xil_Out32(DMA_BASE + 0x30, 0x0001);

            RawPointPacket *target_staging = (RawPointPacket *)TARGET_STAGING_ADDR;
            RawPointPacket *anchor_staging = (RawPointPacket *)ANCHOR_STAGING_ADDR;
            int            *staging_ids    = (int *)STAGING_IDS_ADDR;
            DistPacket     *all_results    = (DistPacket *)RESULTS_ADDR;

//            xil_printf("Query Points=%d\n\r", query_count);

            // ==========================================================
            //  STEP A: Assign and Sort Query Bins
            // ==========================================================
            QueryBinKey *query_bins = (QueryBinKey *)QUERY_SORT_ADDR;

            for (int qi = 0; qi < query_count; qi++) {
                float qx = query_coords[qi].d[top1_id];
                float qy = query_coords[qi].d[top2_id];
                float qz = query_coords[qi].d[top3_id];

                int qbx = (int)((qx - coord_vals[top1_id].cord_min) * inv_bx);
                int qby = (int)((qy - coord_vals[top2_id].cord_min) * inv_by);
                int qbz = (int)((qz - coord_vals[top3_id].cord_min) * inv_bz);
                if (qbx >= nbx) qbx = nbx-1; if (qbx < 0) qbx = 0;
                if (qby >= nby) qby = nby-1; if (qby < 0) qby = 0;
                if (qbz >= nbz) qbz = nbz-1; if (qbz < 0) qbz = 0;

                query_bins[qi].bin_id    = BIN3D(qbx, qby, qbz, nby, nbz);
                query_bins[qi].query_idx = qi;
            }

            extern int compare_query_bins(const void *, const void *);
            qsort(query_bins, query_count, sizeof(QueryBinKey), compare_query_bins);
//            xil_printf("[P6] Query bins sorted.\n\r");

            int match_count = 0;

            // ==========================================================
            //  STEP B: Query Batching & Fast CPU Target Deduplication
            // ==========================================================

            // Initialize O(1) Deduplication Tracker
            u32 *point_epoch = (u32 *)POINT_EPOCH_ADDR;
            memset(point_epoch, 0, total_points * sizeof(u32));
            u32 current_epoch = 1;

            int qi_group_start = 0;

            while (qi_group_start < query_count) {
                current_epoch++; // Advance epoch to start fresh without memset(0)

                int batch_target_count = 0;
                int batch_query_count = 0;
                int qi_current_group = qi_group_start;

                // Accumulate query groups until we hit BRAM capacity
                while (qi_current_group < query_count) {
                    int current_bin_id = query_bins[qi_current_group].bin_id;

                    // 1. Identify group boundaries
                    int qi_group_end = qi_current_group + 1;
                    while (qi_group_end < query_count && query_bins[qi_group_end].bin_id == current_bin_id) {
                        qi_group_end++;
                    }
                    int group_size = qi_group_end - qi_current_group;

                    int qbz = current_bin_id % nbz;
                    int qby = (current_bin_id / nbz) % nby;
                    int qbx = current_bin_id / (nby * nbz);

                    // 4th-dim pruning calculation for THIS specific group
                    float q_d4_min = query_coords[query_bins[qi_current_group].query_idx].d[top4_id];
                    float q_d4_max = q_d4_min;
                    for (int g = 1; g < group_size; g++) {
                        float d4 = query_coords[query_bins[qi_current_group + g].query_idx].d[top4_id];
                        if (d4 < q_d4_min) q_d4_min = d4;
                        if (d4 > q_d4_max) q_d4_max = d4;
                    }
                    float lowest_valid  = q_d4_min - MAX_RANGE;
                    float highest_valid = q_d4_max + MAX_RANGE;

                    // 2. Gather Unique Targets using Epoch Array
                    int new_targets = 0;

                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            for (int dz = -1; dz <= 1; dz++) {
                                int nx = qbx + dx;
                                int ny2 = qby + dy;
                                int nz = qbz + dz;
                                if (nx<0||nx>=nbx||ny2<0||ny2>=nby||nz<0||nz>=nbz) continue;

                                int nb     = BIN3D(nx, ny2, nz, nby, nbz);
                                int nb_idx = binary_search_bins(unique_bins, unique_bin_count, nb);
                                if (nb_idx == -1) continue;

                                int nb_offset = unique_bins[nb_idx].offset;
                                int nb_count  = unique_bins[nb_idx].count;

                                int start_p = binary_search_bins_points(main_points_array, nb_offset, nb_count, lowest_valid, highest_valid, 1, top4_id);
                                int end_p   = binary_search_bins_points(main_points_array, nb_offset, nb_count, lowest_valid, highest_valid, 0, top4_id);

                                if (end_p >= start_p && start_p < nb_count && end_p >= 0) {
                                    for (int p = start_p; p <= end_p; p++) {
                                        int orig_id = main_points_array[nb_offset + p].original_id;

                                        // O(1) CPU DEDUPLICATION
                                        if (point_epoch[orig_id] != current_epoch) {
                                            point_epoch[orig_id] = current_epoch; // Tentatively mark

                                            target_staging[batch_target_count + new_targets] = *(RawPointPacket *)&main_points_array[nb_offset + p];
                                            staging_ids[batch_target_count + new_targets] = orig_id;
                                            new_targets++;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // 3. Evaluate Batch Capacity
                    // If adding these targets overflows BRAM (and we already have valid queries in the batch),
                    // break to process the current batch and save this group for the next iteration.
                    if (batch_target_count + new_targets > MAX_BRAM_CAPACITY && batch_query_count > 0) {
                        // Rollback tentative epoch markings so they can be picked up in the next loop
                        for (int i = 0; i < new_targets; i++) {
                            int orig_id = staging_ids[batch_target_count + i];
                            point_epoch[orig_id] = 0;
                        }
                        break;
                    }

                    // Otherwise, formally commit this group to the batch
                    batch_target_count += new_targets;
                    batch_query_count += group_size;
                    qi_current_group = qi_group_end;

                    // If we exactly hit the limit, jump out and process
                    if (batch_target_count >= MAX_BRAM_CAPACITY) {
                        break;
                    }
                }

                // Skip empty batches (if spatial queries yield zero targets)
                if (batch_target_count == 0) {
                    qi_group_start = qi_current_group;
                    continue;
                }

                // --------------------------------------------------------
                //  MODE 0: Load deduplicated batch targets into FPGA
                // --------------------------------------------------------
                int targets_remaining = batch_target_count;
                int target_offset     = 0;

                while (targets_remaining > 0) {
                    // If a single query group was larger than MAX_BRAM, handle the overflow chunking
                    int current_bite = (targets_remaining > MAX_BRAM_CAPACITY) ? MAX_BRAM_CAPACITY : targets_remaining;

                    Xil_DCacheFlushRange((UINTPTR)&target_staging[target_offset], current_bite * sizeof(RawPointPacket));

                    Xil_Out32(IP_NUM_POINTS,  current_bite);
                    Xil_Out32(IP_NUM_DIMS,    num_dims);
                    Xil_Out32(IP_MAX_RANGE,   max_range_bits);
                    Xil_Out32(IP_NUM_ANCHORS, 0);
                    Xil_Out32(IP_MODE,        0x00);
                    Xil_Out32(IP_CTRL,        0x01);

                    Xil_Out32(DMA_BASE + 0x00, 0x0001);
                    Xil_Out32(DMA_BASE + 0x18, (UINTPTR)&target_staging[target_offset]);
                    Xil_Out32(DMA_BASE + 0x28, current_bite * sizeof(RawPointPacket));

                    dma_tx_wait("Mode0");
//                    ip_wait_idle("Mode0");

                    // --------------------------------------------------------
                    //  MODE 1: Send ALL queries in the batch as anchors
                    // --------------------------------------------------------
                    int anchors_remaining = batch_query_count;
                    int anchor_group_offset = 0;

                    int rx_buffer_bytes = (int)(MATCHES_ADDR - RESULTS_ADDR);
                    int max_safe_anchors = ((rx_buffer_bytes / ((current_bite + 1) * (int)sizeof(DistPacket))) * UNROLL_FACTOR);
                    if (max_safe_anchors > MAX_ANCHORS_PER_TX) max_safe_anchors = MAX_ANCHORS_PER_TX;
                    if (max_safe_anchors < UNROLL_FACTOR)      max_safe_anchors = UNROLL_FACTOR;

                    while (anchors_remaining > 0) {
                        int current_a_bite = (anchors_remaining > max_safe_anchors) ? max_safe_anchors : anchors_remaining;
                        int padded_a_count = ((current_a_bite + UNROLL_FACTOR - 1) / UNROLL_FACTOR) * UNROLL_FACTOR;

                        // Copy this batch's queries into staging
                        for (int a_idx = 0; a_idx < padded_a_count; a_idx++) {
                            int local_a = anchor_group_offset + a_idx;
                            // Clamp for padding
                            if (local_a >= batch_query_count) local_a = batch_query_count - 1;
                            int qi_real = query_bins[qi_group_start + local_a].query_idx;
                            anchor_staging[a_idx] = query_coords[qi_real];
                        }

                        int tx_bytes = padded_a_count * sizeof(RawPointPacket);
                        Xil_DCacheFlushRange((UINTPTR)anchor_staging, tx_bytes);

                        int num_batches       = padded_a_count / UNROLL_FACTOR;
                        int max_result_pkts   = num_batches * current_bite + 1;
                        int expected_rx_bytes = max_result_pkts * (int)sizeof(DistPacket);

                        total_calcs += (unsigned long long)current_a_bite * current_bite;

                        Xil_Out32(IP_NUM_POINTS,  current_bite);
                        Xil_Out32(IP_NUM_DIMS,    num_dims);
                        Xil_Out32(IP_MAX_RANGE,   max_range_bits);
                        Xil_Out32(IP_NUM_ANCHORS, padded_a_count);
                        Xil_Out32(IP_MODE,        0x01);
                        Xil_Out32(IP_CTRL,        0x01);

                        Xil_Out32(DMA_BASE + 0x30, 0x0001);
                        Xil_Out32(DMA_BASE + 0x48, (UINTPTR)all_results);
                        Xil_Out32(DMA_BASE + 0x58, expected_rx_bytes);

                        Xil_Out32(DMA_BASE + 0x00, 0x0001);
                        Xil_Out32(DMA_BASE + 0x18, (UINTPTR)anchor_staging);
                        Xil_Out32(DMA_BASE + 0x28, tx_bytes);

                        dma_tx_wait("Mode1");
                        dma_rx_wait("Mode1");
//                        ip_wait_idle("Mode1");

                        int actual_rx_bytes = Xil_In32(DMA_BASE + 0x58);
                        Xil_DCacheInvalidateRange((UINTPTR)all_results, actual_rx_bytes);
                        int actual_rx_pkts  = actual_rx_bytes / (int)sizeof(DistPacket);

                        // --------------------------------------------------------
                        //  PARSE RESULTS
                        // --------------------------------------------------------
                        for (int p = 0; p < actual_rx_pkts; p++) {
                            DistPacket res = all_results[p];
                            if (res.target_id < 0) break;

                            int b     = res.anchor_b_idx;
                            int a_idx = b * UNROLL_FACTOR;

                            for (int a = 0; a < UNROLL_FACTOR; a++) {
                                if (a_idx + a >= current_a_bite) continue;
                                float ds = res.distances[a];
                                if (ds < 0.0f || ds > max_range_sq) continue;

                                if (match_count < MAX_MATCHES) match_count++;
                            }
                        }

                        anchors_remaining   -= current_a_bite;
                        anchor_group_offset += current_a_bite;
                    }

                    targets_remaining -= current_bite;
                    target_offset     += current_bite;
                }

                // Advance by the exact number of queries we batched
                qi_group_start += batch_query_count;
            }

            u64 time_end = get_timer_value(&TimerInstance);
            // ==========================================================
                  //  RESULTS
                  // ==========================================================
                  xil_printf(" M: %d\n\r", match_count);

                  unsigned int calcs_hi = (unsigned int)(total_calcs >> 32);
                  unsigned int calcs_lo = (unsigned int)(total_calcs & 0xFFFFFFFFUL);
                  if (calcs_hi > 0) xil_printf(" C   %u%09u\n\r", calcs_hi, calcs_lo);
                  else              xil_printf(" C   %u\n\r", calcs_lo);

                  u64 total_ticks = time_end - time_start_total;
                  u64 hw_ticks    = time_end - time_start_hw;
//                  u64 pre_ticks   = time_start_hw - time_start_total;

                  u64 total_sec = total_ticks / TIMER_FREQ_HZ;
                  u64 hw_sec    = hw_ticks    / TIMER_FREQ_HZ;
//                  u64 pre_sec   = pre_ticks   / TIMER_FREQ_HZ;

                  u32 total_ms = (u32)((total_ticks % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));
                  u32 hw_ms    = (u32)((hw_ticks    % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));
//                  u32 pre_ms   = (u32)((pre_ticks   % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));

                  u32 total_min = (u32)(total_sec/60); u32 total_s = (u32)(total_sec%60);
                  u32 hw_min    = (u32)(hw_sec/60);    u32 hw_s    = (u32)(hw_sec%60);
//                  u32 pre_min   = (u32)(pre_sec/60);   u32 pre_s   = (u32)(pre_sec%60);

//                  xil_printf("Pre :   %lu min %lu.%03lu sec\n\r", pre_min, pre_s,   pre_ms);
                  xil_printf("HW:   %lu min %lu.%03lu sec\n\r", hw_min,  hw_s,    hw_ms);
                  xil_printf("Total  :   %lu min %lu.%03lu sec\n\r", total_min, total_s, total_ms);

                  while(1);
                  return 0;
              }