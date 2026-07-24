
#include "xil_printf.h"
#include "xil_io.h"
#include "xparameters.h"
#include "xil_cache.h"
#include "xtmrctr.h"   // Timer API enabled
#include <math.h>
#include <stdlib.h>

// =============================================================
//  MEMORY MAP (Safely Spaced Out to prevent DMA Collisions)
// =============================================================
#define DDR_BASE_ADDR         0x80000000 // Raw Data (Max ~32 MB)
#define MAIN_POINTS_ADDR      0x82000000 // Main Array (Max ~40 MB)
#define UNIQUE_BINS_ADDR      0x85000000 // Bin Array (Max ~12 MB)
#define TARGET_STAGING_ADDR   0x86000000 // Staging
#define ANCHOR_STAGING_ADDR   0x86200000 // Staging
#define STAGING_IDS_ADDR      0x86400000 // Staging
#define RESULTS_ADDR          0x87000000 // DMA Results
#define MATCHES_ADDR          0x88000000 // Matches Output

#define DMA_BASE              0x41E00000
#define IP_BASE               0x00008000

#define IP_CTRL              (IP_BASE + 0x00)
#define IP_NUM_POINTS        (IP_BASE + 0x10)
#define IP_NUM_DIMS          (IP_BASE + 0x18)
#define IP_MAX_RANGE         (IP_BASE + 0x20)
#define IP_MODE              (IP_BASE + 0x28)
#define IP_NUM_ANCHORS       (IP_BASE + 0x30)

#define TIMER_FREQ_HZ  200000000ULL

// =============================================================
//  CONSTANTS
// =============================================================
#define MAX_RANGE             170.491f
#define MAX_BRAM_CAPACITY     5000
#define MAX_MATCHES           2000000000
#define WATCHDOG_LIMIT        20000000
#define UNROLL_FACTOR         4

// Sending exactly 1024 anchors per batch (Hardware Optimized)
#define MAX_ANCHORS_PER_TX    1024

// =============================================================
//  DATA TYPES
// =============================================================
typedef struct { float d[8]; } RawPointPacket;
typedef struct { float d[8]; int bin_id; int original_id; } BinPointPacket;
typedef struct { int bin_id; int offset; int count; } BinInfo;

typedef struct {
    float distances[UNROLL_FACTOR];
    int   target_id;
    int   anchor_b_idx;
    int   pad1, pad2;
} DistPacket;  // 32 bytes

typedef struct { int anchor_id; int target_id; float distance; } MatchRecord;

#define BIN3D(bx, by, bz, nby, nbz) ((bx)*((nby)*(nbz)) + (by)*(nbz) + (bz))

// =============================================================
//  HELPERS
// =============================================================

// Safe 64-bit cascade timer read function to prevent rollover glitches
u64 get_timer_value(XTmrCtr *TimerInstancePtr) {
	u32 upper1 = XTmrCtr_GetValue(TimerInstancePtr, 1);
	u32 lower  = XTmrCtr_GetValue(TimerInstancePtr, 0);
	u32 upper2 = XTmrCtr_GetValue(TimerInstancePtr, 1);

	// If the upper 32-bits ticked over while we were reading the lower 32-bits, read lower again
	if (upper1 != upper2) {
		lower = XTmrCtr_GetValue(TimerInstancePtr, 0);
	}
	return (((u64)upper2) << 32) | (u64)lower;
}
int compare_bins(const void *a, const void *b) {
    BinPointPacket *p1 = (BinPointPacket *)a;
    BinPointPacket *p2 = (BinPointPacket *)b;
    if (p1->bin_id < p2->bin_id) return -1;
    if (p1->bin_id > p2->bin_id) return  1;
    return 0;
}

int binary_search_bins(BinInfo *arr, int count, int target_id) {
    int low = 0, high = count - 1;
    while (low <= high) {
        int mid = low + (high - low) / 2;
        if      (arr[mid].bin_id == target_id) return mid;
        else if (arr[mid].bin_id <  target_id) low  = mid + 1;
        else                                   high = mid - 1;
    }
    return -1;
}

// -------------------------------------------------------------
// STRICT HARDWARE SYNCHRONIZATION HELPERS
// -------------------------------------------------------------
static inline void dma_tx_wait(const char *label) {
    int wd = 0;
    while (!(Xil_In32(DMA_BASE + 0x04) & 0x0002)) {
        if (wd++ > WATCHDOG_LIMIT) { xil_printf("[FATAL] DMA TX Jam: %s\n\r", label); while(1); }
    }
}

static inline void dma_rx_wait(const char *label) {
    int wd = 0;
    while (!(Xil_In32(DMA_BASE + 0x34) & 0x0002)) {
        if (wd++ > WATCHDOG_LIMIT) { xil_printf("[FATAL] DMA RX Halt: %s\n\r", label); while(1); }
    }
}

static inline void ip_wait_idle(const char *label) {
    int wd = 0;
    while (!(Xil_In32(IP_CTRL) & 0x04)) {
        if (wd++ > WATCHDOG_LIMIT) { xil_printf("[FATAL] IP stuck: %s\n\r", label); while(1); }
    }
}

// =============================================================
//  MAIN
// =============================================================
int main() {

//    Xil_DCacheDisable();

	// ---------------------------------------------------------
	//  TIMER INITIALIZATION (64-BIT CASCADE MODE)
	// ---------------------------------------------------------
	XTmrCtr TimerInstance;
	int status = XTmrCtr_Initialize(&TimerInstance, XPAR_TMRCTR_0_DEVICE_ID);
	if (status != XST_SUCCESS) {
		xil_printf("Ti int !\n\r");
	}

	// Set the timer to cascade mode (chains Timer 0 and Timer 1 to act as a 64-bit timer)
	XTmrCtr_SetOptions(&TimerInstance, 0, XTC_CASCADE_MODE_OPTION);

	// Start the chained timer
	XTmrCtr_Start(&TimerInstance, 0);

	// Capture absolute start time of the entire application
	u64 time_start_total = get_timer_value(&TimerInstance);
	// ---------------------------------------------------------

    int *file_header  = (int *)DDR_BASE_ADDR;
    int  num_dims     = file_header[0];
    int  total_points = file_header[1];

    xil_printf("\n\rDims=%d  Points=%d\n\r", num_dims, total_points);

    RawPointPacket *all_coords        = (RawPointPacket *)(DDR_BASE_ADDR + 8);
    BinPointPacket *main_points_array = (BinPointPacket *)MAIN_POINTS_ADDR;
    int             unique_bin_count  = 0;
    unsigned long long total_calcs    = 0;

    float min_x = 999999.0f, max_x = -999999.0f;
    float min_y = 999999.0f, max_y = -999999.0f;
    float min_z = 999999.0f, max_z = -999999.0f;

    for (int i = 0; i < total_points; i++) {
        for (int dim = 0; dim < 8; dim++)
            main_points_array[i].d[dim] = all_coords[i].d[dim];
        main_points_array[i].bin_id      = 0;
        main_points_array[i].original_id = i;

        float x = main_points_array[i].d[0];
        if (x < min_x) min_x = x; if (x > max_x) max_x = x;
        if (num_dims > 1) {
            float y = main_points_array[i].d[1];
            if (y < min_y) min_y = y; if (y > max_y) max_y = y;
        }
        if (num_dims > 2) {
            float z = main_points_array[i].d[2];
            if (z < min_z) min_z = z; if (z > max_z) max_z = z;
        }
    }

    float spread_x = (max_x - min_x) + 0.0001f;
    float spread_y = (num_dims > 1) ? (max_y - min_y) + 0.0001f : 1.0f;
    float spread_z = (num_dims > 2) ? (max_z - min_z) + 0.0001f : 1.0f;

    float max_range      = MAX_RANGE;
    u32   max_range_bits = *(u32 *)&max_range;
    float max_range_sq   = max_range * max_range;

    float voxel_size = MAX_RANGE;
    float step_size  = MAX_RANGE * 0.1f;
    long long total_voxels = 0;
    int nbx = 1, nby = 1, nbz = 1;

    while (1) {
        nbx = (int)(spread_x / voxel_size); if (nbx < 1) nbx = 1;
        nby = (num_dims > 1) ? (int)(spread_y / voxel_size) : 1; if (nby < 1) nby = 1;
        nbz = (num_dims > 2) ? (int)(spread_z / voxel_size) : 1; if (nbz < 1) nbz = 1;
        total_voxels = (long long)nbx * nby * nbz;
        if (total_voxels < 2000000000LL) break;
        voxel_size += step_size; // Restored to match ~17 million calculations
    }

    float inv_bx = (float)nbx / spread_x;
    float inv_by = (float)nby / spread_y;
    float inv_bz = (float)nbz / spread_z;
    xil_printf("[P2] Grid: %d x %d x %d\n\r", nbx, nby, nbz);

    xil_printf("[P3] Assigning bins...\n\r");
    for (int i = 0; i < total_points; i++) {
        int bx = (int)((main_points_array[i].d[0] - min_x) * inv_bx);
        if (bx >= nbx) bx = nbx-1; if (bx < 0) bx = 0;
        int by = (num_dims > 1) ? (int)((main_points_array[i].d[1] - min_y) * inv_by) : 0;
        if (by >= nby) by = nby-1; if (by < 0) by = 0;
        int bz = (num_dims > 2) ? (int)	((main_points_array[i].d[2] - min_z) * inv_bz) : 0;
        if (bz >= nbz) bz = nbz-1; if (bz < 0) bz = 0;
        main_points_array[i].bin_id = BIN3D(bx, by, bz, nby, nbz);
    }

    xil_printf("[P4] Sorting...\n\r");
    qsort(main_points_array, total_points, sizeof(BinPointPacket), compare_bins);

    BinInfo *unique_bins = (BinInfo *)UNIQUE_BINS_ADDR;
    int current_bin = -1;
    for (int i = 0; i < total_points; i++) {
        int b = main_points_array[i].bin_id;
        if (b != current_bin) {
            current_bin = b;
            unique_bins[unique_bin_count].bin_id = b;
            unique_bins[unique_bin_count].offset = i;
            unique_bins[unique_bin_count].count  = 1;
            unique_bin_count++;
        } else {
            unique_bins[unique_bin_count-1].count++;
        }
    }
    xil_printf("[P5] Found %d bins.\n\r", unique_bin_count);




    xil_printf("[P6] Hardware loop starting...\n\r");

	// =========================================================
	// CAPTURE HARDWARE START TIME
	// =========================================================
	u64 time_start_hw = get_timer_value(&TimerInstance);

    // 1. Clear any pending interrupts
    Xil_Out32(DMA_BASE + 0x04, 0x1000);
    Xil_Out32(DMA_BASE + 0x34, 0x1000);

    // 2. Enable interrupts/master settings
    Xil_Out32(DMA_BASE + 0x00, 0x0001);
    Xil_Out32(DMA_BASE + 0x30, 0x0001);

    RawPointPacket *target_staging = (RawPointPacket *)TARGET_STAGING_ADDR;
    RawPointPacket *anchor_staging = (RawPointPacket *)ANCHOR_STAGING_ADDR;
    int            *staging_ids    = (int *)STAGING_IDS_ADDR;
    MatchRecord    *saved_matches  = (MatchRecord *)MATCHES_ADDR;
    DistPacket     *all_results    = (DistPacket  *)RESULTS_ADDR;

    int match_count = 0;


    for (int i = 0; i < unique_bin_count; i++) {

        int b_anchor = unique_bins[i].bin_id;
        int a_off    = unique_bins[i].offset;
        int a_count  = unique_bins[i].count;

        int bz_coord = b_anchor % nbz;
        int by_coord = (b_anchor / nbz) % nby;
        int bx_coord = b_anchor / (nby * nbz);

        int chunk_pts        = 0;
        int center_vox_count = 0;

        for (int dx = 0; dx <= 1; dx++) {
            int dy_start = (dx == 0) ? 0 : -1;
            for (int dy = dy_start; dy <= 1; dy++) {
                int dz_start = (dx == 0 && dy == 0) ? 0 : -1;
                for (int dz = dz_start; dz <= 1; dz++) {
                    int nx  = bx_coord + dx;
                    int ny2 = by_coord + dy;
                    int nz  = bz_coord + dz;
                    if (nx<0||nx>=nbx||ny2<0||ny2>=nby||nz<0||nz>=nbz) continue;

                    int nb      = BIN3D(nx, ny2, nz, nby, nbz);
                    int nb_idx  = binary_search_bins(unique_bins, unique_bin_count, nb);
                    if (nb_idx == -1) continue;

                    int nb_offset = unique_bins[nb_idx].offset;
                    int nb_count  = unique_bins[nb_idx].count;

                    for (int p = 0; p < nb_count; p++) {
                        target_staging[chunk_pts] = *(RawPointPacket *)&main_points_array[nb_offset + p];
                        staging_ids[chunk_pts]    = main_points_array[nb_offset + p].original_id;
                        chunk_pts++;
                    }
                    if (dx==0 && dy==0 && dz==0) center_vox_count = chunk_pts;
                }
            }
        }

        if (chunk_pts == 0) continue;

        int targets_remaining = chunk_pts;
        int target_offset     = 0;

        while (targets_remaining > 0) {
            int current_bite = (targets_remaining > MAX_BRAM_CAPACITY)
                               ? MAX_BRAM_CAPACITY : targets_remaining;

            Xil_DCacheFlushRange((UINTPTR)&target_staging[target_offset], current_bite * sizeof(RawPointPacket));

            Xil_Out32(IP_NUM_POINTS,  current_bite);
            Xil_Out32(IP_NUM_DIMS,    num_dims);
            Xil_Out32(IP_MAX_RANGE,   max_range_bits);
            Xil_Out32(IP_NUM_ANCHORS, 0);
            Xil_Out32(IP_MODE,        0x00);
            Xil_Out32(IP_CTRL,        0x01); // ap_start

            Xil_Out32(DMA_BASE + 0x00, 0x0001);
            Xil_Out32(DMA_BASE + 0x18, (UINTPTR)&target_staging[target_offset]);
            Xil_Out32(DMA_BASE + 0x28, current_bite * sizeof(RawPointPacket));

            dma_tx_wait("Mode0 cache load");
            ip_wait_idle("Mode0");

            int anchors_remaining = a_count;
            int anchor_offset     = 0;

            while (anchors_remaining > 0) {

                int current_a_bite = (anchors_remaining > MAX_ANCHORS_PER_TX)
                                     ? MAX_ANCHORS_PER_TX : anchors_remaining;
                int padded_a_count = ((current_a_bite + UNROLL_FACTOR - 1)
                                      / UNROLL_FACTOR) * UNROLL_FACTOR;

                for (int a_idx = 0; a_idx < padded_a_count; a_idx++) {
                    int src = (a_idx < current_a_bite)
                              ? (a_off + anchor_offset + a_idx)
                              : (a_off + anchor_offset);
                    anchor_staging[a_idx] = *(RawPointPacket *)&main_points_array[src];
                }

                Xil_DCacheFlushRange((UINTPTR)anchor_staging, padded_a_count * sizeof(RawPointPacket));

                int num_batches       = padded_a_count / UNROLL_FACTOR;
                int max_result_pkts   = num_batches * current_bite + 1;
                int expected_rx_bytes = max_result_pkts * (int)sizeof(DistPacket);

                total_calcs += (unsigned long long)padded_a_count * current_bite;

                Xil_Out32(IP_NUM_POINTS,  current_bite);
                Xil_Out32(IP_NUM_DIMS,    num_dims);
                Xil_Out32(IP_MAX_RANGE,   max_range_bits);
                Xil_Out32(IP_NUM_ANCHORS, padded_a_count);
                Xil_Out32(IP_MODE,        0x01);
                Xil_Out32(IP_CTRL,        0x01); // ap_start

                Xil_Out32(DMA_BASE + 0x30, 0x0001);
                Xil_Out32(DMA_BASE + 0x48, (UINTPTR)all_results);
                Xil_Out32(DMA_BASE + 0x58, expected_rx_bytes);

                Xil_Out32(DMA_BASE + 0x00, 0x0001);
                Xil_Out32(DMA_BASE + 0x18, (UINTPTR)anchor_staging);
                Xil_Out32(DMA_BASE + 0x28, padded_a_count * sizeof(RawPointPacket));

                dma_tx_wait("Mode1 anchor TX");
                dma_rx_wait("Mode1 result RX");
//                ip_wait_idle("Mode1");

//                Xil_DCacheInvalidateRange((UINTPTR)all_results, expected_rx_bytes);
//
//                int actual_rx_pkts = Xil_In32(DMA_BASE + 0x58) / (int)sizeof(DistPacket);

                int actual_rx_bytes = Xil_In32(DMA_BASE + 0x58);
                                Xil_DCacheInvalidateRange((UINTPTR)all_results, actual_rx_bytes);

                                int actual_rx_pkts = actual_rx_bytes / (int)sizeof(DistPacket);

                for (int p = 0; p < actual_rx_pkts; p++) {

                    DistPacket res = all_results[p];
                    if (res.target_id < 0) break;  // terminator

                    int b     = res.anchor_b_idx;
                    int a_idx = b * UNROLL_FACTOR;

                    int local_t_id       = res.target_id;
                    int global_stage_idx = target_offset + local_t_id;
                    int t_id             = staging_ids[global_stage_idx];

                    for (int a = 0; a < UNROLL_FACTOR; a++) {
                        if (a_idx + a >= current_a_bite) continue;

                        float ds = res.distances[a];

                        if (ds < 0.0f || ds > max_range_sq) continue;

                        int a_id = main_points_array[a_off + anchor_offset + a_idx + a].original_id;

                        if (global_stage_idx < center_vox_count) {
                            if (a_id >= t_id) continue;
                        } else {
                            if (a_id == t_id) continue;
                        }

                        if (match_count < MAX_MATCHES) {
//                            saved_matches[match_count].anchor_id = a_id;
//                            saved_matches[match_count].target_id = t_id;
//                            saved_matches[match_count].distance  = sqrtf(ds);
                            match_count++;
                        }
                    }
                }

                anchors_remaining -= current_a_bite;
                anchor_offset     += current_a_bite;

            } // end anchor chunk loop

            targets_remaining -= current_bite;
            target_offset     += current_bite;

        } // end target bite loop
    }
    // =========================================================
    	// CAPTURE END TIME
    	// =========================================================
    	u64 time_end = get_timer_value(&TimerInstance);

    	// =========================================================
    	//  RESULTS
    	// =========================================================
    	xil_printf("Total Matches: %d\n\r", match_count);

    	unsigned int calcs_hi = (unsigned int)(total_calcs >> 32);
    	unsigned int calcs_lo = (unsigned int)(total_calcs & 0xFFFFFFFFUL);
    	if (calcs_hi > 0)
    	    xil_printf("Total Calcs:   %u%09u\n\r", calcs_hi, calcs_lo);
    	else
    	    xil_printf("Total Calcs:   %u\n\r", calcs_lo);

    	// ---------------------------------------------------------
    		//  TIMING — Pure Integer Math with Milliseconds
    		// ---------------------------------------------------------
    		u64 total_ticks = time_end       - time_start_total;
    		u64 hw_ticks    = time_end       - time_start_hw;
    		u64 pre_ticks   = time_start_hw  - time_start_total;

    		// 1. Calculate Full Seconds
    		u32 total_sec = (u32)(total_ticks / TIMER_FREQ_HZ);
    		u32 hw_sec    = (u32)(hw_ticks    / TIMER_FREQ_HZ);
    		u32 pre_sec   = (u32)(pre_ticks   / TIMER_FREQ_HZ);

    		// 2. Calculate Remaining Milliseconds (The "Missing" fractions)
    	    //    % gets the leftover ticks, dividing by (FREQ/1000) converts to ms
    		u32 total_ms = (u32)((total_ticks % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));
    		u32 hw_ms    = (u32)((hw_ticks    % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));
    		u32 pre_ms   = (u32)((pre_ticks   % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));

    		// 3. Split seconds into minutes and remaining seconds
    		u32 total_min   = total_sec / 60;
    		u32 total_s_rem = total_sec % 60;

    	    u32 hw_min      = hw_sec / 60;
    		u32 hw_s_rem    = hw_sec % 60;

    	    u32 pre_min     = pre_sec / 60;
    		u32 pre_s_rem   = pre_sec % 60;

    	    // 4. Print with exactly 3 decimal places for milliseconds (%03lu)
    		xil_printf("Pre-HW Time:   %lu min %lu.%03lu sec\n\r", pre_min,   pre_s_rem,   pre_ms);
    		xil_printf("HW Loop Time:  %lu min %lu.%03lu sec\n\r", hw_min,    hw_s_rem,    hw_ms);
    		xil_printf("Total Time:    %lu min %lu.%03lu sec\n\r", total_min, total_s_rem, total_ms);

    while(1);
    return 0;
}
