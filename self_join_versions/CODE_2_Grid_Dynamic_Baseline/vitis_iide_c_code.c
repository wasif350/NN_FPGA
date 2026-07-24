
// version with dynamic top three dimensions and sort on 4th dimension
#include "xil_printf.h"
#include "xil_io.h"
#include "xparameters.h"
#include "xil_cache.h"
#include "xtmrctr.h" // UNCOMMENTED: Xilinx Timer Library
#include <math.h>
#include <stdlib.h>

// =============================================================
//  MEMORY MAP (STRICTLY PACKED FOR 128 MB NEXYS A7 DDR2)
//  Ends exactly at 0x87FFFFFF
// =============================================================
//#define DDR_BASE_ADDR         0x80000000 // Raw Data (22 MB)
//#define MAIN_POINTS_ADDR      0x81600000 // Main Array (28 MB)
//#define UNIQUE_BINS_ADDR      0x83200000 // Bin Array (12 MB)
//#define MATCHES_ADDR          0x83E00000 // Matches Output (24 MB) - KEPT INTACT
//
//#define TARGET_STAGING_ADDR   0x85600000 // Staging (Holds 5000 = 160KB)
//
//// MASSIVE ANCHOR STAGING: Expanded to ~1 Megabyte
//#define ANCHOR_STAGING_ADDR   0x85A00000
//#define COORD_INFO_ADDR          0x85B00000
//
//// Shifted forward by 1MB to make room for the 32,768 anchor limit!
//#define STAGING_IDS_ADDR      0x85C00000 // Staging IDs
//#define RESULTS_ADDR          0x85D00000 // DMA Results (~40 MB headroom to end of RAM)
//
//#define DMA_BASE              0x41E00000
//#define IP_BASE               0x00008000

#define DDR_BASE_ADDR         0x80000000
#define MAIN_POINTS_ADDR      0x82000000
#define UNIQUE_BINS_ADDR      0x84800000
#define TARGET_STAGING_ADDR   0x85400000
#define STAGING_IDS_ADDR      0x85A00000
#define ANCHOR_STAGING_ADDR   0x85B00000
#define COORD_INFO_ADDR       0x85C00000
#define RESULTS_ADDR          0x85D00000
//#define MATCHES_ADDR          0x87200000

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
#define MAX_RANGE             7.451f

#define MAX_BRAM_CAPACITY     5000
#define MAX_MATCHES           2000000000
#define WATCHDOG_LIMIT        20000000
#define UNROLL_FACTOR         4
#define SYSTEM_CLOCK_FREQ_HZ  100000000.0 // 100 MHz clock for Nexys A7 / Artix-7
#define TIMER_FREQ_HZ  200000000ULL

// =============================================================
//  DATA TYPES
// =============================================================
typedef struct {
	float d[8];
} RawPointPacket;
typedef struct {
	float d[8];
	int bin_id;
	int original_id;
} BinPointPacket;
typedef struct {
	int bin_id;
	int offset;
	int count;
} BinInfo;

typedef struct {
	float distances[UNROLL_FACTOR];
	int target_id;
	int anchor_b_idx;
	int pad1, pad2;
} DistPacket;  // 32 bytes

typedef struct {
	int cord_id;
	float cord_min;
	float cord_max;
	float cord_spread;
} CordInfo;

//typedef struct {
//	int anchor_id;
//	int target_id;
//	float distance;
//} MatchRecord;

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
	BinPointPacket *p1 = (BinPointPacket *) a;
	BinPointPacket *p2 = (BinPointPacket *) b;
	if (p1->bin_id < p2->bin_id)
		return -1;
	if (p1->bin_id > p2->bin_id)
		return 1;

	if (p1->d[4] < p2->d[4]) return -1;
	if (p1->d[4] > p2->d[4]) return 1;
	return 0;
}

int binary_search_bins(BinInfo *arr, int count, int target_id) {
	int low = 0, high = count - 1;
	while (low <= high) {
		int mid = low + (high - low) / 2;
		if (arr[mid].bin_id == target_id)
			return mid;
		else if (arr[mid].bin_id < target_id)
			low = mid + 1;
		else
			high = mid - 1;
	}
	return -1;
}

int binary_search_bins_points(BinPointPacket *arr, int nb_offset, int nb_count, float lowest_valid, float highest_valid, int is_start) {
    int low = 0, high = nb_count - 1;
    int ans = is_start ? nb_count : -1;

    while (low <= high) {
        int mid = low + (high - low) / 2;

        if (is_start) {
            // Logic to find the START index ('r' in the paper)
            if (arr[nb_offset + mid].d[4] >= lowest_valid) {
                ans = mid;
                high = mid - 1; // Keep looking left
            } else {
                low = mid + 1;
            }
        } else {
            // Logic to find the END index ('s' in the paper)
            if (arr[nb_offset + mid].d[4] <= highest_valid) {
                ans = mid;
                low = mid + 1; // Keep looking right
            } else {
                high = mid - 1;
            }
        }
    }
    return ans;
}

// -------------------------------------------------------------
// STRICT HARDWARE SYNCHRONIZATION HELPERS
// -------------------------------------------------------------
static inline void dma_tx_wait(const char *label) {
	int wd = 0;
	while (!(Xil_In32(DMA_BASE + 0x04) & 0x0002)) {
		if (wd++ > WATCHDOG_LIMIT) {
//			xil_printf("[FATAL] DMA TX Jam: %s\n\r", label);
			while (1)
				;
		}
	}
}

static inline void dma_rx_wait(const char *label) {
	int wd = 0;
	while (!(Xil_In32(DMA_BASE + 0x34) & 0x0002)) {
		if (wd++ > WATCHDOG_LIMIT) {
//			xil_printf("[FATAL] DMA RX Halt: %s\n\r", label);
			while (1)
				;
		}
	}
}

static inline void ip_wait_idle(const char *label) {
	int wd = 0;
	while (!(Xil_In32(IP_CTRL) & 0x04)) {
		if (wd++ > WATCHDOG_LIMIT) {
//			xil_printf("[FATAL] IP stuck: %s\n\r", label);
			while (1)
				;
		}
	}
}

// =============================================================
//  MAIN
// =============================================================
int main() {

//	Xil_DCacheDisable();

	// ---------------------------------------------------------
	//  TIMER INITIALIZATION (64-BIT CASCADE MODE)
	// ---------------------------------------------------------
	XTmrCtr TimerInstance;
	XTmrCtr_Initialize(&TimerInstance, XPAR_TMRCTR_0_DEVICE_ID);
//	if (status != XST_SUCCESS) {
//		xil_printf("Ti int !\n\r");
//	}

	// Set the timer to cascade mode (chains Timer 0 and Timer 1 to act as a 64-bit timer)
	XTmrCtr_SetOptions(&TimerInstance, 0, XTC_CASCADE_MODE_OPTION);

	// Start the chained timer
	XTmrCtr_Start(&TimerInstance, 0);

	// Capture absolute start time of the entire application
	u64 time_start_total = get_timer_value(&TimerInstance);
	// ---------------------------------------------------------

	int *file_header = (int *) DDR_BASE_ADDR;
	int num_dims = file_header[0];
	int total_points = file_header[1];

	xil_printf("Dims=%d  Points=%d\n\r", num_dims, total_points);
	xil_printf("RANGE=%d \n\r", MAX_RANGE);

	RawPointPacket *all_coords = (RawPointPacket *) (DDR_BASE_ADDR + 8);
	BinPointPacket *main_points_array = (BinPointPacket *) MAIN_POINTS_ADDR;
	int unique_bin_count = 0;
	unsigned long long total_calcs = 0;

	CordInfo *coord_vals = (CordInfo *) COORD_INFO_ADDR;

	// 1. Initialize Min/Max Trackers
    for (int dim = 0; dim < 8; dim++) {
        coord_vals[dim].cord_id = dim;
        coord_vals[dim].cord_min = 999999.0f;
        coord_vals[dim].cord_max = -999999.0f;
    }

    // 2. Single-Pass Memory Copy and Min/Max Evaluation
    for (int i = 0; i < total_points; i++) {

        // Write these ONLY ONCE per point
        main_points_array[i].bin_id = 0;
        main_points_array[i].original_id = i;

        for (int dim = 0; dim < 8; dim++) {
            float d = all_coords[i].d[dim];
            main_points_array[i].d[dim] = d;

            if (d < coord_vals[dim].cord_min) coord_vals[dim].cord_min = d;
            if (d > coord_vals[dim].cord_max) coord_vals[dim].cord_max = d;
        }
    }

    // 3. Calculate Spread
    for (int dim = 0; dim < 8; dim++) {
        coord_vals[dim].cord_spread = (coord_vals[dim].cord_max - coord_vals[dim].cord_min) + 0.0001f;
    }

	int top1_id = -1, top2_id = -1, top3_id = -1;
	float top1_spread = -1.0f, top2_spread = -1.0f, top3_spread = -1.0f;
	for (int j = 0; j < num_dims; j++) {
		float current_spread = coord_vals[j].cord_spread;
		int current_id = coord_vals[j].cord_id;
		if (current_spread > top1_spread) {
			top3_spread = top2_spread;
			top3_id = top2_id;

			top2_spread = top1_spread;
			top2_id = top1_id;

			top1_spread = current_spread;
			top1_id = current_id;
		} else if (current_spread > top2_spread) {
			top3_spread = top2_spread;
			top3_id = top2_id;

			top2_spread = current_spread;
			top2_id = current_id;
		}
		else if (current_spread > top3_spread){
			top3_spread = current_spread;
			top3_id = current_id;
		}
	}

	float spread_x = top1_spread;
	float spread_y = top2_spread;
	float spread_z = top3_spread;

	float max_range = MAX_RANGE;
//	u32 max_range_bits = *(u32 *) &max_range;
	union { float f; u32 u; } f2u;
	f2u.f = max_range;
	u32 max_range_bits = f2u.u;
	float max_range_sq = max_range * max_range;

	float voxel_size = MAX_RANGE;
	float step_size = MAX_RANGE * 0.1f;
	long long total_voxels = 0;
	int nbx = 1, nby = 1, nbz = 1;

	while (1) {
		nbx = (int) (spread_x / voxel_size);
		if (nbx < 1)
			nbx = 1;
		nby = (num_dims > 1) ? (int) (spread_y / voxel_size) : 1;
		if (nby < 1)
			nby = 1;
		nbz = (num_dims > 2) ? (int) (spread_z / voxel_size) : 1;
		if (nbz < 1)
			nbz = 1;
		total_voxels = (long long) nbx * nby * nbz;
		if (total_voxels < 2000000000LL)
			break;
		voxel_size += step_size;
	}

	float inv_bx = (float) nbx / spread_x;
	float inv_by = (float) nby / spread_y;
	float inv_bz = (float) nbz / spread_z;
	xil_printf("[P2] Grid: %d x %d x %d\n\r", nbx, nby, nbz);

	xil_printf("[P3] Ass bin...\n\r");
	for (int i = 0; i < total_points; i++) {
		int bx = (int) ((main_points_array[i].d[top1_id] - coord_vals[top1_id].cord_min) * inv_bx);
		if (bx >= nbx)
			bx = nbx - 1;
		if (bx < 0)
			bx = 0;
		int by =
				(num_dims > 1) ?
						(int) ((main_points_array[i].d[top2_id] - coord_vals[top2_id].cord_min) * inv_by) :
						0;
		if (by >= nby)
			by = nby - 1;
		if (by < 0)
			by = 0;
		int bz =
				(num_dims > 2) ?
						(int) ((main_points_array[i].d[top3_id] - coord_vals[top3_id].cord_min) * inv_bz) :
						0;
		if (bz >= nbz)
			bz = nbz - 1;
		if (bz < 0)
			bz = 0;
		main_points_array[i].bin_id = BIN3D(bx, by, bz, nby, nbz);
	}

	xil_printf("[P4] Sort...\n\r");
	qsort(main_points_array, total_points, sizeof(BinPointPacket), compare_bins);

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
	xil_printf("[P5] Found %d bins.\n\r", unique_bin_count);

	// =====================================================
	// DEBUG BLOCK 1: After P5, before P6
	// Shows grid and bin structure
	// =====================================================
//	xil_printf("[DBG] total_points=%d unique_bins=%d\n\r",
//	           total_points, unique_bin_count);
//	xil_printf("voxel_size=%d nbx=%d nby=%d nbz=%d\n\r",
//	           (int)voxel_size, nbx, nby, nbz);

	// Print first 5 bins to see density
//	for (int b = 0; b < 5 && b < unique_bin_count; b++) {
//	    xil_printf("bin[%d] id=%d offset=%d count=%d\n\r",
//	               b, unique_bins[b].bin_id,
//	               unique_bins[b].offset,
//	               unique_bins[b].count);
//	}

//	xil_printf("[P6] Hardware loop starting...\n\r");

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

	RawPointPacket *target_staging = (RawPointPacket *) TARGET_STAGING_ADDR;
	RawPointPacket *anchor_staging = (RawPointPacket *) ANCHOR_STAGING_ADDR;
	int *staging_ids = (int *) STAGING_IDS_ADDR;
//	MatchRecord *saved_matches = (MatchRecord *) MATCHES_ADDR;
	DistPacket *all_results = (DistPacket *) RESULTS_ADDR;

	int match_count = 0;

	// =====================================================
	// DEBUG BLOCK 2: Inside the main bin loop
	// Track DMA calls and anchor/target sizes
	// Add counters BEFORE the bin loop:
	// =====================================================
//	int total_dma_mode0 = 0;
//	int total_dma_mode1 = 0;
//	int total_anchors_sent = 0;
//	int total_targets_sent = 0;
//	int bins_skipped_empty = 0;
//	int bins_processed = 0;
//
//
//	u64 total_neighbour_ticks = 0;
//
//    // --- SUB-TIMER ACCUMULATORS ---
//    u64 total_axi_setup_ticks = 0;
//    u64 total_dma_wait_ticks  = 0;
//    u64 total_parse_ticks     = 0;
//    u64 t_start, t_mid1, t_mid2, t_end;
//
//
////    u64 current_setup_ticks = 0;
//        u64 max_mode0_setup_ticks = 0;
//        u64 max_mode1_setup_ticks = 0;

    // ------------------------------


	for (int i = 0; i < unique_bin_count; i++) {
//		u64 nb_start = get_timer_value(&TimerInstance);
		int b_anchor = unique_bins[i].bin_id;
		int a_off = unique_bins[i].offset;
		int a_count = unique_bins[i].count;

		int bz_coord = b_anchor % nbz;
		int by_coord = (b_anchor / nbz) % nby;
		int bx_coord = b_anchor / (nby * nbz);

		int chunk_pts = 0;
		int center_vox_count = 0;

		for (int dx = 0; dx <= 1; dx++) {
			int dy_start = (dx == 0) ? 0 : -1;
			for (int dy = dy_start; dy <= 1; dy++) {
				int dz_start = (dx == 0 && dy == 0) ? 0 : -1;
				for (int dz = dz_start; dz <= 1; dz++) {
					int nx = bx_coord + dx;
					int ny2 = by_coord + dy;
					int nz = bz_coord + dz;
					if (nx < 0 || nx >= nbx || ny2 < 0 || ny2 >= nby || nz < 0
							|| nz >= nbz)
						continue;

					int nb = BIN3D(nx, ny2, nz, nby, nbz);
					int nb_idx = binary_search_bins(unique_bins,
							unique_bin_count, nb);
					if (nb_idx == -1)
						continue;

					int nb_offset = unique_bins[nb_idx].offset;
					int nb_count = unique_bins[nb_idx].count;

					// 1. Calculate the 4th-Dimension Safe Zone from the Anchor Bin
					float anchor_min_d4 = main_points_array[a_off].d[4];
					float anchor_max_d4 = main_points_array[a_off + a_count - 1].d[4];

					float lowest_valid  = anchor_min_d4 - MAX_RANGE;
					float highest_valid = anchor_max_d4 + MAX_RANGE;

					int start_p = binary_search_bins_points(main_points_array, nb_offset, nb_count, lowest_valid, highest_valid, 1);
					int end_p   = binary_search_bins_points(main_points_array, nb_offset, nb_count, lowest_valid, highest_valid, 0);

					// 4. Now, ONLY copy the valid targets to the FPGA!
					if (end_p >= start_p && start_p < nb_count && end_p >= 0) {
					    for (int p = start_p; p <= end_p; p++) {
					        target_staging[chunk_pts] = *(RawPointPacket *) &main_points_array[nb_offset + p];
					        staging_ids[chunk_pts]    = main_points_array[nb_offset + p].original_id;
					        chunk_pts++;
					    }
					}
					if (dx == 0 && dy == 0 && dz == 0)
						center_vox_count = chunk_pts;
				}
			}
		}
//		  u64 nb_end = get_timer_value(&TimerInstance);
//		    total_neighbour_ticks += (nb_end - nb_start);
//
		if (chunk_pts == 0)
			continue;
		// Inside the bin loop, after chunk_pts collection:
//		if (chunk_pts == 0) { bins_skipped_empty++; continue; }
//		bins_processed++;

		int targets_remaining = chunk_pts;
		int target_offset = 0;

		while (targets_remaining > 0) {
			int current_bite =
					(targets_remaining > MAX_BRAM_CAPACITY) ?
							MAX_BRAM_CAPACITY : targets_remaining;

			Xil_DCacheFlushRange((UINTPTR )&target_staging[target_offset],
					current_bite * sizeof(RawPointPacket));
//			t_start = get_timer_value(&TimerInstance);

			Xil_Out32(IP_NUM_POINTS, current_bite);
			Xil_Out32(IP_NUM_DIMS, num_dims);
			Xil_Out32(IP_MAX_RANGE, max_range_bits);
			Xil_Out32(IP_NUM_ANCHORS, 0);
			Xil_Out32(IP_MODE, 0x00);
			Xil_Out32(IP_CTRL, 0x01); // ap_start

			Xil_Out32(DMA_BASE + 0x00, 0x0001);
			Xil_Out32(DMA_BASE + 0x18,
					(UINTPTR) &target_staging[target_offset]);
			Xil_Out32(DMA_BASE + 0x28, current_bite * sizeof(RawPointPacket));

			// --- END MODE 0 SETUP TIMER / START MODE 0 WAIT TIMER ---
//			            t_mid1 = get_timer_value(&TimerInstance);
//			            total_axi_setup_ticks += (t_mid1 - t_start);

			            dma_tx_wait("Mode0 cache load");
			            ip_wait_idle("Mode0");
//			            total_dma_mode0++;
//
//			            // --- END MODE 0 WAIT TIMER ---
//			            t_mid2 = get_timer_value(&TimerInstance);
//			            total_dma_wait_ticks += (t_mid2 - t_mid1);

			// After mode0 DMA:
//			total_dma_mode0++;
//			total_targets_sent += current_bite;

			int anchors_remaining = a_count;
			int anchor_offset = 0;

			int max_safe_rx_pkts = 1250000;
			int max_batches = max_safe_rx_pkts / current_bite;
			int dynamic_max_anchors = max_batches * UNROLL_FACTOR;

			if (dynamic_max_anchors > 32768) {
				dynamic_max_anchors = 32768;
			}

			while (anchors_remaining > 0) {

				int current_a_bite =
						(anchors_remaining > dynamic_max_anchors) ?
								dynamic_max_anchors : anchors_remaining;
				int padded_a_count = ((current_a_bite + UNROLL_FACTOR - 1)
						/ UNROLL_FACTOR) * UNROLL_FACTOR;

				for (int a_idx = 0; a_idx < padded_a_count; a_idx++) {
					int src =
							(a_idx < current_a_bite) ?
									(a_off + anchor_offset + a_idx) :
									(a_off + anchor_offset);
					anchor_staging[a_idx] =
							*(RawPointPacket *) &main_points_array[src];\

				}

				int tx_bytes = padded_a_count * sizeof(RawPointPacket);

				Xil_DCacheFlushRange((UINTPTR )anchor_staging, tx_bytes);

				int num_batches = padded_a_count / UNROLL_FACTOR;
				int max_result_pkts = num_batches * current_bite + 1;
				int expected_rx_bytes = max_result_pkts
						* (int) sizeof(DistPacket);
				total_calcs += (unsigned long long) current_a_bite	* current_bite;

//				total_calcs += (unsigned long long) padded_a_count
//						* current_bite;
//
//			       t_start = get_timer_value(&TimerInstance);

				Xil_Out32(IP_NUM_POINTS, current_bite);
				Xil_Out32(IP_NUM_DIMS, num_dims);
				Xil_Out32(IP_MAX_RANGE, max_range_bits);
				Xil_Out32(IP_NUM_ANCHORS, padded_a_count);
				Xil_Out32(IP_MODE, 0x01);
				Xil_Out32(IP_CTRL, 0x01); // ap_start

				Xil_Out32(DMA_BASE + 0x30, 0x0001);
				Xil_Out32(DMA_BASE + 0x48, (UINTPTR) all_results);
				Xil_Out32(DMA_BASE + 0x58, expected_rx_bytes);

				Xil_Out32(DMA_BASE + 0x00, 0x0001);
				Xil_Out32(DMA_BASE + 0x18, (UINTPTR) anchor_staging);
				Xil_Out32(DMA_BASE + 0x28, tx_bytes);

				// --- END MODE 1 SETUP TIMER / START MODE 1 WAIT TIMER ---
//				                t_mid1 = get_timer_value(&TimerInstance);
//				                total_axi_setup_ticks += (t_mid1 - t_start);

				                dma_tx_wait("Mode1 anchor TX");
				                dma_rx_wait("Mode1 result RX");
				                ip_wait_idle("Mode1");
//				                total_dma_mode1++;
//
//				                // --- END MODE 1 WAIT TIMER / START PARSE TIMER ---
//				                t_mid2 = get_timer_value(&TimerInstance);
//				                total_dma_wait_ticks += (t_mid2 - t_mid1);

				// After mode1 DMA inside anchor loop:
//				total_dma_mode1++;
//				total_anchors_sent += padded_a_count;

				int actual_rx_bytes = Xil_In32(DMA_BASE + 0x58);
				Xil_DCacheInvalidateRange((UINTPTR )all_results,
						actual_rx_bytes);

				int actual_rx_pkts = actual_rx_bytes / (int) sizeof(DistPacket);

				for (int p = 0; p < actual_rx_pkts; p++) {

					DistPacket res = all_results[p];
					if (res.target_id < 0)
						break;  // terminator

					int b = res.anchor_b_idx;
					int a_idx = b * UNROLL_FACTOR;

					int local_t_id = res.target_id;
					int global_stage_idx = target_offset + local_t_id;
					int t_id = staging_ids[global_stage_idx];

					for (int a = 0; a < UNROLL_FACTOR; a++) {
						if (a_idx + a >= current_a_bite)
							continue;

						float ds = res.distances[a];

						if (ds < 0.0f || ds > max_range_sq)
							continue;

						int a_id = main_points_array[a_off + anchor_offset
								+ a_idx + a].original_id;

						if (global_stage_idx < center_vox_count) {
							if (a_id >= t_id)
								continue;
						} else {
							if (a_id == t_id)
								continue;
						}

						if (match_count < MAX_MATCHES) {
							match_count++;
						}
					}
				}
				// --- END PARSE TIMER ---
//				                t_end = get_timer_value(&TimerInstance);
//				                total_parse_ticks += (t_end - t_mid2);
				anchors_remaining -= current_a_bite;
				anchor_offset += current_a_bite;

			} // end anchor chunk loop

			targets_remaining -= current_bite;
			target_offset += current_bite;

		} // end target bite loop
		// Print every 10 bins for 1000 points:
//		if (i % 10 == 0) {
//		    xil_printf("[DBG] bin=%d chunk_pts=%d a_count=%d\n\r",
//		               i, chunk_pts, a_count);
//		}
	}
	u64 time_end = get_timer_value(&TimerInstance);

	    // =========================================================
	    //  RESULTS
//	     =========================================================
	    xil_printf(" Matches: %d\n\r", match_count);

	    unsigned int calcs_hi = (unsigned int)(total_calcs >> 32);
	    unsigned int calcs_lo = (unsigned int)(total_calcs & 0xFFFFFFFFUL);
	    if (calcs_hi > 0) xil_printf(" Calc:   %u%09u\n\r", calcs_hi, calcs_lo);
	    else              xil_printf(" Calc:   %u\n\r", calcs_lo);

	    // ---------------------------------------------------------
	    //  ROBUST TIMING
	    // ---------------------------------------------------------
	    u64 total_ticks = (time_end > time_start_total) ? (time_end - time_start_total) : 0;
	    u64 hw_ticks    = (time_end > time_start_hw)    ? (time_end - time_start_hw)    : 0;
	    u64 pre_ticks   = (time_start_hw > time_start_total) ? (time_start_hw - time_start_total) : 0;

	    u32 total_sec = (u32)(total_ticks / TIMER_FREQ_HZ);
	    u32 hw_sec    = (u32)(hw_ticks    / TIMER_FREQ_HZ);
	    u32 pre_sec   = (u32)(pre_ticks   / TIMER_FREQ_HZ);

	    u32 total_ms = (u32)((total_ticks % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));
	    u32 hw_ms    = (u32)((hw_ticks    % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));
	    u32 pre_ms   = (u32)((pre_ticks   % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));

	    u32 total_min   = total_sec / 60; u32 total_s_rem = total_sec % 60;
	    u32 hw_min      = hw_sec / 60;    u32 hw_s_rem    = hw_sec % 60;
	    u32 pre_min     = pre_sec / 60;   u32 pre_s_rem   = pre_sec % 60;

	    xil_printf("Pre-HW :   %lu min %lu.%03lu sec\n\r", pre_min,   pre_s_rem,   pre_ms);
	    xil_printf("HW Loop :  %lu min %lu.%03lu sec\n\r", hw_min,    hw_s_rem,    hw_ms);
	    xil_printf("Total :    %lu min %lu.%03lu sec\n\r", total_min, total_s_rem, total_ms);
//
//	    // --- BREAKDOWN PRINTOUTS ---
//	    u32 setup_s  = (u32)(total_axi_setup_ticks / TIMER_FREQ_HZ);
//	    u32 setup_ms = (u32)((total_axi_setup_ticks % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));
//
//	    u32 wait_s   = (u32)(total_dma_wait_ticks / TIMER_FREQ_HZ);
//	    u32 wait_ms  = (u32)((total_dma_wait_ticks % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));
//
//	    u32 parse_s  = (u32)(total_parse_ticks / TIMER_FREQ_HZ);
//	    u32 parse_ms = (u32)((total_parse_ticks % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));
//
//	    xil_printf("\n\r--- HW LOOP BREAKDOWN ---\n\r");
//	    xil_printf("AXI Setup Time: %lu.%03lu sec\n\r", setup_s, setup_ms);
//	    xil_printf("DMA Wait Time:  %lu.%03lu sec\n\r", wait_s, wait_ms);
//	    xil_printf("CPU Parse Time: %lu.%03lu sec\n\r", parse_s, parse_ms);
//	    xil_printf("-------------------------\n\r");

	    // --- PER-CALL LATENCY ANALYSIS ---
//	          int total_calls = total_dma_mode0 + total_dma_mode1;
//
//	          if (total_calls > 0) {
//	              // 200 ticks = 1 microsecond at 200MHz
//	              u32 avg_setup_us = (u32)((total_axi_setup_ticks / total_calls) / 200);
//	              u32 max_m0_us    = (u32)(max_mode0_setup_ticks / 200);
//	              u32 max_m1_us    = (u32)(max_mode1_setup_ticks / 200);
//
//	              xil_printf("\n\r--- PER-CALL AXI LATENCY ---\n\r");
//	              xil_printf("Total Calls : %d\n\r", total_calls);
//	              xil_printf("Avg Setup   : %lu us\n\r", avg_setup_us);
//	              xil_printf("Max Mode 0  : %lu us\n\r", max_m0_us);
//	              xil_printf("Max Mode 1  : %lu us\n\r", max_m1_us);
//	          }
//	          xil_printf("-------------------------\n\r");
//
//	          // Print after loop:
//	          u32 nb_s  = (u32)(total_neighbour_ticks / TIMER_FREQ_HZ);
//	          u32 nb_ms = (u32)((total_neighbour_ticks % TIMER_FREQ_HZ) / (TIMER_FREQ_HZ / 1000));
//	          xil_printf("Neighbour Search Time: %lu.%03lu sec\n\r", nb_s, nb_ms);


	    while (1);
	    return 0;
}
