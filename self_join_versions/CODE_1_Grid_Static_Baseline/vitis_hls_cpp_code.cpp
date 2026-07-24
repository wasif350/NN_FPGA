#include "hls_stream.h"
#include "ap_axi_sdata.h"

// ============================================================
//  HARDWARE CONSTANTS (OPTIMIZED FOR ARTIX-7 100T SYSTEM)
// ============================================================
#define MAX_DIMS          8
#define UNROLL_FACTOR     4
#define MAX_BRAM_CAPACITY 5000
#define MAX_ANCHORS       2048  // Reduced to 2048 to leave BRAM for MicroBlaze & DMA

struct PointPacket {
	float d[MAX_DIMS];
};

struct DistPacket {
	float distances[UNROLL_FACTOR]; // 16 bytes
	int target_id;                //  4 bytes
	int anchor_b_idx;             //  4 bytes
	int pad1, pad2;               //  8 bytes
};
// 32 bytes total

typedef hls::axis<PointPacket, 0, 0, 0> in_pkt;
typedef hls::axis<DistPacket, 0, 0, 0> out_pkt;

// =========================================================
//  SUB-FUNCTION 1: DMA READ
// =========================================================
void load_anchor_chunk(hls::stream<in_pkt> &in_stream,
		float anchor_buf[MAX_ANCHORS][MAX_DIMS], int chunk_size) {
			for (int a = 0; a < chunk_size; a++) {
				// FIX: Tripcount aligned to new 2048 anchor limit
						#pragma HLS LOOP_TRIPCOUNT min=4 max=2048
						#pragma HLS PIPELINE II=1
						in_pkt pkt = in_stream.read();
						for (int d = 0; d < MAX_DIMS; d++) {
							#pragma HLS UNROLL
							anchor_buf[a][d] = pkt.data.d[d];
						}
			}
}

// =========================================================
//  SUB-FUNCTION 2: MATH PIPELINE
// =========================================================
void compute_anchor_chunk(float anchor_buf[MAX_ANCHORS][MAX_DIMS],
		float target_cache[MAX_BRAM_CAPACITY][MAX_DIMS],
		hls::stream<out_pkt> &out_stream, int chunk_size, int num_points,
		int num_dims, float max_range_sq, int chunk_offset) {
		// TIMING FIX: Bitwise right-shift by 2 replaces hardware division by 4
		int anchor_batches = chunk_size >> 2;

		for (int b = 0; b < anchor_batches; b++) {
			// FIX: 2048 anchors / 4 = 512 max loop iterations
					#pragma HLS LOOP_TRIPCOUNT min=1 max=512
					float anchors[UNROLL_FACTOR][MAX_DIMS];
					#pragma HLS ARRAY_PARTITION variable=anchors complete dim=0

					for (int a = 0; a < UNROLL_FACTOR; a++) {
						#pragma HLS UNROLL
						for (int d = 0; d < MAX_DIMS; d++) {
							#pragma HLS UNROLL
							// TIMING FIX: Bitwise left-shift replaces multiplication (b * 4)
							anchors[a][d] = anchor_buf[(b << 2) + a][d];
						}
					}

			for (int i = 0; i < num_points; i++) {
				#pragma HLS LOOP_TRIPCOUNT min=1 max=5000
				#pragma HLS PIPELINE II=1

							// TIMING FIX: FF Shock Absorber to fix BRAM Fan-out delay
			    float local_target[MAX_DIMS];
				#pragma HLS ARRAY_PARTITION variable=local_target complete dim=1
				for (int d = 0; d < MAX_DIMS; d++) {
								#pragma HLS UNROLL
								local_target[d] = target_cache[i][d];
				}

				out_pkt res;
			    bool has_match = false;

				for (int a = 0; a < UNROLL_FACTOR; a++) {
						#pragma HLS UNROLL
						float sq[MAX_DIMS];
						#pragma HLS ARRAY_PARTITION variable=sq complete dim=1

						for (int d = 0; d < MAX_DIMS; d++) {
								#pragma HLS UNROLL
								// Reads from fast local_target FFs instead of slow BRAM
								float diff =(d < num_dims) ? (local_target[d] - anchors[a][d]) : 0.0f;
								sq[d] = diff * diff;
							}

				// Balanced adder tree
				float s01 = sq[0] + sq[1];
				float s23 = sq[2] + sq[3];
				float s45 = sq[4] + sq[5];
				float s67 = sq[6] + sq[7];

				float s0123 = s01 + s23;
				float s4567 = s45 + s67;

				float dist_sq = s0123 + s4567;

				if (dist_sq <= max_range_sq) {
					res.data.distances[a] = dist_sq;
					has_match = true;
				} else {
					res.data.distances[a] = -1.0f;
				}
			}

			if (has_match) {
				res.data.target_id = i;
				res.data.anchor_b_idx = chunk_offset + b;
				res.keep = -1;
				res.strb = -1;
				res.last = 0;
				out_stream.write(res);
			}
		}
	}
}

// =========================================================
//  MAIN KERNEL
// =========================================================
void ndim_dist_finder(hls::stream<in_pkt> &in_stream,
		hls::stream<out_pkt> &out_stream, int num_points, int num_dims,
		float max_range, int mode, int num_anchors) {
	#pragma HLS INTERFACE axis      port=in_stream
	#pragma HLS INTERFACE axis      port=out_stream
	#pragma HLS INTERFACE s_axilite port=num_points
	#pragma HLS INTERFACE s_axilite port=num_dims
	#pragma HLS INTERFACE s_axilite port=max_range
	#pragma HLS INTERFACE s_axilite port=mode
	#pragma HLS INTERFACE s_axilite port=num_anchors
	#pragma HLS INTERFACE s_axilite port=return

	if (num_dims < 1 || num_dims > MAX_DIMS)
		return;

	static float target_cache[MAX_BRAM_CAPACITY][MAX_DIMS];
	#pragma HLS BIND_STORAGE variable=target_cache type=ram_2p impl=bram
	#pragma HLS ARRAY_PARTITION variable=target_cache complete dim=2

	// ---------------------------------------------------------
	//  MODE 0: LOAD TARGETS INTO BRAM
	// ---------------------------------------------------------
	if (mode == 0) {
		for (int i = 0; i < num_points; i++) {
			#pragma HLS LOOP_TRIPCOUNT min=1 max=5000
			#pragma HLS PIPELINE II=1
			in_pkt pkt = in_stream.read();
			for (int d = 0; d < MAX_DIMS; d++) {
				#pragma HLS UNROLL
				target_cache[i][d] = pkt.data.d[d];
			}
		}
	}

	// ---------------------------------------------------------
	//  MODE 1: COMPUTE WITH DYNAMIC SEQUENTIAL CHUNKING
	// ---------------------------------------------------------
	else if (mode == 1) {
		float max_range_sq = max_range * max_range;

		int processed = 0;

		while (processed < num_anchors) {
			#pragma HLS LOOP_TRIPCOUNT min=1 max=250

			int remaining = num_anchors - processed;
			int current_chunk_size =
					(remaining > MAX_ANCHORS) ? MAX_ANCHORS : remaining;

			// DATAFLOW is intentionally omitted here to ensure stable
			// sequential routing for the Artix-7 100MHz clock limit.

			float anchor_buf[MAX_ANCHORS][MAX_DIMS];
			#pragma HLS BIND_STORAGE variable=anchor_buf type=ram_2p impl=bram
			#pragma HLS ARRAY_PARTITION variable=anchor_buf cyclic factor=4 dim=1
			#pragma HLS ARRAY_PARTITION variable=anchor_buf complete dim=2

			// 1. DMA Load
			load_anchor_chunk(in_stream, anchor_buf, current_chunk_size);

			// 2. Math Execution
			// TIMING FIX: Bitwise right-shift replaces (processed / UNROLL_FACTOR)
			compute_anchor_chunk(anchor_buf, target_cache, out_stream,
					current_chunk_size, num_points, num_dims, max_range_sq,
					processed >> 2);

			processed += current_chunk_size;
		}

		// Send Terminator Packet
		out_pkt end_pkt;
		end_pkt.data.target_id = -1;
		end_pkt.data.anchor_b_idx = -1;
		for (int a = 0; a < UNROLL_FACTOR; a++)
			end_pkt.data.distances[a] = -1.0f;
		end_pkt.data.pad1 = 0;
		end_pkt.data.pad2 = 0;
		end_pkt.keep = -1;
		end_pkt.strb = -1;
		end_pkt.last = 1;
		out_stream.write(end_pkt);
	}
}