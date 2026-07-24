
// reducing resources to have two copies of i=IP

#include "hls_stream.h"
#include "ap_axi_sdata.h"

// ============================================================
//  HARDWARE CONSTANTS (OPTIMIZED FOR DUAL-CORE ARTIX-7)
// ============================================================
#define MAX_DIMS          6
#define UNROLL_FACTOR     2
// REDUCED to fit two copies of the IP into available BRAM
//#define MAX_BRAM_CAPACITY 2000
#define MAX_BRAM_CAPACITY 2500
#define MAX_ANCHORS       1024

struct PointPacket {
   float d[MAX_DIMS];
   float pad1;        // 4 bytes padding
   float pad2;        // 4 bytes padding
};

struct DistPacket {
   float distances[UNROLL_FACTOR]; // 8 bytes (2 floats)
   int target_id;                  // 4 bytes
   int anchor_b_idx;               // 4 bytes
   int pad1;                       // 4 bytes
   int pad2;                       // 4 bytes
   int pad3;                       // 4 bytes
   int pad4;
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
       // FIX: Tripcount aligned to new 1024 anchor limit
       #pragma HLS LOOP_TRIPCOUNT min=4 max=1024
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
#pragma HLS LATENCY min=20 max=30

   int anchor_batches = chunk_size >> 1;

   for (int b = 0; b < anchor_batches; b++) {
       // FIX: 1024 anchors / 4 = 256 max loop iterations
       #pragma HLS LOOP_TRIPCOUNT min=1 max=512
       float anchors[UNROLL_FACTOR][MAX_DIMS];
       #pragma HLS ARRAY_PARTITION variable=anchors complete dim=0

       for (int a = 0; a < UNROLL_FACTOR; a++) {
           #pragma HLS UNROLL
           for (int d = 0; d < MAX_DIMS; d++) {
               #pragma HLS UNROLL
               // TIMING FIX: Bitwise left-shift replaces multiplication (b * 4)
               anchors[a][d] = anchor_buf[(b << 1) + a][d];
           }
       }

       for (int i = 0; i < num_points; i++) {
           // FIX: Tripcount aligned to new 2000 target limit
           #pragma HLS LOOP_TRIPCOUNT min=1 max=2048
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
                   float diff = (d < num_dims) ? (local_target[d] - anchors[a][d]) : 0.0f;
                   float sq_val = diff * diff;
#pragma HLS BIND_OP variable=sq_val op=fmul impl=dsp
                   sq[d] = sq_val;
               }

               // Balanced adder tree (Adders naturally use LUTs/Carry logic)
               float s01 = sq[0] + sq[1];
               float s23 = sq[2] + sq[3];
               float s45 = sq[4] + sq[5];

               float sp = s01 + s23;

               float dist_sq = sp + s45;

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
           // FIX: Tripcount aligned to new 2000 target limit
           #pragma HLS LOOP_TRIPCOUNT min=1 max=2500
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
           #pragma HLS LOOP_TRIPCOUNT min=1 max=256

           int remaining = num_anchors - processed;
           int current_chunk_size =
                   (remaining > MAX_ANCHORS) ? MAX_ANCHORS : remaining;

           float anchor_buf[MAX_ANCHORS][MAX_DIMS];
           #pragma HLS BIND_STORAGE variable=anchor_buf type=ram_2p impl=lutram
           #pragma HLS ARRAY_PARTITION variable=anchor_buf cyclic factor=2 dim=1
           #pragma HLS ARRAY_PARTITION variable=anchor_buf complete dim=2

           // 1. DMA Load
           load_anchor_chunk(in_stream, anchor_buf, current_chunk_size);

           // 2. Math Execution
           compute_anchor_chunk(anchor_buf, target_cache, out_stream,
                   current_chunk_size, num_points, num_dims, max_range_sq,
                   processed >> 1);

           processed += current_chunk_size;
       }

       // Send Terminator Packet
       out_pkt end_pkt;
       end_pkt.data.target_id = -1;
       end_pkt.data.anchor_b_idx = -1;
       for (int a = 0; a < UNROLL_FACTOR; a++)
           end_pkt.data.distances[a] = -1.0f;
       end_pkt.data.pad1 = 0;
       end_pkt.keep = -1;
       end_pkt.strb = -1;
       end_pkt.last = 1;
       out_stream.write(end_pkt);
   }
}