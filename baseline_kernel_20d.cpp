#include <stdint.h>
#include <string.h>

#define MAX_DIMS          20
#define POINT_PAD         20
#define UNROLL_FACTOR      15
#define MAX_BRAM_CAPACITY 22000
#define MAX_ANCHORS       15000

typedef struct { float d[POINT_PAD]; } PointMAX;

void load_anchor_chunk(PointMAX *queries, float anchor_buf[MAX_ANCHORS][MAX_DIMS],
    int chunk_start, int chunk_size, int num_dims){
    for(int a=0;a<chunk_size;a++){
        #pragma HLS LOOP_TRIPCOUNT min=4 max=15000
        #pragma HLS PIPELINE II=2
        PointMAX q=queries[chunk_start+a];
        for(int d=0;d<MAX_DIMS;d++){
            #pragma HLS UNROLL
            anchor_buf[a][d]=(d<num_dims)?q.d[d]:0.0f;
        }
    }
}

void compute_anchor_chunk(float anchor_buf[MAX_ANCHORS][MAX_DIMS],
    float target_cache[MAX_BRAM_CAPACITY][MAX_DIMS], int *results,
    int query_offset, int chunk_size, int num_points, int num_dims,
    float max_range_sq, int chunk_offset){
    
    int anchor_batches=chunk_size/UNROLL_FACTOR;
    for(int b=0;b<anchor_batches;b++){
        #pragma HLS LOOP_TRIPCOUNT min=1 max=12000
        float anchors[UNROLL_FACTOR][MAX_DIMS];
        #pragma HLS ARRAY_PARTITION variable=anchors complete dim=0
        for(int a=0;a<UNROLL_FACTOR;a++){
            #pragma HLS UNROLL
            for(int d=0;d<MAX_DIMS;d++){
                #pragma HLS UNROLL
                anchors[a][d]=anchor_buf[b*UNROLL_FACTOR+a][d];
            }
        }
        int counts[UNROLL_FACTOR];
        #pragma HLS ARRAY_PARTITION variable=counts complete dim=1
        for(int a=0;a<UNROLL_FACTOR;a++){
             #pragma HLS UNROLL
            counts[a]=0; }
        for(int i=0;i<num_points;i++){
            #pragma HLS LOOP_TRIPCOUNT min=1 max=22000
            #pragma HLS PIPELINE II=1
            float local_target[MAX_DIMS];
            #pragma HLS ARRAY_PARTITION variable=local_target complete dim=1
            for(int d=0;d<MAX_DIMS;d++){ 
                #pragma HLS UNROLL
                local_target[d]=target_cache[i][d]; }
            for(int a=0;a<UNROLL_FACTOR;a++){
                #pragma HLS UNROLL
                float dist_sq=0.0f;
                for(int d=0;d<POINT_PAD;d++){
                    #pragma HLS UNROLL
                    float diff=(d<num_dims)?(local_target[d]-anchors[a][d]):0.0f;
                    dist_sq+=diff*diff;
                }
                if(dist_sq<=max_range_sq) counts[a]++;
            }
        }
        for(int a=0;a<UNROLL_FACTOR;a++){
            #pragma HLS UNROLL
            int q_idx=query_offset+(chunk_offset+b)*UNROLL_FACTOR+a;
            results[q_idx]+=counts[a];
        }
    }
}

extern "C" {
void similarity_search(
    PointMAX *dataset, PointMAX *queries, int *results,
    int num_points, int num_queries, int num_dims, float max_range
){
    #pragma HLS INTERFACE m_axi port=dataset bundle=gmem0 \
        max_read_burst_length=256 num_read_outstanding=16
    #pragma HLS INTERFACE m_axi port=queries bundle=gmem1 \
        max_read_burst_length=32
    #pragma HLS INTERFACE m_axi port=results bundle=gmem2 \
        max_write_burst_length=32
    #pragma HLS INTERFACE s_axilite port=num_points
    #pragma HLS INTERFACE s_axilite port=num_queries
    #pragma HLS INTERFACE s_axilite port=num_dims
    #pragma HLS INTERFACE s_axilite port=max_range
    #pragma HLS INTERFACE s_axilite port=return

    if(num_dims<1||num_dims>MAX_DIMS) return;
    float max_range_sq=max_range*max_range;

    static float target_cache[MAX_BRAM_CAPACITY][MAX_DIMS];
    #pragma HLS BIND_STORAGE variable=target_cache type=ram_2p impl=uram
    #pragma HLS ARRAY_PARTITION variable=target_cache complete dim=2

    for(int q=0;q<num_queries;q++){
        #pragma HLS PIPELINE II=1
        results[q]=0;
    }

    for(int t0=0;t0<num_points;t0+=MAX_BRAM_CAPACITY){
        #pragma HLS LOOP_TRIPCOUNT min=1 max=5000
        int tile_n=(num_points-t0<MAX_BRAM_CAPACITY)?(num_points-t0):MAX_BRAM_CAPACITY;
        for(int i=0;i<tile_n;i++){
            #pragma HLS LOOP_TRIPCOUNT min=1 max=22000
            #pragma HLS PIPELINE II=2
            PointMAX p=dataset[t0+i];
            for(int d=0;d<MAX_DIMS;d++){
                #pragma HLS UNROLL
                target_cache[i][d]=(d<num_dims)?p.d[d]:0.0f;
            }
        }
        float anchor_buf[MAX_ANCHORS][MAX_DIMS];
        #pragma HLS BIND_STORAGE variable=anchor_buf type=ram_2p impl=bram
        #pragma HLS ARRAY_PARTITION variable=anchor_buf cyclic factor=15 dim=1
        #pragma HLS ARRAY_PARTITION variable=anchor_buf complete dim=2

        int processed=0;
        while(processed<num_queries){
            int remaining=num_queries-processed;
            int chunk_size=(remaining>MAX_ANCHORS)?MAX_ANCHORS:remaining;
            chunk_size=(chunk_size/UNROLL_FACTOR)*UNROLL_FACTOR;
            if(chunk_size==0) break;
            load_anchor_chunk(queries,anchor_buf,processed,chunk_size,num_dims);
            compute_anchor_chunk(anchor_buf,target_cache,results,0,
                chunk_size,tile_n,num_dims,max_range_sq,
                processed/UNROLL_FACTOR);
            processed+=chunk_size;
        }
    }
}
}
