#include <stdint.h>
#include <string.h>

#define MAX_DIMS          20
#define POINT_PAD         20

typedef struct { float d[POINT_PAD]; } PointMAX;

extern "C" {
void similarity_search(
    PointMAX *dataset, PointMAX *queries, int *results,
    int num_points, int num_queries, int num_dims, float max_range
){
    #pragma HLS INTERFACE m_axi port=dataset bundle=gmem0
    #pragma HLS INTERFACE m_axi port=queries bundle=gmem1
    #pragma HLS INTERFACE m_axi port=results bundle=gmem2
    #pragma HLS INTERFACE s_axilite port=num_points
    #pragma HLS INTERFACE s_axilite port=num_queries
    #pragma HLS INTERFACE s_axilite port=num_dims
    #pragma HLS INTERFACE s_axilite port=max_range
    #pragma HLS INTERFACE s_axilite port=return

    if(num_dims < 1 || num_dims > MAX_DIMS) return;
    float max_range_sq = max_range * max_range;

    for(int q = 0; q < num_queries; q++){
        results[q] = 0;
    }

    for(int q = 0; q < num_queries; q++){
        PointMAX qp = queries[q];
        int count = 0;
        for(int i = 0; i < num_points; i++){
            PointMAX tp = dataset[i];
            float dist_sq = 0.0f;
            for(int d = 0; d < MAX_DIMS; d++){
                float diff = (d < num_dims) ? (tp.d[d] - qp.d[d]) : 0.0f;
                dist_sq += diff * diff;
            }
            if(dist_sq <= max_range_sq) count++;
        }
        results[q] = count;
    }
}
}