#include <xrt/xrt_bo.h>
#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <cfloat>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <omp.h>

#define CAND_CHUNK 200000

using Clock = std::chrono::high_resolution_clock;
static double ms_since(Clock::time_point t){
    return std::chrono::duration<double,std::milli>(Clock::now()-t).count();
}

struct BinMeta { int bin_id; int original_id; };
struct BinInfo  { int bin_id; int offset; int count; };
struct CordInfo { int dim_id; float cord_min; float cord_max; float cord_spread; };
struct QueryBin { int bin_id; int query_idx; };
struct GroupResult {
    std::vector<int> cand_mp_idx;
    std::vector<int> query_map;
    int gsz;
};

// Precompute all 3^K neighbor offsets for a K-dimensional grid
static std::vector<std::vector<int>> gen_neighbor_offsets(int K){
    std::vector<std::vector<int>> off = {{}};
    for(int k=0;k<K;k++){
        std::vector<std::vector<int>> next;
        next.reserve(off.size()*3);
        for(auto& o:off)
            for(int d=-1;d<=1;d++){
                auto no=o; no.push_back(d); next.push_back(std::move(no));
            }
        off=std::move(next);
    }
    return off;
}

// Compute K-D bin ID from a point's float data
static inline int compute_bin(const float* pt, int ps,
                               const std::vector<int>& idx_dims,
                               const std::vector<CordInfo>& ci,
                               const std::vector<int>& nb_dims,
                               const std::vector<float>& inv_vox,
                               const std::vector<int>& strides, int K){
    int bin=0;
    for(int k=0;k<K;k++){
        int d=idx_dims[k];
        int bk=std::min(nb_dims[k]-1,
               std::max(0,(int)((pt[d]-ci[d].cord_min)*inv_vox[k])));
        bin+=bk*strides[k];
    }
    return bin;
}

bool stage1_load(const std::string& fn,
                 std::vector<float>& pts, int& nd, int& np, int ps)
{
    std::ifstream f(fn,std::ios::binary);
    if(!f){ std::cerr<<"Cannot open: "<<fn<<"\n"; return false; }
    f.read((char*)&nd,4); f.read((char*)&np,4);
    pts.assign((size_t)np*ps,0.0f);
    for(int i=0;i<np;i++)
        f.read((char*)(pts.data()+(size_t)i*ps), nd*sizeof(float));
    return true;
}

void stage1b_coord_info(const std::vector<float>& pts, int nd, int np,
                         std::vector<CordInfo>& ci, std::vector<int>& top, int ps)
{
    ci.resize(nd);
    for(int d=0;d<nd;d++){ci[d].dim_id=d;ci[d].cord_min=FLT_MAX;ci[d].cord_max=-FLT_MAX;}
    for(int i=0;i<np;i++)
        for(int d=0;d<nd;d++){
            float v=pts[(size_t)i*ps+d];
            if(v<ci[d].cord_min)ci[d].cord_min=v;
            if(v>ci[d].cord_max)ci[d].cord_max=v;
        }
    for(int d=0;d<nd;d++) ci[d].cord_spread=(ci[d].cord_max-ci[d].cord_min)+0.0001f;
    top.resize(nd); std::iota(top.begin(),top.end(),0);
    std::sort(top.begin(),top.end(),
        [&](int a,int b){return ci[a].cord_spread>ci[b].cord_spread;});
}

// Build K-D grid
void stage2_build_grid(const std::vector<float>& ds, int nd, int np,
                        const std::vector<CordInfo>& ci, const std::vector<int>& top,
                        float radius, int K,
                        std::vector<float>& mp_data, std::vector<BinMeta>& mp_meta,
                        std::vector<int>& nb_dims, std::vector<float>& inv_vox,
                        std::vector<int>& idx_dims, std::vector<int>& strides, int ps)
{
    K=std::min(K,nd);
    idx_dims.assign(top.begin(), top.begin()+K);

    float vs=radius; long long tot;
    do{
        nb_dims.resize(K);
        for(int k=0;k<K;k++)
            nb_dims[k]=std::max(1,(int)(ci[idx_dims[k]].cord_spread/vs));
        tot=1; for(int k=0;k<K;k++) tot*=nb_dims[k];
        if(tot>2000000000LL) vs*=1.5f;
    }while(tot>2000000000LL);

    inv_vox.resize(K);
    for(int k=0;k<K;k++)
        inv_vox[k]=(float)nb_dims[k]/ci[idx_dims[k]].cord_spread;

    strides.resize(K); strides[K-1]=1;
    for(int k=K-2;k>=0;k--) strides[k]=strides[k+1]*nb_dims[k+1];

    mp_data.resize((size_t)np*ps);
    mp_meta.resize(np);
    for(int i=0;i<np;i++){
        memcpy(mp_data.data()+(size_t)i*ps, ds.data()+(size_t)i*ps, ps*sizeof(float));
        mp_meta[i].original_id=i;
        mp_meta[i].bin_id=compute_bin(ds.data()+(size_t)i*ps,ps,idx_dims,ci,nb_dims,inv_vox,strides,K);
    }
}

void stage3_sort(std::vector<float>& mp_data, std::vector<BinMeta>& mp_meta, int ps)
{
    int np=(int)mp_meta.size();
    std::vector<int> idx(np); std::iota(idx.begin(),idx.end(),0);
    std::sort(idx.begin(),idx.end(),
        [&](int a,int b){return mp_meta[a].bin_id<mp_meta[b].bin_id;});
    std::vector<float>   tmp_d((size_t)np*ps);
    std::vector<BinMeta> tmp_m(np);
    for(int i=0;i<np;i++){
        memcpy(tmp_d.data()+(size_t)i*ps, mp_data.data()+(size_t)idx[i]*ps, ps*sizeof(float));
        tmp_m[i]=mp_meta[idx[i]];
    }
    mp_data=std::move(tmp_d); mp_meta=std::move(tmp_m);
}

void stage4_build_bins(const std::vector<BinMeta>& mp_meta,
                        std::vector<BinInfo>& ub,
                        std::unordered_map<int,int>& bin_map)
{
    ub.clear(); bin_map.clear(); int cur=-1;
    for(int i=0;i<(int)mp_meta.size();i++){
        int b=mp_meta[i].bin_id;
        if(b!=cur){ bin_map[b]=(int)ub.size(); ub.push_back({b,i,1}); cur=b; }
        else ub.back().count++;
    }
}

long long stage5_query_binning_opt(
    const std::vector<float>& queries, int qnp, int nd,
    const std::vector<float>& mp_data,
    const std::vector<BinMeta>& mp_meta,
    const std::vector<BinInfo>& ub,
    const std::unordered_map<int,int>& bin_map,
    const std::vector<CordInfo>& ci,
    const std::vector<int>& nb_dims,
    const std::vector<float>& inv_vox,
    const std::vector<int>& idx_dims,
    const std::vector<int>& strides,
    int K,
    float radius,
    std::vector<xrt::kernel>& ku, int num_cus,
    std::vector<xrt::bo>& bo_ds,
    std::vector<xrt::bo>& bo_qs,
    std::vector<xrt::bo>& bo_rs,
    double& out_kernel_ms, double& out_xfer_ms,
    long long& out_total_cands, long long& out_actual_pairs,
    int ps, int unroll_factor)
{
    int np=(int)mp_meta.size();
    auto neighbor_offsets=gen_neighbor_offsets(K);

    // Assign each query to its K-D bin
    std::vector<QueryBin> qb(qnp);
    for(int qi=0;qi<qnp;qi++){
        int bin=compute_bin(queries.data()+(size_t)qi*ps,ps,idx_dims,ci,nb_dims,inv_vox,strides,K);
        qb[qi]={bin,qi};
    }
    std::sort(qb.begin(),qb.end(),
        [](const QueryBin& a,const QueryBin& b){return a.bin_id<b.bin_id;});

    struct Group{ int start; int end; int bin_id; };
    std::vector<Group> groups;
    for(int qi=0;qi<qnp;){
        int gbin=qb[qi].bin_id, gend=qi+1;
        while(gend<qnp && qb[gend].bin_id==gbin) gend++;
        groups.push_back({qi,gend,gbin}); qi=gend;
    }
    int ngroups=(int)groups.size();

    std::vector<GroupResult> group_results(ngroups);

    #pragma omp parallel
    {
        std::vector<int> local_epoch(np,0); int local_epoch_val=0;
        #pragma omp for schedule(dynamic,4)
        for(int gi=0;gi<ngroups;gi++){
            Group& g=groups[gi];
            int gbin=g.bin_id, gsz=g.end-g.start;
            local_epoch_val++;
            // Decode bin_id → K-D coordinates
            std::vector<int> g_coords(K); int tmp=gbin;
            for(int k=K-1;k>=0;k--){ g_coords[k]=tmp%nb_dims[k]; tmp/=nb_dims[k]; }

            std::vector<int> cand_mp_idx;
            cand_mp_idx.reserve(neighbor_offsets.size()*(np/std::max(1,(int)ub.size())));
            for(auto& off:neighbor_offsets){
                bool valid=true; int nbin=0;
                for(int k=0;k<K;k++){
                    int nc=g_coords[k]+off[k];
                    if(nc<0||nc>=nb_dims[k]){valid=false;break;}
                    nbin+=nc*strides[k];
                }
                if(!valid) continue;
                auto it=bin_map.find(nbin);
                if(it==bin_map.end()) continue;
                int idx=it->second;
                int off2=ub[idx].offset, cnt=ub[idx].count;
                for(int p=0;p<cnt;p++){
                    int pid=mp_meta[off2+p].original_id;
                    if(local_epoch[pid]!=local_epoch_val){
                        local_epoch[pid]=local_epoch_val;
                        cand_mp_idx.push_back(off2+p);
                    }
                }
            }
            std::vector<int> query_map(gsz);
            for(int gg=0;gg<gsz;gg++) query_map[gg]=qb[g.start+gg].query_idx;
            group_results[gi]={std::move(cand_mp_idx),std::move(query_map),gsz};
        }
    }

    std::vector<int> total_matches(qnp,0);
    long long grand_total=0, total_cands=0, actual_pairs=0;
    double kernel_ms=0, xfer_ms=0;
    std::vector<std::vector<float>> cu_upload(num_cus,
        std::vector<float>((size_t)CAND_CHUNK*ps));

    // Dispatch: one group at a time. All num_cus CUs process different CHUNKS
    // of the same group's candidates simultaneously — correct for large groups
    // (SUSY: ~54 groups each with ~1.7M candidates >> CAND_CHUNK).
    for(int gi=0;gi<ngroups;gi++){
        GroupResult& gr=group_results[gi];
        int gsz=gr.gsz, total_cand=(int)gr.cand_mp_idx.size();
        if(total_cand==0||gsz==0) continue;

        int qpad=((gsz+unroll_factor-1)/unroll_factor)*unroll_factor;
        std::vector<float> q_flat((size_t)qpad*ps,0.0f);
        for(int gg=0;gg<gsz;gg++)
            memcpy(q_flat.data()+(size_t)gg*ps,
                   queries.data()+(size_t)gr.query_map[gg]*ps, ps*sizeof(float));

        // Upload queries to all CUs (same group, all CUs need same queries)
        auto t_x0=Clock::now();
        size_t q_bytes=(size_t)qpad*ps*sizeof(float);
        for(int c=0;c<num_cus;c++){
            float* m=bo_qs[c].map<float*>();
            memcpy(m,q_flat.data(),q_bytes);
            bo_qs[c].sync(XCL_BO_SYNC_BO_TO_DEVICE,q_bytes,0);
        }
        xfer_ms+=ms_since(t_x0);

        int c0=0;
        while(c0<total_cand){
            int active_cus=0;
            std::vector<int> chunk_sizes(num_cus,0);
            for(int c=0;c<num_cus&&c0<total_cand;c++,active_cus++){
                int this_chunk=std::min(CAND_CHUNK,total_cand-c0);
                chunk_sizes[c]=this_chunk;
                // Upload dataset chunk to this CU
                auto t_x1=Clock::now();
                for(int p=0;p<this_chunk;p++)
                    memcpy(cu_upload[c].data()+(size_t)p*ps,
                           mp_data.data()+(size_t)gr.cand_mp_idx[c0+p]*ps, ps*sizeof(float));
                size_t cb=(size_t)this_chunk*ps*sizeof(float);
                float* m=bo_ds[c].map<float*>();
                memcpy(m,cu_upload[c].data(),cb);
                bo_ds[c].sync(XCL_BO_SYNC_BO_TO_DEVICE,cb,0);
                xfer_ms+=ms_since(t_x1);
                c0+=this_chunk;
            }
            // Launch all active CUs simultaneously (different candidate chunks,
            // same queries) — this is where CU parallelism happens for large groups
            auto t_k=Clock::now();
            std::vector<xrt::run> runs(active_cus);
            for(int c=0;c<active_cus;c++)
                runs[c]=ku[c](bo_ds[c],bo_qs[c],bo_rs[c],chunk_sizes[c],qpad,nd,radius);
            for(int c=0;c<active_cus;c++) runs[c].wait();
            kernel_ms+=ms_since(t_k);

            // Readback and accumulate results
            auto t_x2=Clock::now();
            for(int c=0;c<active_cus;c++){
                bo_rs[c].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
                int* rptr=bo_rs[c].map<int*>();
                for(int i=0;i<gsz;i++){
                    total_matches[gr.query_map[i]]+=rptr[i];
                    grand_total+=rptr[i];
                }
            }
            xfer_ms+=ms_since(t_x2);
        }
        total_cands+=total_cand;
        actual_pairs+=(long long)total_cand*qpad;
    }

    out_kernel_ms=kernel_ms; out_xfer_ms=xfer_ms;
    out_total_cands=total_cands; out_actual_pairs=actual_pairs;
    return grand_total;
}

int main(int argc, char* argv[]){
    if(argc<8){
        std::cout<<"Usage: ./host_index_fpga <xclbin> <dataset> <queries>"
                 <<" <radius> <num_cus> <point_pad> <unroll_factor> [num_index_dims=3]\n";
        std::cout<<"  point_pad    : must match kernel POINT_PAD (e.g. 20, 32, 64)\n";
        std::cout<<"  unroll_factor   : must match kernel UNROLL_FACTOR (e.g. 9, 15)\n";
        std::cout<<"  num_index_dims  : how many top-variance dims to use for grid (default 2)\n";
        return 1;
    }
    std::string xclbin=argv[1], ds_file=argv[2], q_file=argv[3];
    float radius       = std::stof(argv[4]);
    int   ncus         = std::stoi(argv[5]);
    int   ps           = std::stoi(argv[6]);
    int   unroll_factor= std::stoi(argv[7]);
    int   K            = (argc>8) ? std::stoi(argv[8]) : 2;

    auto t0=Clock::now();

    std::vector<float> ds,qs; int nd,np,qnd,qnp;
    if(!stage1_load(ds_file,ds,nd,np,ps)) return 1;
    if(!stage1_load(q_file,qs,qnd,qnp,ps)) return 1;
    if(K>nd){ std::cerr<<"num_index_dims "<<K<<" > actual dims "<<nd<<", clamping\n"; K=nd; }
    double file_load_ms = ms_since(t0);   // ← dataset + query loading done

    std::vector<CordInfo> ci; std::vector<int> top;
    stage1b_coord_info(ds,nd,np,ci,top,ps);

    std::vector<float> mp_data; std::vector<BinMeta> mp_meta;
    std::vector<int> nb_dims, idx_dims, strides;
    std::vector<float> inv_vox;
    stage2_build_grid(ds,nd,np,ci,top,radius,K,
                      mp_data,mp_meta,nb_dims,inv_vox,idx_dims,strides,ps);

    stage3_sort(mp_data,mp_meta,ps);

    std::vector<BinInfo> ub; std::unordered_map<int,int> bin_map;
    stage4_build_bins(mp_meta,ub,bin_map);

    double build_ms=ms_since(t0);

    auto t_xrt = Clock::now();
    auto dev=xrt::device(0); auto xcl=dev.load_xclbin(xclbin);
    std::vector<xrt::kernel> ku(ncus);
    for(int c=0;c<ncus;c++){
        std::string kn="similarity_search:{similarity_search_"+std::to_string(c+1)+"}";
        ku[c]=xrt::kernel(dev,xcl,kn);
    }
    int max_q=qnp+unroll_factor*ncus;
    std::vector<xrt::bo> bo_ds_v(ncus),bo_qs_v(ncus),bo_rs_v(ncus);
    for(int c=0;c<ncus;c++){
        bo_ds_v[c]=xrt::bo(dev,(size_t)CAND_CHUNK*ps*sizeof(float),ku[c].group_id(0));
        bo_qs_v[c]=xrt::bo(dev,(size_t)max_q*ps*sizeof(float),ku[c].group_id(1));
        bo_rs_v[c]=xrt::bo(dev,(size_t)max_q*sizeof(int),ku[c].group_id(2));
    }
    double xrt_ms = ms_since(t_xrt);     // ← xclbin load + kernel handles + HBM buffer alloc

    auto t56=Clock::now();
    double kernel_ms=0,xfer_ms=0; long long total_cands=0,actual_pairs=0;
    long long total=stage5_query_binning_opt(
        qs,qnp,nd,mp_data,mp_meta,ub,bin_map,ci,
        nb_dims,inv_vox,idx_dims,strides,K,
        radius,ku,ncus,bo_ds_v,bo_qs_v,bo_rs_v,
        kernel_ms,xfer_ms,total_cands,actual_pairs,ps,unroll_factor);
    double q_ms=ms_since(t56);

    double wall_ms             = ms_since(t0);
    double total_after_load_ms = wall_ms - file_load_ms;
    double index_build_ms      = build_ms - file_load_ms;   // stages 1b–4, no file I/O
    double host_overhead_ms    = q_ms - kernel_ms - xfer_ms; // CPU work in query phase
    // xfer_ms: query uploads (per group) + dataset chunk uploads + result readbacks
    float pruning=100.0f*(1.0f-(float)total_cands/((long long)np*qnp));

    std::cout<<"\n================================================\n";
    std::cout<<"  RESULTS — U280 FPGA Spatial Index (DYNAMIC)\n";
    std::cout<<"================================================\n";
    std::cout<<"Dataset:        "<<np<<" points, "<<nd<<" dims\n";
    std::cout<<"Queries:        "<<qnp<<"\n";
    std::cout<<"Radius:         "<<radius<<"\n";
    std::cout<<"CUs:            "<<ncus<<"\n";
    std::cout<<"Point stride:   "<<ps<<"\n";
    std::cout<<"Unroll factor:  "<<unroll_factor<<"\n";
    std::cout<<"Index dims:     "<<K<<" (";
    for(int k=0;k<K;k++) std::cout<<"dim"<<idx_dims[k]<<(k<K-1?",":"");
    std::cout<<")\n";
    std::cout<<"Total matches:  "<<total<<"\n";
    std::cout<<"Pruning rate:   "<<pruning<<"%\n";
    std::cout<<"------------------------------------------------\n";
    std::cout<<"File load:      "<<file_load_ms/1000.0      <<"s\n";
    std::cout<<"Index build:    "<<index_build_ms/1000.0    <<"s\n";
    std::cout<<"XRT setup:      "<<xrt_ms/1000.0            <<"s\n";
    std::cout<<"Kernel time:    "<<kernel_ms/1000.0         <<"s\n";
    std::cout<<"Transfer time:  "<<xfer_ms/1000.0           <<"s\n";
    std::cout<<"Host overhead:  "<<host_overhead_ms/1000.0  <<"s\n";
    std::cout<<"After-load:     "<<total_after_load_ms/1000.0<<"s\n";
    std::cout<<"Wall time:      "<<wall_ms/1000.0           <<"s\n";
     if(kernel_ms>0){
        std::cout<<"HW Throughput:  "<<(double)actual_pairs/(kernel_ms/1000.0)/1e9<<" B pairs/sec\n";
        std::cout<<"Eff Throughput: "<<(double)np*qnp/(kernel_ms/1000.0)/1e9<<" B pairs/sec\n";
    }
    std::cout<<"================================================\n";
    return 0;
}

