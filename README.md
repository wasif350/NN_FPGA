# FPGA-Accelerated Exact Range Similarity Search (20D)

Exact range-based similarity search on the Xilinx Alveo U280 FPGA.
The host builds a dynamic K-dimensional grid index, prunes candidates, and send candidates to the FPGA kernel for exact distance computation.

---

## Files

| File | Description |
| :--- | :--- |
| `baseline_kernel_20d.cpp` | Optimized kernel — UNROLL_FACTOR=15, URAM dataset caching, BRAM query caching|
| `baseline_kernel_20d_basic_1.cpp` | Basic sequential kernel — no caching, no unrolling; reference implementation |
| `index_fpga_dynamic.cpp` | Host driver — K-D grid index + OpenMP pruning + XRT dispatch to FPGA |

---

## Dataset Binary Format

Same format for both dataset and query files:

[4 bytes] int32 — num_dims
[4 bytes] int32 — num_points
[num_points × num_dims × 4 bytes] float32 — point data, row-major

The SUSY 20D dataset was used for testing this specific kernel.
Download the SUSY dataset from: https://archive.ics.uci.edu/dataset/279/susy
Convert to binary format using `convert_susy.py` before running.
---

## Prerequisites

- Xilinx XRT (Xilinx Runtime) installed — provides the `xrt/xrt_device.h`, `xrt/xrt_kernel.h`, and `xrt/xrt_bo.h` headers and runtime libraries required by the host driver.
- Install XRT: https://github.com/Xilinx/XRT

## Compile & Link (Xilinx U280)

**1. Compile kernel**
```bash
v++ -t hw -c \
    --platform xilinx_u280_gen3x16_xdma_1_202211_1 \
    -k similarity_search \
    -o similarity_search.xo \
    baseline_kernel_20d.cpp
```

**2. Create link.cfg** (1 CU example)
```ini
[connectivity]
nk=similarity_search:1:similarity_search_1
```

**3. Link to xclbin** (~8–12 hours)
```bash
v++ -t hw -l \
    --platform xilinx_u280_gen3x16_xdma_1_202211_1 \
    --config link.cfg \
    -o similarity_search.xclbin \
    similarity_search.xo
```

**4. Compile host**
```bash
g++ -O3 -std=c++17 -fopenmp \
    -I/opt/xilinx/xrt/include \
    -L/opt/xilinx/xrt/lib \
    -o index_dynmaic_debug \
    index_fpga_dynamic.cpp \
    -lxrt_coreutil -lxrt_core
```

---

## Run

```bash
./index_dynmaic_debug <xclbin> <dataset.bin> <queries.bin> <radius> <num_cus> <point_pad> <unroll_factor> [num_index_dims]
```

Example:
```bash
./index_dynmaic_debug similarity_search.xclbin higgs.bin higgs_queries.bin 0.037 1 20 15 3
```

- `point_pad` must match kernel POINT_PAD → use `20`
- `unroll_factor` must match kernel UNROLL_FACTOR → use `15` (optimized) or `1` (basic)

---

## Porting to a Different FPGA

Change the `--platform` string in both `v++` commands to your board's platform string.
Find it with: `platforminfo --list`
Full re-synthesis required for each new platform. Resource configurations (BRAM, URAM, DSP usage) depend on the available resources of the target FPGA.