# FPGA-Accelerated Exact k-Nearest Neighbor Search

This repository contains Vitis HLS and Vivado implementations for exact similarity search and distance calculations on multi-dimensional spatial datasets. The project utilizes hardware-software co-design to eliminate data movement bottlenecks by processing data directly on the hardware layer.

The project began with a simple batching baseline to handle point transmission and distance calculation, and has since evolved to utilize dynamic grid indexing, custom dataflows, and parallel IP core execution.

## ⚙️ Architecture & Hardware Parameters

*   **Algorithm:** Grid Indexing (Indexed on 3 dimensions)
*   **Maximum Dimensions:** 8
*   **HLS Unroll Factor:** 4 (Applied to both data points and dimensional loops)
*   **BRAM Storage (Targets/Data Points):** 5,000 (Scalable up to 7,168)
*   **BRAM Storage (Query/Anchor Points):** 2,048

---

## 📁 Codebase Structure

The codebase is split into two primary methodologies: **Self-Join** implementations (comparing a dataset against itself) and **Dual Dataset** implementations (query vs. target).

### 1. Self-Join Implementations

| Project / Filename | Version | Core Features |
| :--- | :--- | :--- |
| `Ndim_ddr_anc_batch_app` | Grid Static Baseline | Static 3D grid construction strictly using the first three (X, Y, Z) dimensions. |
| `Anchor_batch_opt_try_application` | Grid Dynamic Baseline | Indexing on the 3 highest-variance dimensions. Unindexed 4th dimension is sorted. |
| `Anchor_batch_opt_64float_application`| Grid Dynamic V2 | Retains dynamic grid and 4th-dimension sorting. Explicitly enables integer division and multiplication optimizations in Vivado. |

### 2. Query vs. Target Implementations

These versions process two distinct datasets to support general similarity searches based on different application requirements.

| Project / Filename | Version | Core Features |
| :--- | :--- | :--- |
| `Twodatasetproj_app` | Baseline | Computes distances between two completely separate datasets (Query vs. Target). |
| `dual_ip_core_app` | Dual IP Cores | Instantiates two identical copies of the kernel to process data concurrently. Features an unroll factor of 2, 6 max dimensions, two DMAs, and dual IPs in Vivado with parallel execution calls in Vitis IDE. |