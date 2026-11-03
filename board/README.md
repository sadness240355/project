# 板级构建与执行

先在工程根目录执行 `bash build/reproduce.sh software <新ID>`。它从提交源码构建运行库、RDMA 发送/接收程序、彩条发送程序、相机配置/采集工具及视频发送/接收程序，末尾输出本次构建的名称。所有输出使用本机架构，不复用归档中的可执行文件。复制到另一端后应在另一端重新编译。

## 选择构建结果与机器配置

运行前将构建阶段打印的变量导出，例如：

```bash
export RUNTIME_BUILD=demo_sw_runtime
export SENDER_BUILD=demo_sw_sender RECEIVER_BUILD=demo_sw_receiver
export CAPTURE_BUILD=demo_sw_capture CONFIG_BUILD=demo_sw_config
export VIDEO_SENDER_BUILD=demo_sw_video_sender VIDEO_RECEIVER_BUILD=demo_sw_video_receiver
```

`env.sh` 列出默认设备和网络参数，支持提前 `export` 覆盖：`FPGA_RESOURCE`、`DMA_DEVICE`、`FPGA_IP`、`PEER_IP`、`PEER_CONTROL_IP`、`PEER_MAC`、`FPGA_NETDEV`、`PEER_NETDEV`、`RDMA_DEVICE`、`RDMA_GID_INDEX`、`CONTROL_URL`。C 测试程序中的设备/IP 字面量由 `configure_sources.py` 仅在临时构建副本中替换，原设计源码不变；因此更换机器配置后须重新构建。`host_config.json` 记录使用的映射。设备队列号和 FPGA 寄存器协议仍由设计规定。

`CC`、`AR` 可选择本机构建工具或交叉编译工具。交叉编译时需自行提供目标 sysroot 和相应开发库；默认使用当前主机的 `cc`、`ar`。

## 驱动

```bash
bash build/reproduce.sh driver demo_driver
# 使用指定的目标内核构建树：
KERNEL_BUILD=/目标内核构建树 bash build/reproduce.sh driver demo_driver_target
```

默认使用 `/lib/modules/$(uname -r)/build`。脚本只编译模块，不加载驱动；目标设备上必须使用与运行内核匹配的模块。本次在已安装的 6.17.0-14-generic 头文件上完成编译；当前运行内核的构建树未安装。

## 实验入口

| 实验 | 入口 |
| --- | --- |
| PCIe DMA 矩阵 | `run_dma_matrix.sh <预留DDR基址> <新ID>` |
| RDMA 两端矩阵 | `run_rdma_receiver_matrix.sh <campaign>`；`run_rdma_sender_matrix.sh <campaign> <位流SHA256>` |
| 相机配置 | `run_camera_geometry_fresh.sh <新ID>` |
| 相机与 RDMA 双路径 | 先在远端执行 `run_camera_video_receiver_checked.sh <新ID>`，再在 FPGA 主机执行 `run_camera_dual_final.sh <新ID> <秒数>` |
| 彩条 DMA | `run_colorbar_dma.sh <新ID> <秒数>` |
| 彩条 RDMA | `run_colorbar_rdma_receiver_rounds.sh`、`run_colorbar_rdma_sender_rounds.sh` |

彩条两轮入口保留原实验 ID，已存在结果时需更换 ID 后执行。初次板测前先运行 `python3 data/generate_reference.py`，生成默认 `data/reference_v1/`；已有参考目录时脚本拒绝覆盖。`build/reproduce.sh reference <ID>` 可另建独立参考目录。

板级入口会读设备、发送流量或操作相机，不属于默认离线复现。运行前需要已编程位流、正确驱动和设备权限、网络配置、空闲且可访问的两端控制服务，以及无冲突的 DDR 内存区域。已有空闲检查会拒绝活动会话；请按实际硬件配置启动工作站。新测量写入 `board/results/` 和 `board/campaigns/`，历史论文汇总仍位于 `data/results/`。
