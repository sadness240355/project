# 基于 RDMA 的异构边缘计算终端工程包

本工程面向 AMD FPGA 自主选题高级组，包含 PCIe DMA、相机采集与 ISP、RoCEv2 LRDMA、驱动、主机应用和实验记录。保持 `src/`、`sim/`、`build/`、`board/`、`data/`、`skill/`、`report/` 的原有布局。

## 脚本复现

将整个 `project` 目录复制到目标主机，进入该目录后执行：

```bash
bash build/reproduce.sh check
bash build/reproduce.sh sim demo_sim
bash build/reproduce.sh hls demo_hls
bash build/reproduce.sh software demo_sw
bash build/reproduce.sh vivado demo_fpga
```

每次使用新的 ID，脚本拒绝覆盖既有输出。也可从任意工作目录运行 `bash "/工程包路径/build/reproduce.sh" check`。入口根据自身位置定位文件；无需原开发机目录、原 Vivado 工程或网络下载源码。含空格和中文的目录已验证。

`vivado` 模式仅创建干净工程、载入源码和 IP、更新编译顺序并输出 IP 状态，不启动综合、布局布线或位流生成。直接运行 `build/rebuild_clean.tcl` 的默认阶段同样是 `create`。本次验收按此范围执行。

## 环境依赖

| 阶段 | 依赖 | 输出位置 |
| --- | --- | --- |
| `check` | Linux、Python 3.8+、Bash | 标准输出，检查文件完整性和脚本语法 |
| `sim` | Icarus Verilog、`vvp`、GNU coreutils/timeout | `sim/results/<ID>_*` |
| `hls` | Vitis HLS 2022.2、系统 C++ 编译器 | `sim/results/<ID>`，78 个 C 模型测试 |
| `software` | GCC、binutils、开发库 `libibverbs`、zlib | `board/runtime_build/`、`board/rdma_build/`、`board/application_build/` |
| `vivado` | Vivado 2022.2，安装 Kintex UltraScale+ 器件支持 | `build/<ID>/vivado/lrdma_repro.xpr`、`ip_status.rpt` |
| `ui-sender` / `ui-receiver` | 软件依赖，加 JPEG/EGL/GLESv2/X11/Xext 开发库 | `src/software/ui/` 下的本机二进制 |
| `driver` | make、与目标内核匹配的内核构建树、kmod | `board/driver_build/<ID>` |

例如 Debian/Ubuntu 的编译依赖为 `build-essential iverilog libibverbs-dev zlib1g-dev libjpeg-dev libegl1-mesa-dev libgles2-mesa-dev libx11-dev libxext-dev kmod`。运行图形工作站另需 Python Pillow、Tk；检测功能另需匹配 Jetson 的 PyTorch、Ultralytics、TensorRT/CUDA 和 `yolo11n.pt`。检测模型未包含在本工程中。

先加载本机 Xilinx 工具的 `settings64.sh`，使 `vivado`、`vitis_hls` 位于 PATH。也可通过 `VIVADO`、`VITIS_HLS` 指定可执行文件。实际生成 FPGA 位流时需相应器件和所用 IP 的有效许可；本次未执行该阶段。

“可迁移”指工程路径和构建输入不绑定开发机。FPGA 工程目标仍为 `xcku15p-ffve1517-2-i`；板级实验仍需匹配 FPGA/DDR4/PCIe/Camera Link 硬件、Jetson 和支持 RoCEv2 的远端网卡。普通计算机可做离线检查、仿真、工程创建及本机软件编译，不能替代实验硬件。

## 目录说明

- `src/rtl/vivado_project/`：原 XPR 清单、补齐的 RTL、IP 配置、封装 HLS IP 和约束；原 XPR 保留不改，正式复现入口创建新的干净工程，忽略已删除的增量 DCP 和原运行状态。
- `src/rtl/baselines/`：基线 IP 配置及信用通道对照仿真输入。
- `src/hls/`：ISP 的 C++ 源码。
- `src/host_driver/`：内核驱动和接口头文件。
- `src/software/runtime_lib/`、`src/software/ui/`：运行库、图形界面及应用源码；`ui/lib` 是包内相对链接。
- `sim/`：RTL 和 ISP C 模型仿真入口、测试台及输入。
- `build/`：统一入口、文件检查、工程创建、软件构建和复现说明。
- `board/`：可重新编译的测试程序、板级执行入口；运行前阅读其 README。
- `data/`：参考数据生成器、实验索引和汇总脚本；`data/results/` 保留历史结果。
- `skill/`、`report/`：方法说明、论文和海报。

详细复现范围、修复说明和实测记录见 [build/REPRODUCIBILITY.md](build/REPRODUCIBILITY.md)。历史 `PAPER_RECONSTRUCTED_RESULT` 是按论文重建的数据，不代表本次重新上板测量。
