# UI 与应用软件

- `video.sh`：统一启动入口，支持 `station`、`desktop`、`browser`、`capture`、`sender`、`receiver`、`yolo` 和 `control`。
- `video_desktop.py`：桌面 UI；当前包采用已修正安全退出路径的版本。
- `video_control.py`、`video_console.py`、`video_web.py`：控制服务、状态接口和网页界面。
- `console.html`、`console.css`、`console.js`、`dual.html`：网页控制台资源。
- `src/`、`capture_reference/`：UI 所调用的相机、显示和视频传输应用源码。
- `lib/`：指向 `../runtime_lib` 的相对链接，避免重复打包运行库。

运行时需要在目标 Jetson 上提供相机设备、PCIe 驱动、RDMA 网卡和编译生成的发送/接收程序。本次已完成离线编译，未启动 UI 或执行板级采集。

## 可迁移构建入口

在工程根目录执行 `bash build/reproduce.sh ui-sender` 或 `bash build/reproduce.sh ui-receiver`；也可在本目录执行 `bash video.sh build-sender` / `bash video.sh build-receiver`。Makefile 的 sender/receiver/capture/pattern/check 目标委托给随包脚本，不再依赖缺失的旧 tests 目录。`check` 表示源码和语法检查。

Web 双端控制需要两端相同的共享密钥：通过环境变量 `STATION_KEY` 提供，或各自在本目录放置内容相同的 `station.key`。配置 `STATION_LOCAL_IP`、`STATION_PEER_IP` 指定控制端地址；`video_web.py --bind <本机地址>` 可指定监听地址。检测为可选功能，模型放在本目录 `models/yolo11n.pt` 后执行 `bash video.sh yolo-deploy`，TensorRT 引擎须在实际 Jetson 上生成。

相机脚本 `capture_reference/bayerrg8_app.sh` 负责相机尺寸、帧率、曝光和测试图配置，再将采集/ISP 参数传给现有 viewer。此过程操作硬件，本次仅检查和编译。

跨机器 SSH 入口改为显式配置 `JETSON_SSH`、`RECEIVER_SSH`，以及 `JETSON_PROJECT`、`RECEIVER_PROJECT`（远端工程根目录，可相对远端登录目录）。运行前需设置免交互环境传递或在远端终端导出机器参数；SSH 不会自动传递本地全部环境。
