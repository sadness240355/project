# FPGA 异构 RDMA 工程编码、优化、构建与验证技能包

## 1 技能包定位

本技能包面向需要把 FPGA、PCIe、DMA、相机 ISP、Jetson 软件和 RoCEv2/RDMA 组合成完整系统的工程。它不是某一个工程的操作记录，而是一套可以迁移到图像、雷达、音频、传感器阵列和高速存储项目的工程方法。

本工程的典型数据路径为：

```text
Camera Link
    ↓
FPGA 解串、像素重排、帧同步、ISP
    ├── PCIe DMA 内部桥接器 → Jetson 主机帧池 → 本地预览/推理
    └── FPGA DDR4 帧环 → LRDMA RoCEv2 WRITE → 远端注册内存
```

这类系统的难点不在于某个单独模块能否工作，而在于多个消费者以不同速率访问同一批数据时，系统仍然能够明确回答以下问题：

1. 当前数据属于哪一个帧、哪一个包和哪一个会话。
2. 当前缓冲由哪个模块拥有，何时可以复用。
3. 当前完成代表数据已经写入、已经被远端确认，还是已经可以释放。
4. 发生反压、超时、NAK、DMA 错误或停止请求时，哪一部分必须保留。
5. 一个实验结果到底是源码、仿真、综合、实现还是板级证据。

技能包的所有建议都围绕这五个问题展开。

## 2 适用范围和使用边界

### 2.1 适用范围

- AMD/Xilinx 7 系列、UltraScale、UltraScale+、Zynq、Versal FPGA 项目。
- Vivado、Vitis HLS、C/C++ 驱动和 Linux 用户态程序。
- AXI4、AXI-Stream、PCIe、DMA、DDR4、CMAC、Camera Link、RoCEv2/RDMA。
- 有多个数据消费者、持续数据源、严格帧边界或远端可靠完成要求的系统。
- 需要把源码、仿真、综合、实现、烧录、板级数据和报告交给其他队伍复现的项目。

### 2.2 不适用的情况

- 纯软件离线算法，没有硬件流接口、持续数据源或共享缓冲区。
- 只进行一次性小数据传输，且没有并发、反压或错误恢复要求。
- 只有综合报告而没有最终实现和板上验证的项目。此时技能包只能帮助整理风险，不能产生硬件通过结论。

### 2.3 结论必须区分证据等级

| 证据等级 | 可以证明 | 不能证明 |
| --- | --- | --- |
| 源码检查 | 接口、状态机、寄存器和所有权设计 | 逻辑在真实时钟和真实 IP 中一定正确 |
| 仿真 | 指定输入、时序和异常注入下的行为 | 未建模 IP、CDC、物理链路和板卡行为 |
| HLS C 仿真 | 算法参考模型的输出 | RTL 反压、AXI 时序和资源收敛 |
| 综合 | 逻辑可综合、估算资源 | 最终布线时序、DRC、板卡功能 |
| 实现 | 最终网表时序、DRC、CDC 和资源 | PCIe、DDR、光纤和相机实际运行 |
| 板级测试 | 指定硬件、指定负载和指定时间窗口的实测结果 | 没有覆盖的分辨率、队列、异常和持续时间 |

所有报告、脚本和论文中的结论都应标注证据等级。不要用“验证过”同时描述仿真和板级测试。

## 3 总体工作流

### 3.1 第一步：冻结输入

在任何修改之前冻结以下内容：

- 器件型号、开发板、PCIe 拓扑、DDR 型号、光模块和相机型号。
- Vivado/Vitis/HLS、编译器、Linux 内核和驱动版本。
- RTL、HLS、C/C++、Python、XDC、IP 配置和模型文件。
- 时钟频率、总线宽度、MTU、MPS、MRRS、队列数量和缓冲容量。
- 测试数据、黄金参考、设备地址、rkey、QP、PSN 和运行参数。

为每个文件生成 SHA-256 清单，构建和测试结果使用新的 ID，禁止覆盖已经存在的结果目录。

### 3.2 第二步：建立数据流契约

对每条边建立以下记录：

```text
生产者 → 消费者
数据格式和字节数
单元速率和最大突发
帧/包边界
时钟域
缓存位置和槽数量
写入完成条件
读取完成条件
释放条件
反压方向
错误处理
证据位置
```

视频载荷预算为：

```text
有效载荷 = 宽 × 高 × 每像素字节数 × 帧率
```

还要单独计算 AXI、PCIe、RoCEv2、ACK/NAK、CRC 和软件控制开销。有效载荷不能直接等同于 PCIe 或 100GbE 标称线速。

### 3.3 第三步：先验证最小闭环

推荐顺序：

1. RTL 小单元和状态机仿真。
2. HLS C 仿真和独立参考模型。
3. DMA 单队列、小块和完整缓冲区校验。
4. RDMA 最小 WRITE、ACK、重复 PSN、NAK 和超时。
5. 最终综合、布局布线和时序门禁。
6. 板卡最小数据通路。
7. 连续数据、两路并发、参数切换和安全停止。
8. 真实相机、ISP、Jetson 推理和远端应用。

每一步保存输入、命令、版本、输出、哈希和失败原因。

## 4 RTL 编码技巧

### 4.1 RTL 设计先写“状态和所有权”，再写数据通路

对每个帧槽或包槽定义明确的状态，例如：

```text
EMPTY
  ↓ 采集完成
HOST_WRITING / DDR_WRITING
  ↓ AXI 写响应
PUBLISHED
  ↓ DMA 或 RDMA 读取
READING
  ↓ 对应消费者完成
RELEASE_WAIT
  ↓ 所有消费者都确认
EMPTY
```

如果主机直写和远端 RDMA 共用一批帧，不能只设置一个 `done` 位。至少要分开记录：

```text
host_write_done
host_read_done
rdma_write_done
remote_credit_received
slot_released
```

槽位只有在所有必需消费者都完成后才能复用。某一支路暂时无数据，不能误推进另一支路的释放指针。

### 4.2 ready/valid 必须保持数据稳定

AXI-Stream 或自定义流接口应遵循：

```systemverilog
assign s_axis_tready = !fifo_full && downstream_ready;

always_ff @(posedge clk) begin
  if (!rst_n) begin
    hold_valid <= 1'b0;
  end else if (s_axis_tvalid && !s_axis_tready) begin
    hold_valid <= 1'b1;
    hold_data  <= s_axis_tdata;
    hold_last  <= s_axis_tlast;
  end else if (s_axis_tready) begin
    hold_valid <= 1'b0;
  end
end
```

必须检查：

- `valid=1 && ready=0` 时，数据、`keep`、`last`、帧号和包号保持不变。
- `last` 只在最后一个被接受的 beat 上有效。
- 不让发送端的 `valid` 组合依赖接收端的 `ready`，避免组合环路。
- 反压恢复后不能重复发送同一个 beat，也不能跳过 beat。
- 帧开始和帧结束信号不能仅依赖 FIFO 空非空判断。

### 4.3 FIFO、行缓存和 DDR 帧环

FIFO 深度不要凭经验填写。应依据：

```text
FIFO 深度 ≥ 最大突发长度 + 时钟域相位差引起的积压 + 下游最坏停顿量
```

图像 ISP 通常使用行缓存和窗口缓存。DDR 帧环需要保存：

```text
slot_index
frame_id
publish_seq
ready_timestamp
device_address
bytes
width
height
format
host_done
rdma_done
crc
owner_token
```

`publish_seq` 用于 FPGA 内部顺序，`frame_id` 用于相机源帧号，不能把两者混用。停机时必须先停止输入，再等待已发布槽位排空，再禁用写入器。

### 4.4 AXI-MM 和 PCIe DMA

AXI-MM 事务至少要检查：

- 地址对齐。
- `AWLEN` 和 `AWSIZE` 是否被位宽转换器接受。
- `WSTRB` 是否和实际有效字节一致。
- 写响应 `BVALID/BREADY` 是否被正确等待。
- 读响应是否按事务标记返回原队列。
- RQ 反压时是否禁止提前产生完成或 ACK。

本工程曾遇到一个典型问题：确认缓冲区只有 8 个有效字节，但原事务使用的 `AWSIZE` 不被真实 `512_to_256` 转换器接受，导致 `AWREADY` 一直为低。修正时不能只改仿真模型，而要用真实位宽转换器、PCIe 桥接器和返回通道反压联合仿真；最终修正版使用转换器支持的全宽 beat，并通过 `WSTRB` 保留有效字节。

DMA 控制器中的队列调度建议拆成：

```text
门铃检测
  → 活跃队列选择
  → 描述符读取
  → 地址和长度检查
  → MM 或 ST 引擎启动
  → 事务标记分配
  → 返回数据归队
  → 完成项写回
  → MSI-X 或轮询通知
```

队列号、软件标记和事务标记必须可追溯。完成项写回不能早于最后一个数据 beat 和所有必要的写响应。

### 4.5 RoCEv2/LRDMA 状态机

LRDMA 发送端应至少维护以下状态：

```text
FETCH_WQE
CHECK_WQE
BUILD_HEADER
SEND_PACKET
WAIT_ACK
CHECK_ACK
COMPLETE_WQE
RETRANSMIT
TIMEOUT
QP_ERROR
```

未确认表将以下信息绑定在一起：

```text
QP
WQE 标识
起始 PSN
最后发送 PSN
最后确认 PSN
远端地址
rkey
剩余长度
重传起点
重传次数
```

ACK 覆盖最后一个分片后才写完成项。NAK 必须恢复到协议定义的 PSN 边界，而不是简单重发最后一个内部状态。超时计数器要在发送新包、收到 ACK、收到 NAK 和重传时分别定义清零条件。

### 4.6 报文过滤和字段验证

RoCEv2 接收侧不要使用过窄的“整字节标志必须等于零”判断。应按协议字段掩码检查：

```text
MAC、IPv4 地址、UDP 端口
opcode、QP、PSN
目标虚拟地址、rkey、写入长度
访问权限、地址对齐、区域长度
IPv4 checksum、ICRC
```

对实际抓包报文建立正例和负例：有效 MigReq 标志、错误版本、错误 opcode、错误 rkey、错误长度、重复 PSN、错误 ICRC。仿真应确认：

1. 字段不完整时不产生写请求。
2. PCIe 返回通道未响应时不产生 ACK。
3. 重复 PSN 不产生第二次写入。
4. 错误包不改变有效 PSN。
5. ICRC 错误不推进完成计数。

### 4.7 CDC 和诊断寄存器

多时钟域之间不要直接采样多位状态。根据场景选择：

- 单 bit 电平：两级同步器。
- 单 bit 事件：toggle 同步或握手。
- 多位计数器：Gray 码或请求/响应快照。
- 数据流：异步 FIFO。
- 复位：每个时钟域独立同步释放。

诊断寄存器至少包括：收到首拍数、字段通过数、丢弃数、提交数、写错误数、当前状态、最后一次地址/rkey、最后一次 keep、PSN 接受结果和复位原因。诊断逻辑必须标出资源代价，并与正式性能位流区分。

## 5 HLS 优化技巧

### 5.1 HLS 之前先写独立参考模型

参考模型必须与 RTL/HLS 实现独立，不能复制相同的边界判断错误。参考模型输入使用固定种子或固定测试图，输出保存逐字节黄金数据、CRC32 和关键像素。

ISP 参考模型至少覆盖：

- 最小合法图像。
- 完整目标分辨率。
- 奇数宽高。
- 全黑、全白和棋盘格。
- Bayer 相位变化。
- 第一行、最后一行、第一列、最后一列。
- 连续两帧和配置切换。
- 空闲周期、AXI-Stream 反压和 `TLAST`。

### 5.2 循环优化顺序

不要一开始就堆叠 pragma。推荐顺序：

1. 消除数组越界和未初始化值。
2. 验证单帧逐字节输出。
3. 分析循环依赖和存储端口需求。
4. 加 `PIPELINE II=1`。
5. 将窗口、行缓存和通道数组映射到合适的 RAM。
6. 对确实需要多个端口的数组做 `ARRAY_PARTITION`。
7. 用 `DATAFLOW` 拆分读入、窗口、计算、打包和输出。
8. 重新跑 C 仿真、协同仿真和综合。
9. 对照 II、BRAM、URAM、DSP、LUT 和时序报告。

### 5.3 典型 HLS 结构

```cpp
void isp_top(hls::stream<pixel_t>& in,
             hls::stream<rgb_t>& out,
             int width, int height) {
#pragma HLS INTERFACE axis port=in
#pragma HLS INTERFACE axis port=out
#pragma HLS INTERFACE s_axilite port=width bundle=control
#pragma HLS INTERFACE s_axilite port=height bundle=control
#pragma HLS DATAFLOW
  line_buffer(in, window_stream, width, height);
  demosaic(window_stream, rgb_stream, width, height);
  color_correct(rgb_stream, out, width, height);
}
```

这段结构只有在各阶段之间的 FIFO 深度、边界行为和反压能够被验证时才有意义。`DATAFLOW` 不是自动提高吞吐的开关；任意一个阶段阻塞都会把阻塞传回输入。

### 5.4 行缓存与窗口

行缓存要明确：

- 每一行写入和读出的时机。
- 窗口在图像边缘如何补齐。
- 下一帧开始时是否清空有效位。
- Bayer RG/GB 相位如何随行列变化。
- `TLAST` 是否对应真实行尾还是整帧尾。

如果窗口使用二维数组，先计算每个周期需要的读端口数量，再决定 partition 维度。过度 partition 会把 BRAM 转成大量寄存器，造成面积和布线恶化。

### 5.5 定点化和资源控制

颜色矩阵、白平衡和 Gamma 表优先使用经过误差分析的定点格式。记录：

```text
整数位数
小数位数
输入范围
饱和策略
舍入策略
最大绝对误差
平均误差
```

不要只依据 C 仿真速度选择浮点；必须查看 DSP、BRAM/URAM 和最终时序。查找表适合使用 ROM，重复访问的窗口适合使用 BRAM，短小常量可以使用 LUT。

### 5.6 HLS 验收门

| 阶段 | 必须保存 |
| --- | --- |
| C 仿真 | 输入、输出、CRC、失败样本 |
| 协同仿真 | AXI 时序、反压、`TLAST`、帧边界 |
| 综合 | II、延迟、LUT、FF、BRAM、URAM、DSP |
| IP 导出 | component.xml、驱动、版本、参数 |
| 上板 | 帧计数、CRC、丢帧、配置回读、停止排空 |

C 仿真通过不能写成 HLS IP 上板通过。

## 6 C/C++ 编码技巧

### 6.1 DMA 缓冲区和共享描述符

DMA 缓冲区必须保存以下属性：

```c
struct buffer_desc {
    void    *cpu_va;
    uint64_t dma_addr;
    uint32_t buf_id;
    uint32_t size;
    uint32_t direction;
    uint64_t session;
};
```

共享环形描述符的发布顺序应为：

```c
slot->frame_id = frame_id;
slot->bytes = bytes;
slot->crc = crc;
atomic_store_explicit(&slot->token, token, memory_order_release);
```

读取端必须先 acquire token，再读其他字段：

```c
if (atomic_load_explicit(&slot->token, memory_order_acquire) == token) {
    consume(slot->addr, slot->bytes, slot->frame_id);
}
```

不能先发布 token，再填充地址和长度。不能用 `volatile` 代替 C11 原子内存序。

### 6.2 ARM MMIO 和缓存一致性

ARM 平台写 FPGA BAR 寄存器时使用对齐访问：

```c
static inline void mmio_write32(volatile void *base,
                                size_t offset, uint32_t value) {
    volatile uint32_t *reg =
        (volatile uint32_t *)((uintptr_t)base + offset);
    *reg = value;
    __sync_synchronize();
    (void)*reg;
}
```

不要用可能生成非对齐字节写的通用打包函数访问控制寄存器。写入控制寄存器后，用读回或状态位确认硬件已经接受。DMA 缓冲区释放前确认设备不再访问，并根据平台要求执行 cache flush/invalidate。

### 6.3 C++/C 资源生命周期

资源释放顺序应与所有权相反：

```text
停止源
→ 等待帧边界
→ 等待 DMA 空闲
→ 等待 RDMA ACK/CQ
→ 发送 END
→ 禁用 QP
→ 清理 QP 上下文
→ 关闭队列
→ 解除 MR
→ 释放内存
```

发生安全保持时，不能调用无条件 `kill -9`。应保存 PID、会话 ID、硬件寄存器、SQ/CQ 指针和内存地址，完成受控复位、QP 清理和驱动空闲检查后再释放。

### 6.4 RDMA 确认和帧序号

如果接收端写入的是累计完成数，发送端必须按完成数等待：

```c
uint64_t submitted = 0;
uint64_t completed = 0;

while (submitted - completed >= WINDOW) {
    uint64_t credit = video_credit_wait(credit_buffer,
                                        completed + 1,
                                        submitted);
    if (credit == 0) {
        fail_and_retain_buffers();
    }
    completed++;
}
```

源帧号用于检查相机连续性，累计确认数用于释放发送窗口；两者不能混为一个变量。会话结束消息与帧确认消息必须区分，不能因为收到 TCP END 就认为所有 RDMA 写已完成。

### 6.5 锁和错误处理

共享设备锁必须是唯一正式锁。界面快速停止和重新启动时，可以在同一个锁上进行有限次数重试，但不能创建第二个锁文件。锁错误日志应记录：

```text
打开路径
进程 UID/EUID
文件权限
flock 返回值和 errno
重试次数
最终阶段
```

每个失败都写明阶段，例如：

```text
prepare/camera_read
start/receiver
start/capture
start/sender
wait/first_frame
wait/rdma_credit
stop/drain
```

不要只返回“启动失败”。

### 6.6 编译和静态检查

C/C++ 建议统一使用：

```text
-O2 -Wall -Wextra -Wconversion -Wshadow -pthread
```

根据工程实际启用，不要为了消除告警盲目关闭检查。保存完整编译命令、编译器版本、目标架构、头文件清单、静态库顺序、运行库版本和二进制 SHA-256。驱动模块额外记录内核版本、vermagic、srcversion 和设备节点权限。

## 7 Python 和控制软件技巧

Python 主要负责配置、状态汇总、测试编排和 UI，不应把大块像素复制和严格时序放在 Python 主循环。建议：

- 用 UNIX socket 或明确认证的 HTTP API 控制 root 硬件服务。
- 所有长操作设置阶段超时，并把状态写入 JSON/JSONL 日志。
- 用会话 ID 区分不同采集任务，旧会话不能释放新会话的缓冲。
- UI 轮询状态时不能假设网卡名称固定；优先使用配置值，再根据目标 IP 自动发现实际网卡。
- UI 的停止按钮必须等待后端 drain 结果，不能只关闭窗口。
- 任何后台线程退出时保存异常和资源状态，不能静默吞掉异常。

## 8 构建工程技巧

### 8.1 干净 Vivado 工程

源码目录只放工程输入：

```text
vivado_project/
├── *.xpr
├── *.srcs/
├── *.xdc
└── IP 配置和 HDL 源码
```

以下内容必须放到构建输出目录或被清理：

```text
.Xil/
.cache/
.gen/
.runs/
.hw/
.sim/
ip_user_files/
*.dcp
*.bit
*.ltx
*.jou
*.log
```

工程包中的 Vivado 副本不能依赖另一台电脑的绝对路径。构建脚本使用工程根目录计算相对路径，并在开始时打印器件、顶层、工具版本和源文件清单。

### 8.2 Vivado 构建门禁

构建流程：

```text
打开工程
→ 校验 IP 和源文件
→ 生成 IP
→ 综合
→ 综合资源和时序报告
→ 布局布线
→ 实现资源、时序、DRC、CDC 报告
→ write_bitstream
→ 计算 bitstream 哈希
```

最终门禁至少检查：

- setup、hold 和 pulse width 的最差裕量。
- TNS、THS、TPWS 和失败端点数量。
- 未约束路径数量。
- 已路由网络和未路由网络。
- DRC Error/Critical Warning。
- CDC Critical 的数量、规则和是否已解释。
- 资源使用量和器件总容量。

报告出现 `-0.000` 时不能凭显示值判断通过，应使用 Tcl 查询原始负裕量。

### 8.3 HLS 和软件构建

HLS 构建必须保存：

- C 仿真和失败输入。
- C/RTL 协同仿真结果。
- HLS 综合报告。
- 生成 IP 的版本和参数。
- component.xml、驱动和打包脚本。

软件构建必须保存：

- 编译器版本和目标架构。
- 完整编译命令。
- 静态库和链接顺序。
- 内核版本和驱动 vermagic。
- 运行库、模型和二进制哈希。

### 8.4 唯一最终版本原则

正式工程包中只放一个最终源码树、一个最终构建入口和一组与其对应的实验数据。诊断版、旧候选、旧位流和失败迭代应放在开发归档，不进入提交包。失败结论只保留在最终实验索引的文字说明中，不携带无关的迭代代码。

## 9 连续流和板级验证

### 9.1 分层验证

| 层级 | 负载 | 放行条件 |
| --- | --- | --- |
| 物理层 | PCIe、DDR、光口、Camera Link | 链路稳定，无新增物理错误 |
| 最小事务 | 4/8 字节、单包、单 WQE | 地址、长度、CRC、完成状态正确 |
| 队列层 | 多队列、回绕、批量 WQE | 每队列完成和顺序正确 |
| 连续流 | 真实帧或固定彩条 | 长时间无未解释缺口、CRC 错误和积压 |
| 并发流 | 主机支路、DDR/RDMA 支路、框通道 | 所有权不冲突，停止能排空 |
| 异常恢复 | NAK、超时、断链、停止重启 | 不提前释放内存，恢复后能重新接收 |

### 9.2 测试记录

每轮记录：

```text
源码/位流哈希
器件、链路、MPS、MRRS、MTU
发送/接收时间窗口
帧数或 WQE 数
源序号缺失/重复
CRC/黄金参考失败数
队列高水位
有效吞吐
延迟分位数
发送端完成状态
接收端应用状态
停机排空状态
```

在途帧不直接计为丢帧。必须先执行定义好的 stop、drain 和 completion 检查。

### 9.3 相机和 ISP

相机实验要分别记录：

```text
相机读回宽高、像素格式、帧率、曝光
Camera Link 输入诊断
ISP 配置 mask 和 generation
主机直写帧数
DDR 发布帧数
LRDMA 提交帧数
远端接收帧数
CRC 错误
停机尾帧和累计 drop
```

相机配置的 140 fps 不是实际传输帧率。DMA 与 RDMA 分开运行的彩条带宽不能相加成并发带宽。在线 ISP 的寄存器提交通过也不能代替颜色质量评价。

## 10 在线重配置和安全停止

在线重配置采用事务化流程：

```text
检查新参数
→ 停止新提交
→ 等当前帧边界
→ 等 DMA 写响应
→ 等 RDMA 完成/确认
→ 停止消费者
→ 更新寄存器和软件配置
→ 先启动消费者
→ 再启动数据源
```

不同消费者按依赖图逆序停止、正序启动。每个会话使用唯一 ID；旧会话不能读取或释放新会话的共享文件。

发送端故障时：

1. 停止相机输入。
2. 关闭输入门控并确认硬件静默。
3. 等 AXI 写入器和 DMA 请求器空闲。
4. 保留 DMA 内存和 RDMA 注册区。
5. 清理 QP 上下文并复位必要互连。
6. 验证驱动空闲。
7. 只有在所有门禁通过后释放进程和内存。

## 11 工程包交付清单

工程包应包含：

- 最终 RTL、HLS、驱动、运行库和应用源码。
- 仿真脚本、输入、输出、波形和黄金参考。
- 可重建 Vivado/HLS/C/C++ 构建脚本。
- 综合、实现、资源、时序、DRC、CDC 报告。
- 烧录、驱动、权限、运行和恢复脚本。
- 实测数据、参考结果、设备条件和文件哈希。
- 中文 README、实验状态和复现步骤。
- 技能包，写明适用范围、使用方法、已验证效果和失效条件。

敏感密钥、临时绝对路径、无关迭代代码、重复位流和生成缓存不得进入提交包。预编译二进制必须说明系统、内核、ABI、工具版本和构建来源。

## 12 本工程的已验证经验和边界

本工程验证了 PCIe DMA 矩阵、LRDMA 矩阵、静态彩条 DMA/RDMA、相机双路传输、在线 ISP 切换和修正版位流的板级流程。实际调试中确认了 RoCEv2 标志位过滤、AXI 写宽度、发送端 RDMA 确认通道、QP 退休和 DMA 内存释放等关键问题。

XDMA 对比实验按要求排除。YOLO11n、ByteTrack 和检测框专项实验尚未完成；模型存在不等于推理通过，单元仿真通过不等于完整重传上板通过。所有工程结论必须服从实际证据范围。

---

# 系统构建与软硬件集成技能

## 1 适用范围

本技能适用于由 FPGA 逻辑、板载 DDR、PCIe DMA、网络协议栈、Linux 驱动、C/C++ 运行库、Python 控制服务和应用 UI 组成的完整系统。它解决的是“各模块分别能编译，但整机无法启动或无法复现”的问题。

系统构建的最终目标不是生成一个 bitstream，而是生成一套能够在指定器件、指定驱动、指定操作系统和指定外部设备上完成启动、运行、停止和再次启动的闭环。

## 2 系统构建的五层输入

构建前建立五层输入表：

| 层级 | 输入 | 必须固定的内容 |
| --- | --- | --- |
| FPGA 工程 | RTL、HLS IP、XCI、XDC、顶层 | 器件、顶层、时钟、约束、IP 版本 |
| 固件和位流 | bitstream、BMM/XSA 或启动文件 | 生成工具、位流来源、哈希、烧录方式 |
| 驱动 | 内核模块、设备树、设备节点 | 内核版本、vermagic、绑定 PCIe 设备、权限 |
| 运行库 | DMA、RDMA、控制 API、C/C++ 静态库 | 编译器、目标架构、头文件、链接顺序 |
| 应用 | 采集、发送、接收、UI、模型 | Python 版本、模型、网卡名、IP、端口、运行目录 |

任何一层输入变化，都要重新生成对应哈希和构建记录。不能只替换 bitstream 而继续使用无法确认来源的驱动或应用二进制。

## 3 推荐的系统目录

```text
project/
├── src/
│   ├── rtl/
│   ├── hls/
│   ├── driver/
│   ├── runtime/
│   └── application/
├── sim/
├── build/
│   ├── scripts/
│   ├── reports/
│   └── final/
├── board/
│   ├── program/
│   ├── driver/
│   ├── run/
│   └── results/
├── data/
├── skill/
└── report/
```

`src` 只保存输入，`build` 保存生成结果，`board` 保存设备相关操作和实测记录，`data` 保存参考值和统计结果。不要把 `.Xil`、`.runs`、`.dcp`、`.bit`、临时日志和大体积缓存放入源码目录。

## 4 系统构建顺序

### 4.1 先做静态输入审查

在启动 Vivado 前检查：

1. 工程顶层和器件型号是否正确。
2. 所有 XCI、XDC、HLS IP 和 HDL 文件是否存在。
3. 工程中的绝对路径是否指向当前工作区。
4. 是否混入上一轮 `.Xil`、`.runs`、`.gen` 或旧 checkpoint。
5. 约束中的时钟名称、端口名称和层级路径是否仍然存在。
6. 编译脚本是否会覆盖已经存在的结果目录。

建议脚本在开始时输出：

```text
工具版本
器件型号
顶层模块
约束集合
源码文件数量
IP 数量
输入清单哈希
输出目录
```

### 4.2 FPGA 逻辑构建

推荐顺序：

```text
读取工程
→ 校验 IP
→ 生成 HLS/IP 输出
→ 综合
→ 综合资源和时序报告
→ 布局布线
→ 实现资源、时序、DRC、CDC 报告
→ 生成 bitstream
→ 计算 bitstream 哈希
```

综合阶段失败时先检查 HDL、IP 和约束；实现阶段失败时再检查布局、扇出、拥塞、CDC 和时序。不要通过删除约束、降低时钟或关闭检查来“修复”构建。

### 4.3 驱动构建

驱动构建必须使用目标 Jetson 的真实内核头文件和配置：

```text
读取 uname -r
确认内核头文件
确认 vermagic
编译 .ko
记录 srcversion
安装到临时目录
加载模块
检查 /dev/edev0 和 /dev/edev0_q*
检查 PCIe BAR、MPS、MRRS、链路宽度和速率
```

驱动模块加载成功不等于 FPGA 设备可用。还要检查：

- PCIe 设备是否枚举。
- 驱动是否绑定到正确的 BDF。
- BAR0/BAR2 是否可以映射。
- 设备队列是否创建。
- 用户权限是否允许打开设备。
- 设备复位后请求器是否真正空闲。

### 4.4 运行库和应用构建

C/C++ 运行库必须先于发送端、接收端和采集程序构建。推荐顺序：

```text
运行库头文件
→ DMA 内存池
→ PCIe 控制 API
→ RDMA API
→ 采集程序
→ 发送程序
→ 接收程序
→ UI 和控制服务
```

每个二进制保存：

```text
编译命令
编译器版本
目标架构
链接库和顺序
依赖的头文件哈希
二进制 SHA-256
```

Python 应用构建还要固定解释器版本、第三方包、模型文件、TensorRT/CUDA 版本和 `PYTHONPATH`。启动脚本不应依赖用户当前目录。

## 5 系统启动顺序

### 5.1 FPGA 和 PCIe

1. 确认 FPGA 已下载最终位流。
2. 记录位流哈希和启动时间。
3. 确认 PCIe 设备枚举。
4. 确认链路速率、宽度、MPS 和 MRRS。
5. 加载驱动并检查设备节点。
6. 运行只读 BAR、DMA 映射和设备空闲检查。

### 5.2 网络和 RDMA

1. 检查 Jetson 和远端网卡名称，不要假定接口名永远不变。
2. 检查管理网 IP 和数据网 IP。
3. 检查 MTU、GID、端口状态和 RoCEv2 类型。
4. 先启动接收端控制服务，再启动发送端。
5. 交换 QP、地址、rkey、PSN 和缓冲区元数据。
6. 发送最小测试包，再进入完整帧或大块传输。

控制网、数据网、RDMA 确认和 UI 状态接口必须分别记录。UI 显示“设备在线”不等于 RDMA 数据通道已通过。

### 5.3 相机、ISP 和应用

1. 相机串口读回型号、宽度、高度、像素格式、帧率和触发模式。
2. 写入 ISP 参数并检查 generation、busy 和 error。
3. 预创建主机帧池、DDR 帧环和检测共享内存。
4. 让接收端和发送端进入就绪状态。
5. 等 DDR 描述表发布后再启动相机采集。
6. 取得第一帧后再允许 UI 进入 streaming 状态。
7. YOLO/ByteTrack 作为独立消费者启动，不得阻塞视频主路径。

## 6 系统启动失败的定位顺序

### 6.1 采集无法启动

依次检查：

```text
相机串口读回
→ 像素格式和分辨率
→ FPGA ISP generation
→ /dev/edev0 和队列设备
→ 主机帧池
→ DDR ring 描述表
→ 发送端设备锁
→ 接收端监听状态
→ 第一帧超时
```

如果日志显示“发送端未 ready”，先读取发送端独立日志。不要直接把相机停源失败归因于 Camera Link。发送端可能在设备锁、网卡、QP、MR 或 RDMA 元数据阶段提前退出。

### 6.2 设备锁失败

共享锁必须满足：

```text
路径一致
所有者和组正确
root 控制服务可打开
普通用户测试可打开
没有第二个锁文件
```

在 `/run/lock` 的粘滞目录中，如果 root 服务通过 `O_CREAT` 打开普通用户创建的文件，可能受到 `protected_regular` 策略限制。解决方法是把同一个共享锁设置为 root 所有、运行用户所属组可访问，并保持统一路径。不要通过新建另一个锁文件绕过设备所有权。

### 6.3 UI 显示错误

UI 只负责状态展示和操作请求，故障定位要回到后端：

- `status` 请求是否能返回 JSON。
- 控制服务 UNIX socket 是否存在。
- 网卡状态是否使用当前实际接口。
- UI 进程是否持有旧会话或旧共享内存。
- 后端操作是否返回阶段名和错误日志。
- 停止按钮是否等待 drain，而不是只关闭窗口。

网卡名称应支持环境变量配置，并在配置名称不存在时根据目标 IP 自动发现实际接口。

## 7 构建和部署门禁

在部署最终版本前生成以下门禁表：

| 检查项 | 通过条件 |
| --- | --- |
| 源码 | 所有源文件存在，清单哈希一致 |
| HLS/IP | C 仿真、协同仿真、IP 导出通过 |
| 综合 | 无综合错误，资源报告已归档 |
| 实现 | setup/hold/pulse width 满足要求，未约束路径已解释 |
| DRC/CDC | Error 为 0，Critical 项有分类和处理结论 |
| 位流 | 生成成功，哈希已记录 |
| PCIe | 设备枚举，链路宽度/速率/MPS/MRRS 正确 |
| 驱动 | 模块版本匹配，设备节点可访问 |
| 网络 | GID、MTU、端口和 IP 关联正确 |
| 应用 | 发送、接收、采集和 UI 退出码正确 |
| 停止 | DMA、QP、MR、队列和共享内存都排空 |
| 交付 | 清单、脚本、报告、数据和中文说明齐全 |

任何一项没有证据，只能写“未验证”，不能写“系统完成”。

## 8 系统构建脚本的写法

一个可复现脚本应具备：

```bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd -P)
ID=${1:?需要新的结果 ID}
OUT="$ROOT/build/$ID"
[ ! -e "$OUT" ] || { echo "拒绝覆盖已有结果" >&2; exit 2; }
mkdir -p "$OUT"
```

脚本必须：

- 不使用 `$HOME` 作为工程根目录。
- 不写死作者电脑的绝对路径。
- 不覆盖已有结果。
- 失败时保留日志和中间状态。
- 在最后输出清晰的 `PASS` 或 `FAIL`。
- 把使用的脚本、输入哈希和工具版本复制到结果目录。

长时间板级实验还要保存开始前和结束后的设备状态，结束后运行统一的空闲检查。

## 9 可复现工程包

正式工程包只包含最终版本：

```text
src/        最终源码和工程输入
sim/        最终仿真和结果
build/      最终构建脚本和报告
board/      最终上板脚本和结果
data/       最终参考数据和实验索引
skill/      可复用技能
report/     设计报告和中文说明
```

不应包含：

- 旧候选、旧诊断工程、失败迭代源代码。
- 无法对应最终源码的 bitstream 或 checkpoint。
- `.Xil`、`.runs`、`.cache`、`.gen` 等缓存。
- 密钥、密码、临时绝对路径和未授权数据。
- 只存在于开发者电脑中的依赖。

交付前执行：

```bash
sha256sum -c package_manifest.sha256
find src/vivado_project -name '*.bit' -o -name '*.dcp' -o -name '.Xil'
gzip -t package.tar.gz
```

## 10 本工程的构建经验

本工程中曾出现以下系统级问题：

1. 实际宽度转换器不接受确认写入使用的 `AWSIZE`，导致 RTL 单元看似工作但板上写请求停滞。
2. RoCEv2 实际报文携带合法 MigReq 标志，而接收过滤器按整字节等于零拒绝报文。
3. 发送端错误地等待 TCP 帧确认，而接收端通过 RDMA 写回累计确认。
4. 源结束时发送端提前判断为故障，未先排空确认和退休 QP。
5. UI 和发送脚本固定使用已经不存在的网卡名。
6. root 控制服务和普通用户共用 `/run/lock` 时受到文件所有权和 `protected_regular` 策略影响。

这些问题说明系统构建不能只关注 RTL 或应用代码。协议字段、实际 IP 行为、启动顺序、权限、设备锁、服务身份和停止顺序同样属于系统设计的一部分。

## 11 失效条件

以下情况不能使用本技能包得出“系统可复现”结论：

- 只提供 bitstream，没有源码、IP 配置和构建脚本。
- 只提供综合报告，没有最终实现和板上数据。
- 只检查发送端完成计数，没有远端逐字节/逐帧校验。
- 只做单包测试，没有连续流、队列回绕和停机排空。
- 通过修改锁文件、杀死进程或跳过复位掩盖资源未释放。
- 使用旧源码、旧报告和新位流混合解释。
- 未记录工具版本、器件、约束、驱动和运行库版本。
