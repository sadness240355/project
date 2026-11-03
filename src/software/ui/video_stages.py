"""Stage-local counters. Never equate frame sequence gaps with network packet loss."""
def stage_counts(capture,send,recv,preview=None,save=None):
 diag=capture.get('diag',{})
 crc_active=recv.get('crc_mode',recv.get('config',{}).get('crc_mode','full'))=='full'
 direct=send.get('source')=='fpga_ddr' or send.get('config',{}).get('transport')=='ddr'
 session=send.get('config',{}).get('session')
 same=bool(session and session==recv.get('config',{}).get('session'))
 final=same and send.get('passed') and recv.get('passed') and not send.get('running') and not recv.get('running')
 undelivered=max(0,send.get('submitted',0)-recv.get('received',0)) if final else None
 def row(key,stage,where,value,unit,kind,note):
  return dict(key=key,stage=stage,where=where,value=value,unit=unit,kind=kind,note=note,brief=STAGE_HELP[key]['brief'])
 return [
 row('input_bad','输入异常帧','Camera Link → FPGA',diag.get('bad'),'帧','硬件诊断','无有效像素或帧同步异常；可能同时计入 FPGA 丢帧，不能重复相加。'),
 row('fpga_drop','FPGA 丢帧','FPGA → DDR 发布',capture.get('drops'),'帧','实际丢弃','含无效帧、存储路径及 AXI 写入异常；最后原因码 '+str(diag.get('reason','未读取'))+'。硬件累计计数，以硬件复位为界。'),
 row('ddr_skip','DDR 旧帧跳过','旧版 DDR→PCIe DMA（当前未启用）',capture.get('skip_ddr'),'帧','主动跳帧',('当前普通采集不走 DDR→主机 DMA；此计数应为 0。DDR 槽由 RDMA 完成释放；全帧 CRC 还等待主机直写参考。' if direct else 'latest-frame 策略跳过尚未提交 DMA 的旧完整 DDR 槽，优先搬运新帧。')),
 row('host_skip','主机旧帧丢弃','Jetson 主机帧就绪队列',capture.get('skip_host'),'帧','主动跳帧',('当前顺序直写不主动丢弃 READY 旧帧；本项不是 YOLO 采样跳帧。DDR 视频独立按序发送。' if direct else 'DMA 已完成的旧 READY 缓冲被回收，采集应用只取最新完成帧。')),
 row('send_skip','发送取帧跳过',('FPGA DDR → ERNIC' if direct else 'Jetson SHM → RDMA 提交前'),send.get('source_skipped'),'帧','源序号缺口' if direct else '主动跳帧',('DDR 完整帧描述符序号缺口；直传按序发送，不沿用主机预览 latest 策略。' if direct else '发送线程跨过的 SHM 发布序号；这些帧没有提交 RDMA，不是网络丢包。')),
 row('rdma_missing','RDMA 未交付帧','ERNIC → Mellanox',undelivered,'帧','结束后核对' if final else '尚不能归因','仅同一会话双方正常结束且最终计数齐全时，核对提交数与接收数；传输中不把在途帧判为丢失。CRC32 不能代替网络丢包计数。'),
 row('crc','接收 CRC 错帧','Mellanox 整帧校验',recv.get('crc_errors') if crc_active else None,'帧','完整性错误' if crc_active else '当前会话未检','收到整帧后 CRC32 不匹配的帧数；与网络重传次数不同。' if crc_active else '当前旧会话未逐帧计算 CRC；停止两端并重新开始采集后启用。'),
 row('transfer_fault','传输失败事件','发送 / 接收软件',None if not send and not recv else send.get('faults',0)+recv.get('faults',0),'次','错误事件','超时、控制连接断开等失败日志次数；一次事件可能涉及多帧，不能换算为丢帧数。'),
 row('preview','显示源序号缺口','本端完整帧 → 顺序呈现',preview,'帧','源序号缺口','主视频逐帧呈现并确认后释放，不主动越过已入队帧；这里统计输入源帧号缺口。待呈现队列不计为丢帧。'),
 row('save','保存队列跳帧','预览采样 → 写盘队列',save,'帧','保存背压','已达到保存采样时点但写盘队列满而放弃的图像；主动降低保存采样率不计为队列丢帧。')
 ]


STAGE_HELP={
 "input_bad": {
  "brief": "相机输入无有效像素或同步异常的帧数。",
  "meaning": "统计 FPGA 在 Camera Link 帧结束时发现无有效像素、帧同步异常或同步副本不一致的帧。",
  "purpose": "定位相机 → Camera Link → FPGA 输入这一段是否正常，区分源端异常与后端传输问题。",
  "check": "连续增加时检查相机供电、Base/Full 接线、tap 模式、宽高与像素格式，以及输入同步/对齐诊断。",
  "limit": "这是输入诊断计数，不是总丢帧数；同一坏帧还可能计入 FPGA 丢帧，不能相加。只覆盖硬件能识别的异常，不能统计没有到达 FPGA 的曝光。"
 },
 "fpga_drop": {
  "brief": "FPGA 未能发布到 DDR 的帧数。",
  "meaning": "统计 FPGA 因输入/尺寸异常、缓存或写入路径异常等原因丢弃、没有正常发布为完整 DDR 帧的数量。",
  "purpose": "判断视频是否在进入可读取的 DDR 完整帧队列前已经丢失。",
  "check": "持续增加时结合 DDR 占用、帧宽高/字节数、输入异常和最后原因码排查。环形槽满只是可能原因之一；AXI 响应错误等也会触发丢帧。",
  "limit": "硬件累计值以相关采集逻辑复位/重新配置清零为界。最后原因码只描述最近一次原因，不是每种原因的累计数量；输入异常与本项可能重叠。"
 },
 "ddr_skip": {
  "brief": "旧版 DDR→主机 DMA 跳帧计数；当前路径应为 0。",
  "meaning": "这是旧版 DDR→Jetson DMA 取最新帧时跳过完整 DDR 槽的兼容计数。当前视频只从 ISP 直写主机，不再提交该类 D2H 读取。",
  "purpose": "确认旧视频 DMA 路径未被误启用，并区分历史计数与 RDMA 丢帧。",
  "check": "当前采集会话应为 0；若增加，检查是否运行了旧版采集程序或旧 bit。",
  "limit": "不是当前 ISP→主机帧的丢帧数，也不是网络丢包数。DDR 槽在 RDMA 完成后释放；全帧 CRC 模式还等待主机参考值。"
 },
 "host_skip": {
  "brief": "主机直写帧就绪队列中被取最新策略回收的旧帧。",
  "meaning": "主机直写缓冲已 READY 后，如果启用取最新策略，采集程序可能回收较旧的 READY 帧；当前顺序模式不主动回收。",
  "purpose": "判断 Jetson 采集程序是否为追赶最新画面而主动越过了已写入主机内存的帧。",
  "check": "顺序采集模式不主动回收未交付旧帧，本项应为 0；若增加，检查是否启用了 latest-frame 及主机处理负载。",
  "limit": "这不是 YOLO 未推理的帧数。DDR 直传视频走独立所有权路径，不能直接把本项换算成 Mellanox 少收到的帧数。"
 },
 "send_skip": {
  "brief": "发送取帧时观察到的源发布序号缺口。",
  "meaning": "发送线程相邻取帧的源序号之间，未被该线程取到的帧数。DDR 直传观察完整帧描述符序号；主机路径观察 SHM 发布序号。",
  "purpose": "判断发送端在提交 RDMA 前是否已经越过源帧。",
  "check": "DDR 直传按序读取，本项应为 0；若增加，结合 FPGA 丢帧、描述符连续性及发送日志定位。主机 SHM 取最新路径可因覆盖旧帧而增长。",
  "limit": "缺口本身只说明发送源序号不连续，不能单凭本项断定网络丢包或全部由发送线程主动丢弃；上游缺帧也可能体现为序号缺口。"
 },
 "rdma_missing": {
  "brief": "双方正常结束后核对提交与接收数量。",
  "meaning": "仅同一会话的发送端与接收端都正常结束并有最终 PASS 计数时，计算 max(0, 最终提交帧数 − 最终接收帧数)。",
  "purpose": "核对已经提交到 RDMA 的完整视频帧是否全部交付到接收软件。",
  "check": "传输中显示“—”是尚不能确定，绝不把在途帧记成丢失。正常停止排空后应为 0；非 0 时检查双端结束日志、会话、提交和接收计数。",
  "limit": "失败、超时、会话不同或未正常结束时不能用它证明零丢帧；即使为 0，也不证明源头从未跳帧。它是整帧交付核对，不是网络包丢失/重传计数。"
 },
 "crc": {
  "brief": "Mellanox 收到整帧后 CRC32 不匹配的次数。",
  "meaning": "接收端将整帧实际数据计算 CRC32，与发送端提供的参考值比较；不一致计入本项。",
  "purpose": "发现图像数据错位、损坏或缓冲区过早复用等端到端完整性问题。",
  "check": "仅诊断模式统计；低延迟模式显示 —，不表示零错帧。诊断模式应保持 0。增加时检查 DMA 地址/长度、槽所有权、缓存一致性、RTL 数据通路及双方日志，不只检查光纤。",
  "limit": "CRC 错误不等于没收到帧；CRC 只能检查已收到的数据，不能直接表示网络丢包或重传次数，也不能证明像素内容符合相机真实场景。"
 },
 "transfer_fault": {
  "brief": "发送/接收失败日志的记录次数，不是丢帧数。",
  "meaning": "累计发送端和接收端被识别为失败的日志记录，例如完成超时、控制连接异常等。",
  "purpose": "快速发现传输软件/硬件链路是否出现故障，并提示查看详细日志。",
  "check": "应保持 0。增加时查看双端运行日志和错误信息；如进入 DMA 安全保持，先确认 FPGA DMA 已停止，再处理进程与内存。",
  "limit": "一次故障可能产生多条日志，也可能影响多帧；此值既不能直接换算为丢帧数，也不一定等于互相独立的故障次数。"
 },
 "preview": {
  "brief": "本端显示队列观察到的源帧号缺口。",
  "meaning": "本端按序呈现队列根据相邻完整帧的源帧号差值，累计中间缺少的源帧号数量。",
  "purpose": "判断交给本端显示的输入帧序列是否连续；Jetson 与 Mellanox 分别统计各自显示输入。",
  "check": "顺序视频正常时应为 0。增加时先检查上游采集/发送的序号；显示队列待处理数量是积压，不记入本项。",
  "limit": "当前 GPU 在提交成功后才确认消费，不主动覆盖未呈现队列帧。但此计数不直接测量显示器物理扫描：屏幕刷新率不足仍可能看不到每一张提交帧。也不统计 YOLO 跳过推理的帧。"
 },
 "save": {
  "brief": "到达保存采样时点但写盘队列已满。",
  "meaning": "独立图像保存线程到达设置的保存采样时点，但待写盘队列已满，因此放弃本次保存的图像数。",
  "purpose": "判断保存速率是否超过图像编码和磁盘写入能力。",
  "check": "未连续保存时可能为 0/“—”；增加时降低保存 fps、使用更快磁盘并检查磁盘空间及保存错误。",
  "limit": "只影响保存出来的图像文件，不直接表示采集、RDMA 或显示丢帧。例如视频 140 fps、保存 5 fps 是主动采样，其余 135 帧不计入保存队列跳帧。"
 }
}
STAGE_COUNT_GUIDE="各项不能直接相加：输入异常可能与 FPGA 丢帧重叠，源序号缺口也可能反映上游缺帧。0 表示本项未记录异常；— 表示暂不可用或尚不能核对。硬件、软件和显示队列的计数周期不同；两端轮询时刻也不同。YOLO 取最新帧、只推理部分视频帧是独立采样，不计入这些视频丢帧项。"

def stage_detail(row):
 """Explain the exact counter, its diagnostic use, and its limits."""
 h=STAGE_HELP[row['key']]
 value='—（暂不可用或尚不能核对）' if row['value'] is None else str(row['value'])+' '+row['unit']
 return '\n'.join((row['stage']+'  |  '+row['where']+'  |  当前：'+value,'统计含义：'+h['meaning'],'主要作用：'+h['purpose'],'如何判断：'+h['check'],'注意边界：'+h['limit'],'当前路径说明：'+row['note']))
