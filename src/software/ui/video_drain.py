"""Request FPGA frame-boundary input stop, drain DMA, then stop camera."""
import os
import json,mmap,re,struct,subprocess,time
from pathlib import Path

def drained(row):
 end=row['producer']
 # A completed but unread host slot is not an in-flight DMA.  The
 # writer's hardware IDLE bit includes empty pixel/metadata FIFOs and zero
 # outstanding AXI writes; bound the unread count by the configured slots.
 host_prod=row.get('host_hw_producer')
 host_cons=row.get('host_hw_consumer')
 host_slots=row.get('host_slots',0)
 host_credit_ok=(isinstance(host_prod,int) and isinstance(host_cons,int) and
                 isinstance(host_slots,int) and 1<=host_slots<=32 and
                 ((host_prod-host_cons)&0xffffffff)<=host_slots)
 host_ok=(not row.get('host_enabled',False) or
          (bool(row.get('host_status',0)&1) and not row.get('host_error',0) and
           host_credit_ok))
 # The display queue owns a completed CPU copy of each DMA frame. Its
 # produced/consumed counters track GUI presentation, not FPGA/DDR/DMA
 # ownership; a closed or slow GUI may legitimately leave copied frames
 # unread. The DDR release counters and optional host hardware idle/credit
 # counters above are the safety conditions for stopping capture.
 return bool(host_ok and row['consumer']==end and row['published']==end and
             row['released']==end)

def tail_abort_ready(row):
 # Rejected Camera Link frames consume host slots but never advance the DDR
 # producer or the valid-frame display queue. Compare ownership within each
 # domain instead of requiring their unrelated frame counters to be equal.
 return bool(row['producer']==row['consumer']==row['published']==row['released'] and
             row['host_hw_producer']==row['host_hw_consumer'])

def park_camera_source(binary,run=subprocess.run):
 # TriggerMode only selects free-run/trigger behavior; it does not stop the
 # camera acquisition engine. AcquisitionStop is the camera's actual stop command.
 cmd='w AcquisitionStop 1'
 p=run([str(binary),'--fpga','--cmd-timeout-ms','2000','--cmd',cmd],
       capture_output=True,text=True,timeout=3)
 output=str(p.stdout or '')
 if p.returncode or 'Success!' not in output or 'Failed!' in output:
  detail=' '.join((output+' '+str(p.stderr or '')).split())[-800:]
  raise RuntimeError('相机 AcquisitionStop 未确认；FPGA 与 DMA 保留：'+cmd+'：'+detail)
 return dict(command=cmd,acknowledged=True)

def park_and_drain(root,session,timeout=8):
 root=Path(root)
 if not re.fullmatch('[0-9a-f]{24}',session):raise ValueError('无效采集会话')
 path=root/'runtime'/('ddr_'+session+'.bin')
 marker=path.with_name(path.name+'.park')
 with open(os.environ.get('FPGA_RESOURCE','/sys/bus/pci/devices/0005:01:00.0/resource2'),'r+b',buffering=0) as bar, \
      path.open('rb') as ddr,(root/'runtime/display_memory/frames.bin').open('rb') as display:
  with mmap.mmap(bar.fileno(),0x15000,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ|mmap.PROT_WRITE) as hw, \
       mmap.mmap(ddr.fileno(),0,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ) as ring, \
       mmap.mmap(display.fileno(),0,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ) as fifo:
   def read(offset):return struct.unpack_from('<I',hw,offset)[0]
   if read(0x1406c)!=0x52474233:raise RuntimeError('未识别采集核心，拒绝修改采集使能')
   control=read(0x14000)
   build=read(0x14070)
   if control&1:
    if build<0x00030023 or build==0xffffffff:
     raise RuntimeError('当前 bit 0x%08x 不支持帧边界安全停止；需下载 0x00030023 或更新版。本次未停源、复位或释放 DMA' % build)
    # Mark the intentional pause before the UART command. Keep the viewer's
    # DMA pump alive while final pixels, footers, RDMA and display drain.
    try:previous=json.loads(marker.read_text())
    except (OSError,ValueError):previous={}
    already_parked=(bool(control&0x10) and previous.get('session')==session and
                    previous.get('phase')=='camera_parked')
    if not already_parked:
     marker.write_text(json.dumps(dict(session=session,phase='fpga_frame_boundary_stop')))
     marker.chmod(0o644)
     # AcquisitionStop is acknowledged before gating a later SOF.
     park_camera_source(root/'build/hik_camera_cfg')
     marker.write_text(json.dumps(dict(session=session,phase='camera_parked')))
     struct.pack_into('<I',hw,0x14000,control|0x10)
     if not read(0x14000)&0x10:raise RuntimeError('FPGA 帧边界停止请求未接受，进程和 DMA 保留')
   deadline=time.monotonic()+timeout;last=None;quiet=time.monotonic();abort_requested=False
   while True:
    row={'producer':read(0x14024),'consumer':read(0x14020),'drops':read(0x14038),
         'start_idx':struct.unpack_from('<I',ring,28)[0],
         'published':struct.unpack_from('<Q',ring,40)[0],
         'released':struct.unpack_from('<Q',ring,48)[0],
         'host_produced':struct.unpack_from('<Q',fifo,64)[0],
         'host_consumed':struct.unpack_from('<Q',fifo,128)[0],
         'host_enabled':bool(read(0x14a00)&1),'host_slots':read(0x14a04),'host_status':read(0x14a10),
         'host_hw_producer':read(0x14a14),'host_hw_consumer':read(0x14a0c),
         'host_error':read(0x14a1c),'host_debug':read(0x14a20),
         'input_gate_ack':bool(read(0x14000)&0x20)}
    now=time.monotonic()
    if row['producer']!=last:last=row['producer'];quiet=now
    if row['producer']==0xffffffff or row['host_status']==0xffffffff:
     raise RuntimeError('停源期间 PCIe BAR 不可访问，进程保留')
    if row['host_error']:raise RuntimeError('主机直写 DMA 硬件错误，进程和内存保留：'+json.dumps(row))
    # 0x3002f can discard an unpublished host tail only after all completed
    # DDR and host frames have been consumed and the input gate is closed.
    # The hardware then drains its pixel FIFO and every AXI BRESP before IDLE.
    full_frames_done=tail_abort_ready(row)
    if (control&1 and build>=0x0003002f and row['input_gate_ack'] and
        row['host_enabled'] and not row['host_status']&1 and not row['host_error'] and
        (row['host_debug']&15)==1 and full_frames_done and now-quiet>=.04 and
        not abort_requested):
     struct.pack_into('<I',hw,0x14a28,1)
     if read(0x14a28)!=1:raise RuntimeError('主机帧尾丢弃请求未被接受，保留 DMA')
     abort_requested=True
    if (not control&1 or row['input_gate_ack']) and drained(row) and now-quiet>=.04:
     # Finish the engine only after the in-progress frame and all DMA are idle.
     # Never use equal software counters to skip the hardware IDLE proof.
     if control&1:
      if (read(0x14024)!=row['producer'] or read(0x14020)!=row['consumer'] or
          read(0x14a14)!=row['host_hw_producer'] or read(0x14a0c)!=row['host_hw_consumer'] or
          read(0x14a1c) or (row['host_enabled'] and not read(0x14a10)&1)):
       raise RuntimeError('相机结束采集后 DMA 状态变化，保留进程和内存')
     # Disabling capture resets HLS immediately. Do this only after hardware
     # IDLE/empty and all ownership counters agree, never at an arbitrary pixel.
     struct.pack_into('<I',hw,0x14000,control&~0x31)
     if read(0x14000)&1:raise RuntimeError('采集使能未停止，进程和内存保留')
     if row['host_enabled']:
      if not read(0x14a10)&1:
       raise RuntimeError('关采集后主机写入器未空闲，保留进程和 DMA 内存')
      # The control CSR accepts disable only while capture is off.  A
      # completed but unread host slot may remain; IDLE proves its BRESP
      # and footer are complete before the buffer owner can exit.
      struct.pack_into('<I',hw,0x14a00,0)
      if read(0x14a00)&1 or not read(0x14a10)&1:
       raise RuntimeError('主机直写未安全关闭，保留进程和 DMA 内存')
      if build>=0x0003002f:
       struct.pack_into('<I',hw,0x14a28,0)
     row.update(quiet_ms=(now-quiet)*1000,source_parked=True,fpga_gated_after_hardware_idle=True)
     out=root/'runtime'/('source_drain_'+session+'.json')
     out.write_text(json.dumps(row,indent=2));out.chmod(0o644)
     return row
    if now>=deadline:raise RuntimeError('FPGA 已请求帧边界停止，但输入握手/当前帧/硬件 DMA/显示尚未排空，保留进程：'+json.dumps(row))
    time.sleep(.002)
