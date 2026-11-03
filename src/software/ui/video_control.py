#!/usr/bin/env python3
"""Fixed-command, local UNIX-socket hardware supervisor. Run with sudo."""
import os,sys,json,time,signal,socket,socketserver,threading,subprocess,fcntl,re,pwd
from pathlib import Path
from video_config import validate_capture
from collections import deque
ROOT=Path(__file__).resolve().parent
JETSON=os.uname().machine=='aarch64'
RUN=ROOT/'runtime'

def require_ordered_display():
 if JETSON:return
 path=RUN/'display_gpu_status.json'
 hint='Mellanox 本地逐帧窗口未就绪；请在本机图形桌面执行 ./video.sh station，并等待 GPU 显示就绪后再开始采集'
 try:
  report=json.loads(path.read_text())
  pid=int(report.get('pid') or 0)
  updated=int(report.get('updated_ns') or 0)
  cmd=(Path('/proc')/str(pid)/'cmdline').read_bytes()
 except (OSError,ValueError,TypeError):
  raise ValueError(hint)
 if pid<=0 or b'video_desktop.py' not in cmd or not updated or abs(time.monotonic_ns()-updated)>3_000_000_000:
  raise ValueError(hint)
 if not report.get('ready') or report.get('error'):
  raise ValueError('Mellanox GPU 显示未就绪：'+str(report.get('error') or '渲染器仍在启动'))
 expected=str(RUN/'display_memory/frames.bin')
 if report.get('display_path')!=expected:
  raise ValueError('Mellanox 显示窗口未连接按序帧队列；请重启本地窗口')


def capture_host_path(cfg,build_version,requested='auto'):
 if cfg['transport']!='ddr':
  raise ValueError('普通视频仅支持 FPGA DDR→RDMA；本地画面由 ISP→PCIe 直写')
 if build_version<0x0003002d:
  raise ValueError('ISP 全帧直写需要 FPGA 版本 0x0003002d；请下载新 bit 后启动')
 if requested not in ('','auto','direct'):
  raise ValueError('普通采集已移除 DDR→Jetson 视频 DMA，请使用 ISP→PCIe 直写')
 return 'direct'

class Manager:
 def __init__(self):
  self.lock=threading.RLock();self.procs={};self.stats={};self.events=deque(maxlen=60);self.generation=0;self.configs={};self.camera_readback={};self.yolo_enabled=False;self.event_lock=threading.Lock()
 def ensure_yolo_worker(self):
  """Warm one detector independently of camera DMA; no engine reload on toggle."""
  if not JETSON:return
  with self.lock:
   p=self.procs.get('yolo')
   if p and p.poll() is None:return
   from video_yolo_deploy import ENGINE,WEIGHTS,usable_engine
   from video_yolo_state import write_state
   python=Path('/usr/bin/python3')
   engine=ENGINE
   if not usable_engine():
    write_state(RUN/'yolo_status.json',dict(ready=False,phase='model_missing',
      error='缺少 YOLO 模型：将 yolo11n.pt 放入 '+str(WEIGHTS.parent)+'，重新启动工作站以生成引擎'))
    return
   if self.external('yolo'):raise ValueError('存在外部 YOLO 进程，请先正常停止')
   write_state(RUN/'yolo_status.json',dict(ready=False,phase='loading'))
   cmd=[str(python),str(ROOT/'video_yolo_worker.py'),'--resident-control',str(RUN/'yolo_command.json'),
        '--resident-status',str(RUN/'yolo_status.json'),'--engine',str(engine),'--imgsz','384','640',
        '--classes','0','--conf','0.30','--iou','0.50','--max-det','20','--tracker','bytetrack.yaml',
        '--input-mode','tensor','--runner','persistent','--no-display']
   env=os.environ.copy();env.update(TMPDIR=str(ROOT/'.tmp'),PYTHONUNBUFFERED='1',
       PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',
       MKL_NUM_THREADS='1',YOLO_CONFIG_DIR=str(ROOT/'.tmp/ultralytics'),
       CUDA_CACHE_PATH=str(ROOT/'.tmp/cuda_cache'))
   owner=Path(pwd.getpwuid(ROOT.stat().st_uid).pw_dir)
   site=owner/'.local'/f'lib/python{sys.version_info.major}.{sys.version_info.minor}'/'site-packages'
   if site.is_dir():env['PYTHONPATH']=str(site)+os.pathsep+env.get('PYTHONPATH','')
   self.generation+=1;log=RUN/('yolo_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(self.generation)+'.log')
   p=subprocess.Popen(cmd,cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,
       stderr=subprocess.STDOUT,text=True,start_new_session=True,bufsize=1)
   self.procs['yolo']=p;self.stats['yolo']={};self.configs['yolo']={}
   threading.Thread(target=self.reader,args=('yolo',p,log),daemon=True).start()
   self.event('YOLO 后台预热 · PID '+str(p.pid))
 def shutdown_idle_worker(self):
  # Keep the supervisor and its process ownership alive until CUDA exits.
  with self.lock:
   if any(p.poll() is None for mode,p in self.procs.items() if mode!='yolo') or self.yolo_enabled:return False
   worker=self.procs.get('yolo')
   if worker and worker.poll() is None:
    worker.send_signal(signal.SIGTERM)
    try:worker.wait(timeout=10)
    except subprocess.TimeoutExpired:return False
   return True
 def set_yolo_command(self,cfg,enabled):
  from video_yolo_state import write_state
  self.yolo_enabled=enabled
  write_state(RUN/'yolo_command.json',dict(session=cfg.get('session','') if cfg.get('format')=='rgb' else '',enabled=enabled))
  self.configs['yolo']=dict(cfg)
 def enable_yolo(self,enabled):
  with self.lock:
   cfg=dict(self.configs.get('capture',{}))
   p=self.procs.get('capture')
   if enabled and (not p or p.poll() is not None or cfg.get('format')!='rgb'):
    raise ValueError('YOLO 需要本工作站的 RGB 采集')
   self.ensure_yolo_worker()
   worker=self.procs.get('yolo')
   if enabled and (not worker or worker.poll() is not None):
    from video_yolo_deploy import WEIGHTS
    raise ValueError('YOLO 模型未部署；将 yolo11n.pt 放入 '+str(WEIGHTS.parent)+'，重新启动工作站')
   self.set_yolo_command(cfg,enabled)
   if 'capture' in self.configs:self.configs['capture']['yolo']=enabled
   return {'message':'YOLO 已启用，复用后台预热模型' if enabled else 'YOLO 已暂停，模型保留；过期框自动隐藏',
           'pid':worker.pid if worker else None}
 def event(self,msg):
  row=dict(time=time.strftime('%H:%M:%S'),message=msg,id=time.time_ns())
  with self.event_lock:
   self.events.append(row)
   try:
    if RUN.exists():
     with (RUN/'events.jsonl').open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
   except OSError:pass
 def park_after_sender_fault(self,p):
  # Contain the source only. Never gate/reset FPGA or free either DMA owner.
  # A full DDR ring cannot be drained while the sender is held.
  if not JETSON:return
  with self.lock:
   if self.procs.get('sender') is not p:return
   capture=self.procs.get('capture')
   cfg=self.configs.get('capture',{})
   if not capture or capture.poll() is not None or cfg.get('transport')!='ddr':return
   session=cfg.get('session','')
   if not re.fullmatch('[0-9a-f]{24}',session):
    self.event('发送故障停源失败：无有效采集会话，DMA 内存保留');return
   marker=RUN/('ddr_'+session+'.bin.park')
   try:
    marker.write_text(json.dumps(dict(session=session,phase='sender_fault')))
    marker.chmod(0o644)
    from video_drain import park_camera_source
    park_camera_source(ROOT/'build/hik_camera_cfg')
    self.stats.setdefault('capture',{}).update(source_paused=True,pause_reason='RDMA 发送故障，已停相机；DMA 内存保留')
    self.event('RDMA 发送故障：已停相机，DDR 与 DMA 所有权继续保留')
   except Exception as exc:
    self.event('RDMA 发送故障停源失败：'+str(exc)+'；DMA 内存保留')
 def reader(self,mode,p,log):
  with log.open('a',buffering=1) as f:
   for line in p.stdout:
    f.write(line)
    if self.procs.get(mode) is not p:continue
    values=dict(re.findall(r'(\w+)=([^\s]+)',line))
    if line.startswith('VIDEO ') or (mode=='capture' and line.startswith('frames=')) or (mode=='yolo' and line.startswith('detect_fps=')):
     parsed={}
     for k,v in values.items():
      try:parsed[k]=float(v) if '.' in v else int(v)
      except ValueError:parsed[k]=v
     old=self.stats.get(mode,{})
     parsed.update(updated=time.monotonic(),first_seq=old.get('first_seq',parsed.get('seq',parsed.get('frame_seq'))),first_count=old.get('first_count',parsed.get('received',parsed.get('frames',0))),ddr_ready=old.get('ddr_ready',False),source_park_supported=old.get('source_park_supported',False),boxes_channel=parsed.get('boxes_channel',old.get('boxes_channel',0)),boxes_updates=old.get('boxes_updates',0),boxes_frame_seq=old.get('boxes_frame_seq',0),source_paused=old.get('source_paused',False),pause_reason=old.get('pause_reason',''),waiting_source=False,diag=old.get('diag',{}),faults=old.get('faults',0),last_error=old.get('last_error',''),first_error=old.get('first_error',''),log_path=old.get('log_path',''))
     self.stats[mode]=parsed
    if line.startswith(('RX_IDLE ','TX_IDLE ')):
     self.stats.setdefault(mode,{}).update(waiting_source=values.get('waiting_source')=='1',fps=0,updated=time.monotonic())
    if mode=='capture' and line.startswith('SOURCE_PARK capability=1'):
     self.stats.setdefault(mode,{})['source_park_supported']=True
    if mode=='capture' and line.startswith('DDR_RING ready=1'):
     self.stats.setdefault(mode,{})['ddr_ready']=True
    if line.startswith('BOXES '):
     state=self.stats.setdefault(mode,{})
     state['boxes_channel']=1
     state['boxes_updates']=int(values.get('sent',values.get('received',0)))
     state['boxes_frame_seq']=int(values.get('frame_seq',0))
    if mode=='capture' and line.startswith('csr '):
     diag={}
     for k,v in values.items():
      try:diag[k]=int(v,0)
      except ValueError:diag[k]=v
     self.stats.setdefault(mode,{})['diag']=diag
    failed=line.startswith(('FAIL','Error','DMA frame wait failed','reject frame:')) or 'capture start failed:' in line or 'timed out waiting for first DMA frame' in line
    if failed:
     state=self.stats.setdefault(mode,{})
     state['faults']=state.get('faults',0)+1
     state['last_error']=line.strip()[:4096]
     if not state.get('first_error'):state['first_error']=state['last_error']
     self.event(mode+': '+state['last_error'])
    if line.startswith('FAIL: video CRC'):
     self.stats.setdefault(mode,{})['crc_errors']=self.stats.get(mode,{}).get('crc_errors',0)+1
    if line.startswith('PASS:'):
     self.stats.setdefault(mode,{})['passed']=True
    if line.startswith('PASS: received and CRC-verified'):
     m=re.search(r'verified (\d+) video frames; last_seq=(\d+) crc_errors=(\d+)',line)
     if m:self.stats.setdefault(mode,{}).update(received=int(m[1]),crc_checked=int(m[1]),seq=int(m[2]),crc_errors=int(m[3]),updated=time.monotonic())
    if line.startswith('PASS: received='):
     self.stats.setdefault(mode,{}).update(received=int(values['received']),seq=int(values['seq']),crc_mode=values['crc_mode'],crc_checked=int(values['crc_checked']),updated=time.monotonic())
    if line.startswith('PASS: video frames='):
     for k in ('frames','source_skipped','bytes_per_frame'):
      if k in values:self.stats.setdefault(mode,{})[k]=int(values[k])
     self.stats.setdefault(mode,{})['submitted']=self.stats[mode].get('frames',0)
    if 'safety hold' in line:
     self.stats.setdefault(mode,{})['safety_hold']=True;self.event('DMA 安全保持：保留进程和内存，请检查硬件')
     if JETSON and mode=='sender' and not self.stats[mode].get('fault_source_park_scheduled'):
      self.stats[mode]['fault_source_park_scheduled']=True
      threading.Thread(target=self.park_after_sender_fault,args=(p,),daemon=True).start()
    elif line.startswith('PASS'):
     self.event(mode+': '+line.strip()[:220])
  code=p.wait()
  if self.procs.get(mode) is p:
   state=self.stats.setdefault(mode,{})
   state['exit_code']=code
   if code and not state.get('stopping') and not state.get('last_error'):state['last_error']=mode+' 已退出，返回码 '+str(code)
   self.event(mode+' 已退出，返回码 '+str(code))
 def external(self,mode):
  needle={'capture':'camera_viewer_bayerrg8_egl','sender':'ernic_video_sender','receiver':'mlx5_video_receiver','yolo':'video_yolo_worker.py'}[mode]
  own={p.pid for p in self.procs.values() if p.poll() is None}
  for d in Path('/proc').iterdir():
   if not d.name.isdigit() or int(d.name) in own:continue
   try:
    cmd=(d/'cmdline').read_bytes().split(b'\0')
    if cmd and any(Path(os.fsdecode(arg)).name==needle for arg in cmd[:2]):return int(d.name)
   except OSError:pass
  return None
 def start(self,mode,config):
  if mode=='yolo':return self.enable_yolo(True)
  with self.lock:
   allowed=('capture','sender','yolo') if JETSON else ('receiver',)
   if mode not in allowed:raise ValueError('本端不支持该任务')
   p=self.procs.get(mode)
   if self.stats.get(mode,{}).get('safety_hold'):
    raise ValueError(mode+' 仍处于上一轮 DMA 安全保持；确认 FPGA 已重配置并恢复 PCIe 映射后，重启控制服务清除旧进程归属')
   if p and p.poll() is None:
    active=self.configs.get(mode,{}).get('session')
    requested=config.get('_session') or config.get('session')
    if requested and active!=requested:
     raise ValueError(mode+' 仍在上一采集会话运行，不能把旧缓冲或 QP 当成新会话；请先安全停止旧会话')
    return {'message':mode+' 已运行'}
   if self.external(mode):raise ValueError('存在终端启动的 '+mode+'，请先在原终端正常停止')
   cfg=validate_capture(config);fps=cfg['fps'];buffers=cfg['buffers']
   pattern=bool(config.get('pressure_pattern'))
   if pattern:
    if cfg['transport']!='ddr' or cfg['format']!='rgb' or cfg['crc_mode']!='transport' or cfg['yolo']:
     raise ValueError('彩条压力测试必须使用 DDR/RGB、链路校验并关闭 YOLO')
    if not any(config.get(k) is True for k in ('pressure_dma','pressure_rdma')):
     raise ValueError('请至少选择 DMA 或 RDMA 压力通道')
   if mode=='receiver':require_ordered_display()
   session=str(config.get('_session') or config.get('session') or self.configs.get('capture',{}).get('session',''))
   if not re.fullmatch('[0-9a-f]{24}',session):
    if mode=='yolo':raise ValueError('YOLO 缺少有效采集会话')
    session='0'*24
   detector_shm='/lrdma_detect_'+session;boxes_shm='/lrdma_boxes_'+session
   cmd=['bash',str(ROOT/'video.sh'),mode]
   if mode=='receiver' and pattern and not config.get('pressure_rdma'):
    if not (ROOT/'video_pattern_source').exists():raise ValueError('缺少已编译的 video_pattern_source')
    cmd=[str(ROOT/'video_pattern_source'),'--preview-only',
         '--size',str(cfg['width'])+'x'+str(cfg['height']),
         '--output-file','./runtime/display_memory/latest_frames.bin']
   if mode=='capture' and pattern:
    if not (ROOT/'video_pattern_source').exists():raise ValueError('缺少已编译的 video_pattern_source')
    cmd=[str(ROOT/'video_pattern_source'),'--ddr-ring',str(RUN/('ddr_'+session+'.bin')),
         '--size',str(cfg['width'])+'x'+str(cfg['height']),
         '--output-file','./runtime/display_memory/latest_frames.bin']
    if config.get('pressure_dma'):cmd+=['--dma-stress']
    if config.get('pressure_rdma'):cmd+=['--rdma-stress']
   elif mode=='capture':
    from video_capture_build import require_capture_core
    capture_core=require_capture_core(cfg)
    cmd+=['--fifo-display']
    if time.time()-self.camera_readback.get('updated',0)>5:self.read_camera()
    from video_camera import check_geometry
    check_geometry(cfg,self.camera_readback)
    from video_preprocess import status as isp_status,apply
    pp=isp_status()
    if pp.get('available') or 'preprocess' in cfg:
     if any(p.poll() is None for mode,p in self.procs.items() if mode!='yolo') or self.external('sender'):raise ValueError('先安全停止采集与发送，再提交 ISP 参数')
     if not pp.get('available'):raise ValueError('所选 ISP 扩展需要配套新 bitstream')
     values=cfg.get('preprocess',{'enable':0})
     active=apply(values,reset_stopped_capture=True)
     self.event('FPGA ISP 提交并回读：generation=%s mask=0x%x'%(active['generation'],active['enable']))
    cmd+=['--size',str(cfg['width'])+'x'+str(cfg['height']),'--fps',str(fps),'--buffers',str(buffers),'--exposure-us',str(cfg['exposure_us']),'--black-level',str(cfg['black_level']),'--wb-r',str(cfg['wb_r']),'--wb-g',str(cfg['wb_g']),'--wb-b',str(cfg['wb_b'])]
    if 'ccm' in cfg:cmd+=['--ccm',','.join(str(v) for v in cfg['ccm'])]
    if 'isp_enable' in cfg:cmd+=['--isp-enable',str(cfg['isp_enable'])]
    cmd+=['--test-pattern',str(cfg.get('test_pattern',0))]
    if cfg['format']=='rgb':
     cmd+=['--detect-shm-name',detector_shm,'--detect-size','640x384']
     cmd+=['--detect-on-demand'] if capture_core['version']>=0x00030027 else ['--detect-publish-fps','0']
    if cfg['format']=='bayer':cmd+=['--raw-bayer']
   ddr_path=RUN/('ddr_'+session+'.bin')
   if mode=='sender':
    cmd+=['--idle-timeout','60','--boxes-shm',boxes_shm]
    if cfg['transport']=='ddr':cmd+=['--ddr-ring',str(ddr_path)]
   if mode=='receiver' and cfg['transport']=='ddr' and (not pattern or config.get('pressure_rdma')):
    cmd+=['--boxes-channel']
   if mode=='receiver' and pattern and config.get('pressure_rdma'):
    cmd+=['--pressure-preview-ms','100']
   env=os.environ.copy();env.update(TMPDIR=str(ROOT/'.tmp'),PYTHONUNBUFFERED='1',PYTHONDONTWRITEBYTECODE='1',YOLO_CONFIG_DIR=str(ROOT/'.tmp/ultralytics'),CUDA_CACHE_PATH=str(ROOT/'.tmp/cuda_cache'))
   env['VIDEO_CRC_MODE']=cfg['crc_mode']
   if pattern and (mode=='capture' or mode=='receiver' and not config.get('pressure_rdma')):
    from video_display_memory import prepare as prepare_display
    env['VIDEO_DISPLAY_FIFO']=str(prepare_display())
   if mode=='sender' and pattern:env['VIDEO_PRESSURE_PATTERN']='1'
   else:env.pop('VIDEO_PRESSURE_PATTERN',None)
   if mode=='capture' and not pattern:
    requested=env.get('CAMERA_HOST_PATH','auto')
    selected=capture_host_path(cfg,capture_core['version'],requested)
    # The continuous ISP->PCIe frame is the sole normal host video copy.
    # YOLO samples this already-completed local preview on demand.
    env['CAMERA_YOLO_INPUT_PATH']='preview'
    env['CAMERA_HOST_PATH']=selected
    self.event('本地预览和 YOLO 共用 ISP→PCIe 直写帧；FPGA DDR 仅供 ERNIC/RDMA 视频读取')
   if mode=='capture' and cfg['transport']=='ddr' and not pattern:
    env['CAMERA_DDR_RING']=str(ddr_path)
    env['CAMERA_ANALYSIS_SHM_FPS']='10'
   else:
    env.pop('CAMERA_DDR_RING',None);env.pop('CAMERA_ANALYSIS_SHM_FPS',None)
   self.generation+=1
   log=RUN/(mode+'_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(self.generation)+'.log')
   p=subprocess.Popen(['stdbuf','-oL','-eL']+cmd,cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True,bufsize=1)
   self.procs[mode]=p;self.stats[mode]={'log_path':str(log)}
   self.configs[mode]={**cfg,'session':session}
   if pattern:self.configs[mode].update(pressure_pattern=True,
     pressure_dma=config.get('pressure_dma') is True,pressure_rdma=config.get('pressure_rdma') is True)
   if mode=='capture' and not pattern:
    self.configs[mode]['host_path']=env.get('CAMERA_HOST_PATH','auto')
    self.configs[mode]['yolo_input_path']='isp_pcie_preview'
   self.event('启动 '+mode+' · PID '+str(p.pid))
   threading.Thread(target=self.reader,args=(mode,p,log),daemon=True).start()
   time.sleep(.3)
   if p.poll() is not None:raise RuntimeError(self.stats.get(mode,{}).get('last_error') or mode+' 启动失败，查看事件日志')
   if mode=='capture' and not pattern:
    self.set_yolo_command(self.configs['capture'],False)
    self.ensure_yolo_worker()
   return {'message':mode+' 已启动','pid':p.pid}
 def stop(self,mode):
  if mode=='yolo':return self.enable_yolo(False)
  with self.lock:
   p=self.procs.get(mode)
   if not p or p.poll() is not None:
    if self.external(mode):raise ValueError('该任务由外部终端启动，请在原终端停止')
    return {'message':mode+' 已停止'}
   if self.stats.get(mode,{}).get('safety_hold'):raise ValueError('DMA 安全保持，禁止释放在途内存；先检查并复位硬件')
   if mode in ('sender','capture') and self.configs.get('capture',{}).get('transport')=='ddr' and not self.configs.get('capture',{}).get('pressure_pattern'):
    capture=self.procs.get('capture')
    if capture and capture.poll() is None:
     if not self.stats.get('capture',{}).get('source_park_supported'):
      raise ValueError('当前采集程序不支持完整帧停源；保持进程和 DMA 内存，请更新配套采集程序')
     from video_drain import park_and_drain
     row=park_and_drain(ROOT,self.configs['capture']['session'])
     self.event('相机停源、完整帧和硬件 DMA 均排空：FPGA=%d DDR释放=%d 本端呈现=%d 主机空闲=%d'%(row['producer'],row['released'],row['host_consumed'],row['host_status']&1))
   self.stats.setdefault(mode,{})['stopping']=True
   p.send_signal(signal.SIGTERM)
   try:p.wait(timeout=35)
   except subprocess.TimeoutExpired:raise RuntimeError('停止尚未完成，保留进程和 DMA 内存；没有强制结束')
   if mode=='capture':self.set_yolo_command({},False)
   self.event('停止 '+mode);return {'message':mode+' 已安全停止'}
 def status(self):
  out={}
  for mode in (('capture','sender','yolo') if JETSON else ('receiver',)):
   p=self.procs.get(mode);s=dict(self.stats.get(mode,{}))
   s.update(running=bool(p and p.poll() is None),pid=p.pid if p else None,config=self.configs.get(mode,{}))
   if mode=='yolo':
    from video_yolo_state import read_state
    warm=read_state(RUN/'yolo_status.json')
    s.update(worker_running=s['running'],ready=bool(warm.get('ready')),
             phase=warm.get('phase','loading'),error=warm.get('error',''))
    s['running']=s['worker_running'] and self.yolo_enabled
    if not s['running']:s['detect_fps']=0
   if not s['running'] and mode!='yolo':s['external_pid']=self.external(mode)
   # A retained DMA owner is alive but is no longer transferring frames.
   # Preserve final counters and ownership; do not show its last 140-fps sample
   # as current throughput after a stall or process exit.
   stale=bool(s.get('updated') and time.monotonic()-s['updated']>3)
   capture_held=bool(self.stats.get('capture',{}).get('safety_hold'))
   if s.get('safety_hold') or not s['running'] or stale or (mode=='yolo' and capture_held):
    if 'fps' in s:s['fps']=0
    if 'detect_fps' in s:s['detect_fps']=0
   if s.get('safety_hold'):s['phase']='dma_safety_hold'
   out[mode]=s
  from video_preprocess import status as isp_status
  from video_live_isp import status as online_status
  return dict(tasks=out,events=list(self.events),controller=True,online_isp=online_status() if JETSON else {},camera_readback=dict(self.camera_readback),preprocess=isp_status() if JETSON else {})
 def yolo_toggle(self,enabled):
  if not JETSON or type(enabled) is not bool:raise ValueError('无效 YOLO 控制')
  return self.enable_yolo(enabled)
 def live_isp(self,values):
  if not JETSON:raise ValueError('在线 ISP 仅在发送端执行')
  from video_live_isp import apply
  from video_isp import save_isp_mask
  with self.lock:
   p=self.procs.get('capture')
   if not p or p.poll() is not None or self.external('capture'):raise ValueError('需要本工作站启动的 RGB 采集')
   if self.configs['capture'].get('format')!='rgb':raise ValueError('原始 Bayer 不应用 ISP')
   if self.stats.get('sender',{}).get('safety_hold'):raise ValueError('DMA 安全保持期间不能修改 ISP')
   from video_preprocess import status as pp_status
   if pp_status().get('enable',0)&0x4b:raise ValueError('扩展 ISP 正在接管基础色彩处理，需要安全停止后调整该配置')
   active=apply(values)
   self.configs['capture'].update(values)
   save_isp_mask(values['isp_enable'])
   self.event('在线 ISP 已在帧边界生效：generation=%s mask=0x%x'%(active['generation'],values['isp_enable']))
   return {'online_isp':active,'message':'ISP 已在帧边界生效，采集与 RDMA 保持运行'}
 def read_camera(self):
  with self.lock:
   if not JETSON:raise ValueError('相机位于 Jetson 发送端')
   if any(p.poll() is None for mode,p in self.procs.items() if mode!='yolo') or any(self.external(m) for m in ('capture','sender')):raise ValueError('先安全停止采集和发送，再读取相机串口')
   from video_camera import read_camera
   self.camera_readback=read_camera(ROOT/'build/hik_camera_cfg')
   self.event('已读取相机：'+self.camera_readback['values']['DeviceModelName'])
   return {'camera_readback':self.camera_readback,'message':'相机型号与能力已回读'}
 def prepare(self,config):
  cfg=validate_capture(config)
  if not JETSON:raise ValueError('采集预检查必须在Jetson执行')
  for mode in ('capture','sender'):
   if self.stats.get(mode,{}).get('safety_hold'):raise ValueError(mode+' 处于 DMA 安全保持；先复位 FPGA，再恢复采集')
   p=self.procs.get(mode)
   if (p and p.poll() is None) or self.external(mode):raise ValueError('先安全停止现有 '+mode+'，再启动新采集')

  if cfg.get('yolo'):
   from video_yolo_deploy import ENGINE,usable_engine
   if not usable_engine():raise ValueError('YOLO 尚未部署：运行 ./video.sh yolo-deploy 生成 '+str(ENGINE))
  if not config.get('pressure_pattern') and not (ROOT/'build/camera_viewer_bayerrg8_egl').exists():
   raise ValueError('先在Jetson项目运行 make capture')
  if not Path('/dev/edev0').exists():
   endpoint=Path('/sys/bus/pci/devices/0005:01:00.0')
   if not endpoint.exists():
    raise ValueError('FPGA PCIe 设备 0005:01:00.0 未枚举，缺少 /dev/edev0；加载驱动模块并不表示设备已就绪。先下载正确 bit，再恢复 Jetson PCIe 枚举；若 Jetson 启动时 bit 尚未就绪，普通 PCI rescan 可能无法恢复已关闭的根端口。')
   bound=endpoint/'driver'
   driver=bound.resolve().name if bound.exists() else '未绑定'
   raise ValueError('FPGA PCIe 已枚举，但缺少 /dev/edev0；当前驱动='+driver+'，请检查 reconic_edev/edev_sgdma 的绑定及 probe 日志。')
  if config.get('pressure_pattern'):
   if not (ROOT/'video_pattern_source').exists():raise ValueError('先在 Jetson 项目运行 make pattern')
   if not Path('/dev/edev0_q64').exists():raise ValueError('缺少 FPGA DDR 队列 /dev/edev0_q64')
   return {'message':'彩条 DDR / DMA 硬件预检查通过；不启动相机'}
  from video_capture_build import require_capture_core
  require_capture_core(cfg)
  from video_preprocess import status as isp_status
  pp=isp_status()
  if 'preprocess' in cfg:
   if not pp.get('available'):raise ValueError('当前 bitstream 没有 UIP2 扩展，请使用基础 ISP 或更新配套 bitstream')
   if cfg['preprocess']['enable']&~pp.get('capabilities',0):raise ValueError('当前 FPGA 不支持选定 ISP 功能')
  self.read_camera()
  from video_camera import check_geometry
  check_geometry(cfg,self.camera_readback)
  return {'message':'采集硬件及当前相机链路尺寸预检查通过'}
 def request(self,d):
  op=d.get('op')
  if op=='status':return self.status()
  if op=='camera_read':return self.read_camera()
  if op=='yolo_toggle':return self.yolo_toggle(d.get('enabled'))
  if op=='live_isp':return self.live_isp(d.get('values',{}))
  if op=='prepare':return self.prepare(d.get('config',{}))
  if op=='start':return self.start(d.get('mode'),d.get('config',{}))
  if op=='stop':return self.stop(d.get('mode'))
  raise ValueError('不支持的操作')
class Handler(socketserver.StreamRequestHandler):
 def handle(self):
  self.connection.settimeout(45)
  d={}
  try:
   d=json.loads(self.rfile.readline(8193));result=self.server.manager.request(d);out={'ok':True,**result}
  except Exception as e:
   reason=str(e) or type(e).__name__
   out={'ok':False,'error':reason}
   if d.get('op')!='status':self.server.manager.event('控制失败 [%s/%s]：%s'%(d.get('op','请求'),d.get('mode',''),reason))
  self.wfile.write((json.dumps(out,ensure_ascii=False)+'\n').encode())
class Server(socketserver.ThreadingUnixStreamServer):
 daemon_threads=True
def main():
 if os.geteuid()!=0:sys.exit('请执行 sudo ./video.sh control')
 RUN.mkdir(exist_ok=True);(ROOT/'.tmp').mkdir(exist_ok=True)
 if '--background' in sys.argv:
  with (RUN/'control.log').open('a') as f:
   p=subprocess.Popen([sys.executable,str(ROOT/'video_control.py')],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=f,stderr=f,start_new_session=True)
  time.sleep(.5);print('控制服务日志：'+str(RUN/'control.log'));return
 lock=(RUN/'control.lock').open('a+')
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:sys.exit('控制服务已运行')
 path=RUN/'control.sock'
 if path.exists():path.unlink()
 server=Server(str(path),Handler);server.manager=Manager()
 uid=ROOT.stat().st_uid;gid=ROOT.stat().st_gid
 os.chown(path,uid,gid);os.chmod(path,0o600)
 (RUN/'control.pid').write_text(str(os.getpid()))
 if JETSON:
  server.manager.set_yolo_command({},False)
  def preload():
   try:server.manager.ensure_yolo_worker()
   except Exception as exc:server.manager.event('YOLO 预热失败：'+str(exc))
  threading.Thread(target=preload,daemon=True).start()
 print('CONTROL ready: '+str(path),flush=True)
 def shutdown(*_):
  # Refuse supervisor shutdown while its stdout pipes or DMA workers are live.
  if not server.manager.shutdown_idle_worker():
   server.manager.event('控制服务仍有运行任务或 YOLO 尚未正常退出，保留服务与进程归属');return
  threading.Thread(target=server.shutdown,daemon=True).start()
 signal.signal(signal.SIGTERM,shutdown);signal.signal(signal.SIGINT,shutdown)
 try:server.serve_forever(poll_interval=.2)
 finally:server.server_close();path.unlink(missing_ok=True)
if __name__=='__main__':main()
