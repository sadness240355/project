"""Interactive console API. Pixels remain on the existing RDMA path."""
import os,json,time,socket,threading,secrets,io,re,shutil,urllib.request,urllib.error
from pathlib import Path
from collections import deque
from PIL import Image,ImageStat
from video_pixels import to_image,BAYER
from video_stages import stage_counts
from video_pressure import PressureRunner
from video_config import validate_capture,automatic_exposure_limit_us
from video_errors import fault_report
ROOT=Path(__file__).resolve().parent
JETSON=os.uname().machine=='aarch64'
LOCAL=os.environ.get('STATION_LOCAL_IP', '10.23.68.0' if JETSON else '10.23.66.188')
PEER=os.environ.get('STATION_PEER_IP', '10.23.66.188' if JETSON else '10.23.68.0')
def control(d):
 with socket.socket(socket.AF_UNIX) as s:
  s.settimeout(60 if d.get('op')=='stop' else 42);s.connect(str(ROOT/'runtime/control.sock'));s.sendall((json.dumps(d)+'\n').encode())
  data=b''
  while b'\n' not in data:
   chunk=s.recv(65536)
   if not chunk:break
   data+=chunk
  result=json.loads(data)
  if not result.get('ok'):raise RuntimeError(result.get('error','控制失败'))
  return result
def peer_request(path,token,data=None,timeout=45):
 req=urllib.request.Request('http://'+PEER+':8788'+path,data=None if data is None else json.dumps(data).encode(),headers={'X-Station-Key':token,'Content-Type':'application/json'})
 opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
 try:
  with opener.open(req,timeout=timeout) as r:return json.load(r)
 except urllib.error.HTTPError as e:
  try:message=json.load(e).get('error',str(e))
  except (ValueError,OSError):message=str(e)
  raise RuntimeError(message) from e
class PipelineStopRequested(Exception):
 pass

class Console:
 def __init__(self,frames):
  self.frames=frames;self.csrf=secrets.token_urlsafe(24);self.save_lock=threading.Lock()
  self.token=os.environ.get('STATION_KEY','').strip() or (ROOT/'station.key').read_text().strip()
  self.lock=threading.RLock();self.operation=threading.Lock();self.operation_state=threading.RLock();self.stop_requested=threading.Event();self.operation_action='';self.busy=False;self.reconfiguring=False;self.message='就绪';self.operation_events=deque(maxlen=30);self.operation_error={}
  # Keep pipeline_error.json for logs only; a new web process is a new live status epoch.
  self.history=deque();self.history_limit=16;self.history_bytes=0;self.last_history=0
  self.recording=False;self.record_fps=5;self.record_count=0;self.record_bytes=0;self.record_last=0;self.record_drops=0
  self.save_queue=deque(maxlen=2);self.record_dir=None;self.record_error=''
  self.raw=None;self.peer_cache={};self.local_cache={};self.network={}
  self.stop=threading.Event()
  (ROOT/'captures').mkdir(exist_ok=True)
  self.pressure=None if JETSON else PressureRunner(ROOT/'results',self.status,lambda a,c:self.launch(a,c,_test=True))
  threading.Thread(target=self.monitor,daemon=True).start()
  threading.Thread(target=self.writer,daemon=True).start()
 def monitor(self):
  while not self.stop.is_set():
   try:self.local_cache=control({'op':'status'})
   except Exception as e:self.local_cache={'controller':False,'error':str(e),'tasks':{}}
   try:self.peer_cache=peer_request('/api/local/status',self.token,timeout=2)
   except Exception as e:self.peer_cache={'controller':False,'error':str(e),'tasks':{}}
   try:
    import subprocess
    rows=json.loads(subprocess.check_output(['ip','-j','address','show', 'dev','enP5p1s0' if JETSON else 'enp1s0f0np0'],text=True,timeout=2))
    n=rows[0];self.network={'interface':n['ifname'],'state':n['operstate'],'mtu':n['mtu'],'mac':n['address'],'ip':[a['local'] for a in n['addr_info'] if a['family']=='inet']}
   except Exception:self.network={}
   self.stop.wait(1)
 def on_frame(self,seq,picture,jpeg,revision):
  now=time.monotonic()
  with self.lock:
   if self.raw and revision<=self.raw[2]:return
   self.raw=(seq,picture,revision,now)
   if now-self.last_history>=.5:
    self.history.append({'id':revision,'seq':seq,'jpeg':jpeg,'time':time.strftime('%H:%M:%S')})
    while len(self.history)>self.history_limit or sum(len(x['jpeg']) for x in self.history)>64*1024*1024:self.history.popleft()
    self.last_history=now
   if self.recording and now-self.record_last>=1/self.record_fps:
    self.record_last=now
    if len(self.save_queue)>=2:self.record_drops+=1
    else:self.save_queue.append((self.record_dir,revision,seq,jpeg))
 def writer(self):
  while not self.stop.is_set():
   with self.lock:item=self.save_queue.popleft() if self.save_queue else None
   if not item:self.stop.wait(.02);continue
   directory,rev,seq,jpeg=item
   try:
    if shutil.disk_usage(ROOT).free<512*1024*1024:raise RuntimeError('磁盘剩余空间不足 512 MiB，已停止保存')
    path=directory/('frame_%012d_%012d.jpg'%(rev,seq));path.write_bytes(jpeg)
    with self.lock:
     self.record_count+=1;self.record_bytes+=len(jpeg)
     if self.record_bytes>=2*1024**3 or self.record_count>=10000:self.recording=False
   except Exception as e:
    with self.lock:self.recording=False;self.record_error=str(e)
 def status(self):
  pressure=self.pressure.snapshot() if getattr(self,'pressure',None) else self.peer_cache.get('pipeline',{}).get('test',{})
  local=self.local_status();remote=self.peer_cache
  send=(local if JETSON else remote).get('tasks',{}).get('sender',{})
  capture=(local if JETSON else remote).get('tasks',{}).get('capture',{})
  recv=(remote if JETSON else local).get('tasks',{}).get('receiver',{})
  camera=(local if JETSON else remote).get('camera_readback',{}).get('values',{})
  cfg=capture.get('config',{})
  operation=(remote if JETSON else local).get('pipeline',{})
  operation_error=operation.get('error',{})
  events=[]
  for side,node in [('Jetson',local if JETSON else remote),('Mellanox',remote if JETSON else local)]:
   events.extend({**e,'side':side} for e in node.get('events',[]))
  events.extend({**e,'side':'采集链路'} for e in operation.get('events',[]))
  faults=fault_report(dict(capture=capture,sender=send,receiver=recv,operation_error=operation_error))

  model=str(camera.get('DeviceModelName') or '型号未读取')
  width=cfg.get('width') or camera.get('Width')
  height=cfg.get('height') or camera.get('Height')
  geometry=('%s × %s'%(width,height)) if width and height else '尺寸未读取'
  pixel={'rgb':'ISP RGB888','bayer':'BayerRG8'}.get(cfg.get('format'),'输出格式待设置')
  source='相机内置测试图' if cfg.get('source')=='camera_pattern' else 'Camera Link 相机'
  
  # Scope: source frame sequence coverage at RX; includes upstream intentional skips.
  span=(recv.get('seq',0)-recv.get('first_seq',0)) if recv.get('first_seq') is not None else 0
  covered=recv.get('received',0)-recv.get('first_count',0)
  gaps=max(0,span-covered)
  def livefps(s):return s.get('fps',0) if s.get('running') and time.monotonic()-s.get('updated',0)<3 else 0
  # Peer monotonic clocks differ: remote age is supplied by its web process.
  def taskfps(s):return s.get('fps',0) if s.get('running') and (s.get('age_s') is not None and s['age_s']<5) else 0
  with self.lock:
   return dict(side='Jetson' if JETSON else 'Mellanox',local_management_ip=LOCAL,peer_management_ip=PEER,
    local_rdma_ip='192.168.1.10' if JETSON else '192.168.1.20',peer_rdma_ip='192.168.1.20' if JETSON else '192.168.1.10',
    network=self.network,controller=local.get('controller',False),peer_controller=remote.get('controller',False),
    tasks=local.get('tasks',{}),events=events,operation_error=operation_error,fault=faults,busy=(remote.get('pipeline',{}).get('busy',False) if JETSON else self.busy),message=(remote.get('pipeline',{}).get('message',self.message) if JETSON else local['pipeline']['message']),
    yolo=(local if JETSON else remote).get('tasks',{}).get('yolo',{}),
    operation_action=operation.get('action',''),stop_requested=operation.get('stop_requested',False),
    reconfiguring=(remote.get('pipeline',{}).get('reconfiguring',False) if JETSON else getattr(self,'reconfiguring',False)),
    online_isp=(local if JETSON else remote).get('online_isp',{}),
    camera_readback=(local if JETSON else remote).get("camera_readback",{}),preprocess=(local if JETSON else remote).get("preprocess",{}),capture=capture,sender=send,receiver=recv,capture_fps=taskfps(capture),receive_fps=taskfps(recv),
    pressure=pressure,coverage_gaps=gaps,stages=stage_counts(capture,send,recv,None,self.record_drops),
    history=[{k:v for k,v in x.items() if k!='jpeg'} for x in reversed(self.history)],
    history_limit=self.history_limit,recording=self.recording,record_count=self.record_count,record_bytes=self.record_bytes,
    record_drops=self.record_drops,record_error=self.record_error,record_directory=str(self.record_dir or ROOT/'captures'),
    devices=[{'id':'camera0','name':'FPGA / Camera Link · 通道 0','card':'Xilinx 10ee:9038 · 0005:01:00.0','camera':model,'source':source,'stream':pixel+' · '+geometry}])
 def yolo_toggle(self,enabled):
  if JETSON:return control({'op':'yolo_toggle','enabled':enabled})
  return peer_request('/api/local/control',self.token,{'op':'yolo_toggle','enabled':enabled})
 def live_isp(self,values):
  if JETSON:return control({'op':'live_isp','values':values})
  return peer_request('/api/local/control',self.token,{'op':'live_isp','values':values})
 def camera_read(self):
  if JETSON:return control({'op':'camera_read'})
  return peer_request('/api/local/control',self.token,{'op':'camera_read'},timeout=100)
 def local_status(self):
  d=json.loads(json.dumps(self.local_cache))
  message=self.message
  if not JETSON and not self.busy:
   tasks=[self.peer_cache.get('tasks',{}).get('capture',{}),self.peer_cache.get('tasks',{}).get('sender',{}),d.get('tasks',{}).get('receiver',{})]
   for task in tasks:
    if task.get('last_error') and not task.get('stopping'):
     message='链路故障：'+task['last_error'];break
  d['pipeline']={'error':getattr(self,'operation_error',{}),'events':list(getattr(self,'operation_events',[])),'busy':self.busy,'action':getattr(self,'operation_action',''),'stop_requested':getattr(self,'stop_requested',threading.Event()).is_set(),'reconfiguring':getattr(self,'reconfiguring',False),'message':message,'test':self.pressure.snapshot() if getattr(self,'pressure',None) else {}}
  for mode,s in d.get('tasks',{}).items():
   s['age_s']=max(0,time.monotonic()-s['updated']) if s.get('updated') else None
   if not s.get('log_path') and s.get('pid'):
    # Compatibility with supervisors kept alive to retain DMA memory.
    paths=list((ROOT/'runtime').glob(mode+'_*.log'))
    if paths:s['log_path']=str(max(paths,key=lambda p:p.stat().st_mtime))
  return d
 def operation_event(self,message):
  if not hasattr(self,'operation_events'):self.operation_events=deque(maxlen=30)
  row=dict(time=time.strftime('%H:%M:%S'),message=message,id=time.time_ns())
  self.operation_events.append(row)
  directory=ROOT/'runtime'
  if directory.exists():
   with (directory/'pipeline_events.jsonl').open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
 def operation_failed(self,exc,stage):
  self.operation_error=dict(message=str(exc) or type(exc).__name__,stage=stage,
                            log_path=str(ROOT/'runtime/pipeline_events.jsonl'))
  self.message='操作失败：'+stage+'：'+self.operation_error['message']
  self.operation_event(self.message)
  directory=ROOT/'runtime'
  if directory.exists():
   path=directory/'pipeline_error.json';temp=path.with_suffix('.tmp')
   temp.write_text(json.dumps(self.operation_error,ensure_ascii=False));temp.replace(path)
 def test_command(self,action,config):
  if JETSON:return peer_request('/api/peer/test',self.token,{'action':action,'config':config})
  if action=='stop':return self.pressure.stop()
  if action!='start':raise ValueError('无效测试操作')
  # Serialize submission with pipeline state, but do not hold the pipeline lock
  # while the worker starts: the worker must acquire it for its own launch.
  with self.operation_state:return self.pressure.start(config)
 def wait_capture_frame(self,node,timeout=40,prepared=False):
  # A live process is not proof of a working camera/DMA pipeline.
  deadline=time.monotonic()+timeout
  while True:
   if getattr(self,'stop_requested',threading.Event()).is_set():raise PipelineStopRequested()
   state=node('jetson','status','capture').get('tasks',{}).get('capture',{})
   if state.get('safety_hold'):raise RuntimeError('采集 DMA 安全保持，保留进程和内存')
   if state.get('last_error'):raise RuntimeError('采集失败：'+state['last_error'])
   if not state.get('running'):raise RuntimeError('采集进程已退出，未取得第一帧；查看发送端采集日志')
   if (state.get('ddr_ready') if prepared else state.get('frames',0)>0):return
   if time.monotonic()>=deadline:raise RuntimeError('等待第一帧超时，采集进程仍保留；请检查相机、ISP→PCIe 直写和主机缓冲')
   time.sleep(.25)
 def wait_receive_frame(self,node,timeout=20):
  deadline=time.monotonic()+timeout
  while True:
   if getattr(self,'stop_requested',threading.Event()).is_set():raise PipelineStopRequested()
   tx=node('jetson','status','sender').get('tasks',{}).get('sender',{})
   rx=node('mlx','status','receiver').get('tasks',{}).get('receiver',{})
   for name,state in [('发送',tx),('接收',rx)]:
    if state.get('safety_hold'):raise RuntimeError(name+' DMA 安全保持，保留进程和内存')
    if state.get('last_error'):raise RuntimeError(name+'失败：'+state['last_error'])
    if not state.get('running'):raise RuntimeError(name+'进程已退出，RDMA 首帧未完成')
   if rx.get('received',0)>0:return
   if time.monotonic()>=deadline:raise RuntimeError('相机已出图，但 RDMA 首帧超时；查看两端传输日志')
   time.sleep(.25)
 def launch(self,action,config,_test=False):
  if JETSON:return peer_request('/api/peer/pipeline',self.token,{'action':action,'config':config})
  if action not in ('start','stop','reconfigure'):raise ValueError('未知操作')
  requested=dict(config)
  if action in ('start','reconfigure'):
   config=validate_capture(config)
   if _test and requested.get('pressure_pattern'):
    config.update(pressure_pattern=True,
      pressure_dma=requested.get('pressure_dma') is True,
      pressure_rdma=requested.get('pressure_rdma') is True)
  if not _test and getattr(self,'pressure',None) and self.pressure.active:
   if action=='stop':return self.pressure.stop()
   raise ValueError('压力测试占用设备，请使用停止测试按钮')
  if not hasattr(self,'operation_state'):self.operation_state=threading.RLock()
  if not hasattr(self,'stop_requested'):self.stop_requested=threading.Event()
  with self.operation_state:
   if action=='stop' and getattr(self,'busy',False):
    if getattr(self,'operation_action','')!='stop':
     self.stop_requested.set();self.message='已请求安全停止：等待当前控制步骤结束，再排空并停止两端'
     self.operation_event(self.message)
    return {'message':self.message}
   if not self.operation.acquire(False):raise ValueError('已有采集操作正在执行')
   self.stop_requested.clear();self.operation_action=action
   self.operation_error={}
   (ROOT/'runtime/pipeline_error.json').unlink(missing_ok=True)
   self.operation_event('请求 '+action+'，正在执行采集链路操作')
   self.reconfiguring=action=='reconfigure'
   self.busy=True;self.message='正在'+('启动采集链路' if action=='start' else '安全停止采集链路')
   if self.reconfiguring:self.message='正在自动重配：视频暂停，安全排空后恢复'
  def work():
   nonlocal config
   stage='采集参数预检查';cancel_handled=False
   def check_stop():
    if action!='stop' and self.stop_requested.is_set():raise PipelineStopRequested()
   def node(side,op,mode):
    nonlocal stage
    stage=('Jetson' if side=='jetson' else 'Mellanox')+' / '+op+' / '+mode
    d={'op':op,'mode':mode,'config':config}
    if (side=='jetson')==JETSON:return control(d)
    return peer_request('/api/local/control',self.token,d,timeout=75 if op=='stop' else 45)
   def safe_stop():
    # Sender drain failure must not release receiver MR or capture DMA.
    for side,mode in [('jetson','sender'),('mlx','receiver'),('jetson','yolo'),('jetson','capture')]:node(side,'stop',mode)
    self.message='采集与 RDMA 已安全停止';self.operation_event(self.message)
   def finish_locked():
    self.reconfiguring=False;self.busy=False;self.operation_action=''
    self.stop_requested.clear();self.operation.release()
   try:
    check_stop()
    if action=='reconfigure':
     remote=node('jetson','status','capture');local=node('mlx','status','receiver');tasks=remote.get('tasks',{})
     for task in (tasks.get('capture',{}),tasks.get('sender',{}),local.get('tasks',{}).get('receiver',{})):
      if task.get('external_pid') or task.get('safety_hold'):raise ValueError('外部任务或 DMA 安全保持期间不能自动重配')
      if not task.get('running'):raise ValueError('自动重配需要两端链路正在运行')
     active_cfg=tasks['capture'].get('config',{})
     merged={**active_cfg,**requested}
     # Keep a user-selected exposure when changing geometry. On an FPS increase,
     # clamp an inherited exposure to the new frame period before validation.
     if 'fps' in requested and 'exposure_us' not in requested and config['fps']!=active_cfg.get('fps'):
      merged['exposure_us']=min(int(active_cfg.get('exposure_us',7500)),automatic_exposure_limit_us(config['fps']))
     config=validate_capture(merged)
     from video_camera import check_geometry
     check_geometry(config,remote.get('camera_readback',{}))
     self.message='正在排空 RDMA：画面暂时冻结，参数尚未生效'
     safe_stop();check_stop()
     self.message='已安全停采，正在更新参数并恢复'
    if action in ('start','reconfigure'):
     config=dict(config);config['_session']=secrets.token_hex(12)
     node('jetson','prepare','capture');check_stop()
     pressure_pattern=config.get('pressure_pattern',False)
     use_rdma=not pressure_pattern or config.get('pressure_rdma',False)
     for side,mode in [('mlx','receiver'),('jetson','capture')]:
      check_stop();node(side,'start',mode);check_stop()
     direct=config.get('transport')=='ddr' and use_rdma
     self.message='正在准备彩条 DDR 测试数据' if pressure_pattern else ('正在准备 FPGA DDR/RDMA 与独立 ISP→PCIe 本地缓冲' if direct else '正在等待 ISP→PCIe 首帧')
     stage='Jetson / DDR 与主机首帧准备'
     self.wait_capture_frame(node,prepared=direct);check_stop()
     if use_rdma:
      node('jetson','start','sender');check_stop()
      if direct:self.wait_capture_frame(node);check_stop()
      self.message='彩条已加载，正在等待 RDMA 首帧' if pressure_pattern else '相机已出图，正在等待 RDMA 首帧'
      stage='Mellanox / RDMA 首帧确认'
      self.wait_receive_frame(node);check_stop()
     if config.get('yolo'):
      node('jetson','start','yolo');check_stop()
     self.message='彩条 DMA/RDMA 压力测试已启动' if pressure_pattern else ('参数已应用，采集与 RDMA 已恢复' if action=='reconfigure' else '采集与 RDMA 传输已启动')
     self.operation_event(self.message)
    elif action=='stop':safe_stop()
   except PipelineStopRequested:
    cancel_handled=True
    try:safe_stop()
    except Exception as exc:self.operation_failed(exc,stage)
   except Exception as exc:self.operation_failed(exc,stage)
   finally:
    # Accepting stop and finishing share a lock, so a late stop cannot be lost.
    with self.operation_state:
     late_stop=action!='stop' and not cancel_handled and self.stop_requested.is_set()
     if not late_stop:finish_locked()
    if late_stop:
     try:safe_stop()
     except Exception as exc:self.operation_failed(exc,stage)
     finally:
      with self.operation_state:finish_locked()
  threading.Thread(target=work,daemon=True).start()
  return {'message':self.message}
 def image(self):
  with self.lock:
   if not self.raw or time.monotonic()-self.raw[3]>2:raise ValueError('当前没有新鲜完整帧')
   seq,(pixels,w,h,fmt),rev,_=self.raw
  return seq,rev,to_image(pixels,w,h,fmt)
 def save(self):
  with self.lock:
   if not self.raw or time.monotonic()-self.raw[3]>2:raise ValueError('当前没有新鲜完整帧')
   seq,(pixels,w,h,fmt),rev,_=self.raw
  im=to_image(pixels,w,h,fmt)
  name='snapshot_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(rev)+'_'+str(seq)+'.png'
  path=ROOT/'captures'/name
  if shutil.disk_usage(ROOT).free<128*1024**2:raise ValueError('磁盘空间不足')
  im.save(path)
  if fmt==BAYER:
   path.with_suffix('.bayer8').write_bytes(pixels)
   path.with_suffix('.json').write_text(json.dumps(dict(format='BayerRG8',width=w,height=h,seq=seq,preview='PNG is demosaiced'),ensure_ascii=False))
  return {'name':name,'url':'/captures/'+name,'seq':seq,'width':im.width,'height':im.height,'message':('已保存原始 Bayer .bayer8 与去马赛克预览 PNG' if fmt==BAYER else '已保存原始 RGB 无损 PNG')}
 def analysis(self,name,x,y):
  if not re.fullmatch(r'(snapshot_|desktop_raw_)[0-9_]+\.png',name):raise ValueError('请先冻结或抓取单帧')
  path=ROOT/'captures'/name
  with Image.open(path) as src:im=src.convert('RGB')
  x=max(0,min(im.width-1,int(x)));y=max(0,min(im.height-1,int(y)))
  gray=im.convert('L');hist=gray.histogram();n=im.width*im.height
  stats=ImageStat.Stat(gray);rgb=im.getpixel((x,y))
  return dict(seq=name.split('_')[-1].split('.')[0],width=im.width,height=im.height,x=x,y=y,rgb=rgb,
   mean=stats.mean[0],stddev=stats.stddev[0],minimum=gray.getextrema()[0],maximum=gray.getextrema()[1],
   saturation_pct=100*hist[255]/n,black_pct=100*hist[0]/n,histogram=hist,
   horizontal=list(gray.crop((0,y,im.width,y+1)).getdata()),vertical=list(gray.crop((x,0,x+1,im.height)).getdata()))
 def config(self,d):
  limit=int(d.get('history',self.history_limit));fps=int(d.get('record_fps',self.record_fps));preview=int(d.get('preview_fps',self.frames.args.fps))
  if not 1<=preview<=120:raise ValueError('后台预览帧率范围 1–120')
  if not 1<=limit<=64 or not 1<=fps<=10:raise ValueError('缓存 1–64 帧，连续保存 1–10 fps')
  with self.lock:
   self.history_limit=limit;self.record_fps=fps;self.frames.args.fps=preview
   while len(self.history)>limit:self.history.popleft()
  return {'message':'预览缓存和保存参数已应用'}
 def record(self,start):
  with self.lock:
   if start:
    if self.recording:return {'message':'正在保存'}
    self.image()
    if self.save_queue:raise ValueError('上一轮保存正在收尾')
    self.record_dir=ROOT/'captures'/('sequence_'+time.strftime('%Y%m%d_%H%M%S')+'_'+secrets.token_hex(2));self.record_dir.mkdir()
    self.record_count=0;self.record_bytes=0;self.record_drops=0;self.record_error='';self.recording=True
   else:self.recording=False
  return {'message':'开始采样保存 JPEG 序列' if start else '停止连续保存'}

 def saved(self):
  paths=list((ROOT/'captures').glob('snapshot_*.png'))+list((ROOT/'captures').glob('preview_*.jpg'))+list((ROOT/'captures').glob('desktop_raw_*.png'))+list((ROOT/'captures').glob('desktop_preview_*.jpg'))
  for directory in sorted((ROOT/'captures').glob('sequence_*'),reverse=True)[:8]:
   paths.extend(directory.glob('frame_*.jpg'))
  paths=sorted(paths,key=lambda p:p.stat().st_mtime,reverse=True)[:40]
  return {'files':[{'name':p.name,'url':'/'+str(p.relative_to(ROOT)),'bytes':p.stat().st_size,'raw':p.suffix=='.png'} for p in paths]}
 def save_history(self,ident):
  with self.lock:
   item=next((dict(x) for x in self.history if x['id']==int(ident)),None)
  if item is None:raise ValueError('该缓存帧已过期，请选择新的缓存帧')
  if shutil.disk_usage(ROOT).free<128*1024**2:raise ValueError('磁盘空间不足')
  name='preview_%d_%d.jpg'%(item['seq'],item['id']);(ROOT/'captures'/name).write_bytes(item['jpeg'])
  return {'url':'/captures/'+name,'name':name,'seq':item['seq']}
