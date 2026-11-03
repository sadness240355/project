"""Mellanox-owned camera/RDMA stress run with persistent reports and safe drain."""
import json,time,threading,secrets,csv
from pathlib import Path
from video_config import validate_capture,frame_bytes
ACTIVE=('PREPARING','RUNNING','STOPPING')
def settings(data):
 for key in ('seconds','fps','buffers','queues'):
  if key in data and (isinstance(data[key],bool) or str(data[key]).strip()!=str(int(data[key]))):raise ValueError('测试参数必须为整数')
 out=dict(seconds=int(data.get('seconds',60)),fps=int(data.get('fps',120)),buffers=int(data.get('buffers',16)),queues=int(data.get('queues',8)),strict_no_drop=data.get('strict_no_drop',False))
 if not 1<=out['seconds']<=7200 or out['fps'] not in (30,60,120,140) or not 2<=out['buffers']<=64 or not 1<=out['queues']<=8 or out['queues']>out['buffers'] or type(out['strict_no_drop']) is not bool:raise ValueError('时长1–7200秒，fps30/60/120/140，缓冲2–64，队列1–8且不超过缓冲')
 if data.get('pressure_pattern'):
  if type(data.get('pressure_dma')) is not bool or type(data.get('pressure_rdma')) is not bool or not (data['pressure_dma'] or data['pressure_rdma']):raise ValueError('彩条压力测试须至少选择 DMA 或 RDMA')
  clean={**data,'source':'camera','test_pattern':0,'transport':'ddr','format':'rgb','crc_mode':'transport','yolo':False,'fps':30}
  clean.pop('preprocess',None)
  return {**validate_capture(clean),**out,'fps':30,'pressure_pattern':True,
          'pressure_dma':data['pressure_dma'],'pressure_rdma':data['pressure_rdma']}
 return {**validate_capture(data),**out}
def counters(c):
 cap=c.get('capture',{});s=c.get('sender',{});r=c.get('receiver',{})
 pattern=cap.get('config',{})
 if pattern.get('pressure_pattern') and not pattern.get('pressure_rdma'):
  s={};r={}
 return dict(received=r.get('received',0),submitted=s.get('submitted',0),
  dma_bytes_total=cap.get('dma_bytes_total',0),rdma_bytes_total=r.get('received',0)*r.get('bytes',0),
  source_skipped=s.get('source_skipped',0),skip_host=cap.get('skip_host',0),skip_ddr=cap.get('skip_ddr',0),
  fpga_drop=cap.get('drops',0),input_bad=cap.get('diag',{}).get('bad',0),
  crc_errors=r.get('crc_errors',0),faults=cap.get('faults',0)+s.get('faults',0)+r.get('faults',0))
def final_check(c,strict=False,baseline=None,pattern=None):
 if pattern:
  enabled=('capture','sender','receiver') if pattern['pressure_rdma'] else ('capture',)
  if any(c.get(k,{}).get('running') for k in enabled):return False,'测试进程尚未安全停止'
  cap=c.get('capture',{});s=c.get('sender',{});r=c.get('receiver',{})
  if not cap.get('passed'):return False,'彩条 DDR 源缺少最终 PASS'
  if pattern['pressure_dma'] and not cap.get('dma_bytes_total'):return False,'DMA 未完成有效测试数据'
  if pattern['pressure_rdma']:
   session=cap.get('config',{}).get('session')
   if not session or session!=s.get('config',{}).get('session') or session!=r.get('config',{}).get('session'):
    return False,'双端会话标识不一致'
   if not s.get('passed') or not r.get('passed') or not s.get('submitted') or s['submitted']!=r.get('received'):
    return False,'RDMA 提交/接收计数不一致或缺少最终 PASS'
  n=counters(c)
  if n['faults'] or n['crc_errors']:return False,'存在 DMA/RDMA 错误或彩条采样 CRC 错误'
  return True,'彩条 DMA/RDMA 完成计数与抽样校验通过'
 s=c.get('sender',{});r=c.get('receiver',{});session=s.get('config',{}).get('session')
 if any(c.get(k,{}).get('running') for k in ('capture','sender','receiver')):return False,'进程尚未完成排空'
 if not session or session!=r.get('config',{}).get('session'):return False,'最终会话标识不一致'
 if not s.get('passed') or not r.get('passed'):return False,'缺少双方最终 PASS'
 if not s.get('submitted') or s['submitted']!=r.get('received'):return False,'最终提交/接收数量不一致或为零'
 n=counters(c);base=baseline or {}
 if n['crc_errors'] or n['faults']:return False,'存在 CRC 或传输失败'
 if strict and any(n[k]>base.get(k,0) for k in ('source_skipped','skip_host','skip_ddr','fpga_drop','input_bad')):return False,'严格模式：出现采集丢帧或主动跳帧'
 return True,'传输完整性通过' if not strict else '完整性及零跳帧检查通过'
class PressureRunner:
 def __init__(self,root,status,launch):
  self.root=Path(root);self.get_status=status;self.launch=launch;self.lock=threading.RLock();self.cancel=threading.Event()
  self.root.mkdir(exist_ok=True);self.last=self.root/'last_test.json';self.data=dict(state='IDLE',message='尚未开始压力测试',progress=0)
  if self.last.exists():
   try:self.data=json.loads(self.last.read_text())
   except (ValueError,OSError):pass
  if self.data.get('directory') and (not Path(self.data['directory']).resolve().is_relative_to(self.root.resolve()) or not Path(self.data['directory']).is_dir()):self.data.pop('directory',None)
  if self.data.get('state') in ACTIVE:
   self.data.update(state='INTERRUPTED',message='后台服务曾中断；未自动续跑，原硬件进程需单独核对');self.persist()
 @property
 def active(self):
  with self.lock:return self.data.get('state') in ACTIVE
 def snapshot(self):
  with self.lock:return json.loads(json.dumps(self.data))
 def persist(self):
  raw=json.dumps(self.data,ensure_ascii=False,indent=2);tmp=self.last.with_suffix('.tmp');tmp.write_text(raw);tmp.replace(self.last)
  if self.data.get('directory'):(Path(self.data['directory'])/'result.json').write_text(raw)
 def update(self,**data):
  with self.lock:self.data.update(data);self.persist()
 def preflight(self,cfg=None):
  c=self.get_status()
  if c.get('busy'):raise ValueError('普通采集操作尚未完成')
  if not c.get('controller') or not c.get('peer_controller'):raise ValueError('双端控制服务未就绪')
  s=c.get('sender',{})
  if s.get('running') and s.get('safety_hold'):raise ValueError('发送端 DMA 安全保持；先完成 FPGA 安全复位和旧会话恢复')
  if any(c.get(k,{}).get('running') or c.get(k,{}).get('external_pid') for k in ('capture','sender','receiver')):raise ValueError('请先正常停止当前采集/传输，再启动独立压力测试')
  if (not cfg or not cfg.get('pressure_pattern') or cfg.get('pressure_rdma')) and c.get('network',{}).get('state')!='UP':raise ValueError('Mellanox 网口未 UP')
  return c
 def start(self,config):
  cfg=settings(config)
  with self.lock:
   if self.active:raise ValueError('已有压力测试运行中')
   self.preflight(cfg);folder=self.root/('rdma_'+time.strftime('%Y%m%d_%H%M%S')+'_'+secrets.token_hex(3));folder.mkdir()
   self.cancel.clear();self.data=dict(state='PREPARING',message='准备独立测试会话',config=cfg,directory=str(folder),elapsed_s=0,remaining_s=cfg['seconds'],progress=0)
   self.persist();threading.Thread(target=self.run,args=(cfg,),daemon=True).start()
  return {'message':'压力测试已提交'}
 def stop(self):
  self.cancel.set();return {'message':'已请求停止测试，等待安全排空；不会强杀 DMA 进程'}
 def wait_state(self,predicate,timeout,fail_on_operation_error=False):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   c=self.get_status()
   if predicate(c):return c
   if fail_on_operation_error and c.get('operation_error'):
    error=c['operation_error']
    raise RuntimeError('启动失败：'+str(error.get('stage','采集链路'))+'：'+str(error.get('message','未知错误')))
   for mode in ('capture','sender','receiver'):
    if c.get(mode,{}).get('safety_hold') and c.get(mode,{}).get('running'):
     raise RuntimeError(mode+' 进入 DMA 安全保持')
   time.sleep(.25)
  raise RuntimeError('等待双端状态超时：'+self.get_status().get('message',''))
 def run(self,cfg):
  launched=False;failure=None;baseline={}
  pattern=cfg.get('pressure_pattern',False)
  use_dma=pattern and cfg['pressure_dma']
  use_rdma=not pattern or cfg['pressure_rdma']
  required=('capture','sender','receiver') if use_rdma else ('capture',)
  try:
   if self.cancel.is_set():self.update(state='CANCELLED',message='启动前取消');return
   self.launch('start',cfg);launched=True
   c=self.wait_state(lambda c:all(c.get(k,{}).get('running') for k in required) and
     (c.get('receiver',{}).get('received',0)>0 if use_rdma else c.get('capture',{}).get('dma_bytes_total',0)>0),
     45,fail_on_operation_error=True)
   session=c['capture'].get('config',{}).get('session') if pattern else c['sender'].get('config',{}).get('session')
   if not session or (use_rdma and session!=c['receiver'].get('config',{}).get('session')):raise RuntimeError('测试会话未对齐')
   baseline=counters(c);begin=time.monotonic();last_count=baseline['received'];last_at=begin
   last_dma=baseline['dma_bytes_total'];last_rdma=baseline['rdma_bytes_total']
   self.update(state='RUNNING',session=session,baseline=baseline,
     message='固定彩条独立 DMA/RDMA 测速中；每秒报告完成字节' if pattern else '完整帧 CRC 与持续传输监测中')
   with (Path(self.data['directory'])/'samples.csv').open('w',newline='') as f:
    fields=['elapsed_s','fps','payload_MiB_s','dma_MiB_s','rdma_MiB_s',*baseline.keys()];writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
    while True:
     now=time.monotonic();elapsed=now-begin;c=self.get_status();n=counters(c)
     if any(not c.get(k,{}).get('running') for k in required):raise RuntimeError('测试期间所选通道进程退出')
     if c['capture'].get('config',{}).get('session')!=session or (use_rdma and c['receiver'].get('config',{}).get('session')!=session):raise RuntimeError('测试会话被替换')
     if any(c.get(k,{}).get('safety_hold') for k in required) or n['faults'] or n['crc_errors']:raise RuntimeError('测试通道失败或彩条抽样 CRC 错帧')
     if use_rdma and c['receiver'].get('age_s',0)>5:raise RuntimeError('接收统计超过5秒未更新')
     rate=max(0,n['received']-last_count)/max(.001,now-last_at)
     dma_rate=max(0,n['dma_bytes_total']-last_dma)/max(.001,now-last_at)/1048576 if use_dma else 0
     rdma_rate=max(0,n['rdma_bytes_total']-last_rdma)/max(.001,now-last_at)/1048576 if use_rdma else 0
     row=dict(elapsed_s=round(elapsed,3),fps=round(rate,3),payload_MiB_s=round(rate*frame_bytes(cfg)/1048576,3),
       dma_MiB_s=round(dma_rate,3),rdma_MiB_s=round(rdma_rate,3),**n);writer.writerow(row);f.flush()
     avg_dma=max(0,n['dma_bytes_total']-baseline['dma_bytes_total'])/max(.001,elapsed)/1048576 if use_dma else 0
     avg_rdma=max(0,n['rdma_bytes_total']-baseline['rdma_bytes_total'])/max(.001,elapsed)/1048576 if use_rdma else 0
     self.update(elapsed_s=round(elapsed,1),remaining_s=max(0,round(cfg['seconds']-elapsed,1)),progress=min(99,elapsed/cfg['seconds']*100),metrics=n,
       measured_fps=max(0,n['received']-baseline['received'])/max(.001,elapsed),dma_MiB_s=dma_rate,rdma_MiB_s=rdma_rate,
       avg_dma_MiB_s=avg_dma,avg_rdma_MiB_s=avg_rdma,
       peak_dma_MiB_s=max(self.data.get('peak_dma_MiB_s',0),dma_rate),peak_rdma_MiB_s=max(self.data.get('peak_rdma_MiB_s',0),rdma_rate))
     last_count=n['received'];last_dma=n['dma_bytes_total'];last_rdma=n['rdma_bytes_total'];last_at=now
     if self.cancel.is_set() or elapsed>=cfg['seconds']:break
     self.cancel.wait(min(1,cfg['seconds']-elapsed))
  except Exception as e:failure=str(e)
  if launched:
   self.update(state='STOPPING',message='排空已提交帧并核对双方最终计数')
   try:
    self.wait_state(lambda c:not c.get('busy'),45);self.launch('stop',{})
    c=self.wait_state(lambda c:not c.get('busy') and all(not c.get(k,{}).get('running') for k in required),50)
    ok,reason=final_check(c,cfg['strict_no_drop'],baseline,cfg if pattern else None)
    self.update(final=c,metrics=counters(c))
    if not ok:failure=(failure+'; ' if failure else '')+reason
   except Exception as e:failure=(failure+'; ' if failure else '')+str(e)
  if failure:self.update(state='FAIL',message=failure)
  elif self.cancel.is_set():self.update(state='CANCELLED',message='用户停止；不作为完整时长验收通过')
  else:self.update(state='PASS',progress=100,message='彩条 DMA/RDMA 完成计数与抽样校验通过' if pattern else ('传输完整性通过；各阶段跳帧另列' if not cfg['strict_no_drop'] else '完整性与零跳帧检查通过'))
