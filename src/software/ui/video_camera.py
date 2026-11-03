"""Stopped-camera identity/capability query. No start/reset/trigger commands."""
import re,subprocess,time
NODES=('DeviceModelName','DeviceVendorName','DeviceSerialNumber','WidthMax','HeightMax','Width','Height','OffsetX','OffsetY','PixelFormat','DeviceTapGeometry','AcquisitionFrameRate','ExposureTime','TriggerMode','TestPattern')
def parse_reply(text,node):
 m=re.search(r'get '+re.escape(node)+r':([^\r\n]+)',text)
 if not m:raise ValueError('相机未返回 '+node)
 value=m[1].strip()
 if value.endswith('.'):value=value[:-1]
 if not value:raise ValueError('相机返回空值 '+node)
 return value
def _query(binary,node,run,timeout):
 p=run([str(binary),'--fpga','--serial-status','--show-polarity','--cmd-timeout-ms','2000','--cmd','r '+node],capture_output=True,text=True,timeout=timeout)
 try:
  if p.returncode:raise ValueError('串口命令返回码 '+str(p.returncode))
  return parse_reply(p.stdout,node)
 except ValueError as exc:
  # Keep the actual UART/open/response failure; do not label all failures as
  # an unsupported camera. Output is bounded for the UI and HTTP response.
  detail=' '.join((str(p.stdout or '')+' '+str(p.stderr or '')).split())[-800:]
  raise ValueError(str(exc)+((': '+detail) if detail else '')) from exc
def read_camera(binary,run=subprocess.run):
 values={};errors={};attempts={}
 # Peer control has a 42 s timeout. Bound the entire capability sweep,
 # including retries, so a missing camera never strands the UI request.
 deadline=time.monotonic()+32
 for node in NODES:
  for attempt in range(3 if node=='DeviceModelName' else 1):
   remaining=deadline-time.monotonic()
   if remaining<=0:
    errors[node]='相机回读总时限已到'
    break
   attempts[node]=attempt+1
   try:
    values[node]=_query(binary,node,run,min(6,remaining))
    errors.pop(node,None)
    break
   except (ValueError,OSError,subprocess.TimeoutExpired) as exc:
    errors[node]=str(exc)
    if node=='DeviceModelName' and attempt<2:
     # Only repeat an identity read. Never reset the FPGA, start acquisition
     # or write camera parameters to recover an unresponsive serial link.
     time.sleep(.25*(attempt+1))
  if node=='DeviceModelName' and not values.get(node):
   raise ValueError('相机型号未读取（已尝试 '+str(attempts.get(node,0))+' 次）；检查相机供电与 Camera Link Base 串口连接。底层错误：'+errors.get(node,'空回复'))
 return dict(values=values,errors=errors,attempts=attempts,updated=time.time())
def check_geometry(cfg,readback):
 values=readback.get('values',{})
 for key,node in (('width','WidthMax'),('height','HeightMax')):
  try:limit=int(values[node])
  except (KeyError,ValueError):raise ValueError('未读到当前相机 '+node+'，请先读取相机能力')
  if not 0<int(cfg[key])<=limit:raise ValueError('当前相机/Camera Link 模式 '+node+'='+str(limit)+'，请求 '+str(cfg[key])+' 超限；不能只修改 FPGA 宽高')
