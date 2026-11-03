"""GPU display independent of Python frame analysis; rendering FPS is not scanout FPS."""
import ctypes as C,json,os,re,subprocess
from pathlib import Path
# Must precede the first Tk window / XOpenDisplay in this process.
_x=C.CDLL('libX11.so.6');_x.XInitThreads()
ROOT=Path(__file__).resolve().parent
class Renderer:
 def __init__(self,window,path):
  self.lib=C.CDLL(str(ROOT/'libvideo_gpu.so'))
  self.lib.video_gpu_start.argtypes=[C.c_ulong,C.c_char_p];self.lib.video_gpu_start.restype=C.c_void_p
  self.lib.video_gpu_config.argtypes=[C.c_void_p,C.c_int,C.c_int,C.c_int,C.c_double,C.c_double,C.c_double,C.c_double]
  self.lib.video_gpu_status.argtypes=[C.c_void_p,C.c_char_p,C.c_size_t]
  self.lib.video_gpu_stop.argtypes=[C.c_void_p];self.lib.video_gpu_probe.argtypes=[C.c_void_p,C.c_int]
  self.lib.video_gpu_boxes.argtypes=[C.c_void_p,C.c_char_p,C.c_int]
  self.handle=self.lib.video_gpu_start(window,os.fsencode(path))
  if not self.handle:raise RuntimeError('GPU renderer startup failed')
 def configure(self,w,h,paused=False,limit=0,zoom=0,x=0,y=0):
  self.lib.video_gpu_config(self.handle,w,h,paused,limit,zoom,x,y)
 def status(self):
  buf=C.create_string_buffer(2048);self.lib.video_gpu_status(self.handle,buf,len(buf));return json.loads(buf.value)
 def boxes(self,path='',enabled=True):self.lib.video_gpu_boxes(self.handle,os.fsencode(path),enabled)
 def probe(self,enabled):self.lib.video_gpu_probe(self.handle,enabled)
 def close(self):
  if self.handle:self.lib.video_gpu_stop(self.handle);self.handle=None
def screen_rates():
 try:
  out=subprocess.check_output(['xrandr','--current'],text=True,stderr=subprocess.DEVNULL,timeout=3)
  port='';rates=[]
  for line in out.splitlines():
   if ' connected' in line:port=line.split()[0]
   if '*' in line:
    m=re.search(r'(\d+(?:\.\d+)?)\*',line)
    if m:rates.append(port+' '+m[1]+' Hz')
  return ' / '.join(rates) or '未检测到屏幕刷新率'
 except (OSError,subprocess.SubprocessError):return '屏幕刷新率未知'

def gpu_failure_detail(error):
 try:
  result=subprocess.run(['nvidia-smi','--query-gpu=driver_version','--format=csv,noheader'],capture_output=True,text=True,timeout=3)
  if 'Driver/library version mismatch' in result.stdout+result.stderr:
   return 'NVIDIA 内核驱动与图形库版本不一致；恢复匹配驱动后重新打开窗口。'
 except (OSError,subprocess.SubprocessError):pass
 return error
