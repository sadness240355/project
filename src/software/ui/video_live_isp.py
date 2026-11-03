"""Atomic base-ISP updates. Requires LISP ABI; never resets capture or RDMA."""
import math,struct,time
from video_preprocess import registers
BASE=0x14100
MAGIC=0x4c495350
MIN_LIVE_BUILD=0x00030018

def read(m,o):return struct.unpack_from('<I',m,BASE+o)[0]
def write(m,o,v):struct.pack_into('<I',m,BASE+o,v&0xffffffff)
def status_from(m):
 if read(m,0)!=MAGIC or read(m,4)!=0x10000:return {'available':False}
 build=struct.unpack_from('<I',m,0x14070)[0]
 if build<MIN_LIVE_BUILD:return dict(available=False,build_version=build,reason='在线 ISP 需要已修复译码和写使能的 0x30018 或更新位流')
 s=read(m,12)
 return dict(available=True,build_version=build,capabilities=read(m,8),busy=bool(s&1),error=bool(s&2),allowed=bool(s&4),generation=read(m,20),isp_enable=read(m,24))
def status():
 try:
  with registers() as m:return status_from(m)
 except OSError as e:return dict(available=False,error=str(e))
def encode(values):
 keys={'isp_enable','black_level','wb_r','wb_g','wb_b','ccm'}
 if set(values)!=keys:raise ValueError('在线 ISP 仅接受整组开关、黑电平、白平衡和 CCM 参数')
 mask=values['isp_enable'];black=values['black_level']
 if type(mask) is not int or not 0<=mask<=15 or type(black) is not int or not 0<=black<=255:raise ValueError('ISP 开关或黑电平范围错误')
 gains=[float(values[k]) for k in ('wb_r','wb_g','wb_b')]
 ccm=values['ccm']
 if not isinstance(ccm,(list,tuple)) or len(ccm)!=9:raise ValueError('CCM 需要 9 个系数')
 ccm=[float(v) for v in ccm]
 if any(not math.isfinite(v) or not 0<=v<=65535/4096 for v in gains):raise ValueError('白平衡范围 0..15.99975')
 if any(not math.isfinite(v) or not -8<=v<=32767/4096 for v in ccm):raise ValueError('CCM 范围 -8..7.99975')
 return [mask,black]+[round(v*4096) for v in gains]+[round(v*4096)&0xffff for v in ccm]
def apply(values,timeout=1.0):
 fields=encode(values)
 with registers(True) as m:
  before=status_from(m)
  if not before['available']:raise ValueError('当前 bit 不支持在线 ISP，需要加载修复后的 0x30018 或更新版本')
  if not before['allowed'] or before['busy']:raise ValueError('在线 ISP 需要 RGB 采集运行且前一提交已完成')
  for i,value in enumerate(fields):write(m,0x40+4*i,value)
  if any(read(m,0x40+4*i)!=value for i,value in enumerate(fields)):raise RuntimeError('在线 ISP 参数回读失败，未提交')
  write(m,16,1)
  deadline=time.monotonic()+timeout
  while True:
   current=status_from(m)
   if current.get('error'):raise RuntimeError('硬件拒绝或中止在线 ISP 更新；保留错误状态')
   if current.get('generation')==(before['generation']+1)&0xffffffff and not current.get('busy'):
    if any(read(m,0x80+4*i)!=value for i,value in enumerate(fields)):raise RuntimeError('硬件生效参数与提交值不一致')
    return current
   if time.monotonic()>=deadline:raise RuntimeError('等待帧边界生效超时；未停止或复位采集，请检查状态')
   time.sleep(.001)
