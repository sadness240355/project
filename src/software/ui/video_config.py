"""Validated camera/transport settings shared by both controllers and GUI."""
import math
DEFAULT_CAPTURE=dict(width=1920,height=1080,fps=120,buffers=16,queues=8,format='rgb',exposure_us=7500,black_level=0,wb_r=1.01,wb_g=1.04,wb_b=.95)
MAX_WIDTH,MAX_HEIGHT,MAX_FPS=2400,2048,140
RESOLUTIONS=((2400,2048),(1920,1080),(1280,720),(640,480))
def available_resolutions(readback=None):
 """Presets for the capture path, which always uses the 10-tap geometry.

 The sensor may report 2432 in its power-on 2-tap mode. That is not a usable
 width in the 10-tap mode configured by bayerrg8_app.sh; keep menus stable.
 """
 values=(readback or {}).get('values',{})
 try:
  mw,mh=int(values['WidthMax']),int(values['HeightMax'])
  if mw<=0 or mh<=0:raise ValueError('invalid capability limit')
 except (KeyError,TypeError,ValueError):
  # No capability readback yet: use only presets supported by this path.
  return RESOLUTIONS
 return tuple(p for p in RESOLUTIONS if p[0]<=mw and p[1]<=mh)

FRAME_RATES=(30,60,120,140)
DEFAULT_CCM=[1.2157,-.0329,-.1828,-.1984,1.3814,-.1830,-.0048,-.3104,1.3151]
STREAMS={'FPGA ISP · RGB888':'rgb','摄像头原始 · BayerRG8':'bayer'}
# Current Camera Link camera needs headroom beyond exposure itself.
# A full frame-period exposure gave ~138 fps at a 140 fps request; 6500 us
# restored 140 fps on the board. Reserve 200 us for automatic choices.
# This is a conservative recommendation, not a universal sensor maximum.
AUTO_EXPOSURE_HEADROOM_US=200
def automatic_exposure_limit_us(fps):
 return max(1,1000000//int(fps)-AUTO_EXPOSURE_HEADROOM_US)

def validate_capture(data):
 out=dict(DEFAULT_CAPTURE)
 if data.get('transport','ddr')!='ddr':raise ValueError('普通视频仅支持 FPGA DDR→ERNIC RDMA；已移除 DDR→主机视频转发模式')
 out['transport']=data.get('transport','ddr')
 mode=data.get('crc_mode','full')
 if mode not in ('full','transport'):raise ValueError('校验模式必须为 full 或 transport')
 if mode=='transport' and out['transport']!='ddr':raise ValueError('低延迟链路校验仅支持 DDR 直传')
 out['crc_mode']=mode
 if 'fps' in data and 'exposure_us' not in data:
  try:out['exposure_us']=min(out['exposure_us'],automatic_exposure_limit_us(data['fps']))
  except (ValueError,TypeError,ZeroDivisionError):raise ValueError('无效帧率')
 if data.get('source','camera') not in ('camera','camera_pattern'):raise ValueError('当前 ERNIC 仅支持 Camera Link 相机或相机内置测试图')
 pattern=data.get('test_pattern',0)
 if type(pattern) is not int or pattern not in (0,11,12,13):raise ValueError('不支持的相机测试图')
 if data.get('source','camera')=='camera' and pattern:raise ValueError('真实相机数据源必须关闭测试图')
 if data.get('source')=='camera_pattern' and not pattern:raise ValueError('请选择相机内置测试图')
 if 'source' in data:out['source']=data['source']
 if 'test_pattern' in data:out['test_pattern']=pattern
 for k in tuple(DEFAULT_CAPTURE):
  if k in data:out[k]=data[k]
 if 'buffers' in data and 'queues' not in data:
  try:out['queues']=min(8,int(out['buffers']))
  except (ValueError,TypeError):raise ValueError('buffers 必须为整数')
 for k in ('width','height','fps','buffers','queues','exposure_us','black_level'):
  v=out[k]
  if isinstance(v,bool) or str(v).strip()!=str(int(v)):raise ValueError(k+' 必须为整数')
  out[k]=int(v)
 if (out['width'],out['height']) not in RESOLUTIONS:raise ValueError('不支持的采集分辨率')
 if out['fps'] not in FRAME_RATES or out['buffers'] not in (2,4,8,16,32) or not 1<=out['queues']<=8 or out['queues']>out['buffers']:raise ValueError('采集fps30/60/120/140，直写缓冲2/4/8/16/32，兼容队列参数1–8且不超过缓冲')
 if out['format'] not in ('rgb','bayer'):raise ValueError('数据流必须为 RGB888 或 BayerRG8')
 if out['format']=='rgb' and (out['width'],out['height'],out['fps'])==(2400,2048,140) and out['buffers']<16:
  raise ValueError('2400×2048、140 fps 的 RGB 直写至少需要 16 个主机缓冲；8 个缓冲曾导致主机帧缺口和 DDR 环停滞')
 if not 1<=out['exposure_us']<=1000000//out['fps']:raise ValueError('曝光时间必须大于0且不超过帧周期')
 if not 0<=out['black_level']<=255:raise ValueError('黑电平范围0–255')
 for k in ('wb_r','wb_g','wb_b'):
  out[k]=float(out[k])
  if not math.isfinite(out[k]) or not .25<=out[k]<=4:raise ValueError('白平衡增益范围0.25–4')
 if 'isp_enable' in data:
  v=data['isp_enable']
  if type(v) is not int or not 0<=v<=15:raise ValueError('ISP mask范围0–15')
  out['isp_enable']=v
 if 'ccm' in data:
  if not isinstance(data['ccm'],(list,tuple)) or len(data['ccm'])!=9:raise ValueError('CCM需要9个系数')
  out['ccm']=[float(v) for v in data['ccm']]
  if any(not math.isfinite(v) or not -8<=v<=32767/4096 for v in out['ccm']):raise ValueError('CCM Q12系数范围 -8..7.99975')
 if 'preprocess' in data:
  from uip_profile import profile,DEFAULTS
  p=profile(data['preprocess'])
  if out['format']=='bayer' and p['enable']:raise ValueError('摄像头原始 Bayer 数据流必须旁路全部 ISP')
  out['preprocess']={k:p[k] for k in DEFAULTS}
 if type(data.get('yolo',False)) is not bool:raise ValueError('YOLO 开关必须为布尔值')
 out['yolo']=data.get('yolo',False)
 if out['yolo'] and out['format']!='rgb':raise ValueError('当前 YOLO 输入使用 FPGA RGB；Bayer 原始模式请关闭 YOLO')
 return out
def frame_bytes(cfg):
 return int(cfg.get('width',1920))*int(cfg.get('height',1080))*(1 if cfg.get('format')=='bayer' else 3)

def resolution_pair(label):
 """One preset controls both dimensions; mixed or unsupported pairs are rejected."""
 try:w,h=map(int,str(label).lower().replace('×','x').split('x'))
 except (ValueError,TypeError):raise ValueError('请选择完整分辨率组合，例如 1920×1080')
 if (w,h) not in RESOLUTIONS:raise ValueError('不支持的采集分辨率组合')
 return w,h
