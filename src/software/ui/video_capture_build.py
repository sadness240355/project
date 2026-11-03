"""Read-only preflight for capture FIFO integrity; never resets hardware."""
import struct
from video_preprocess import registers
FIFO_SAFE_BUILD=0x00030020
FULL_RATE_HOST_BUILD=0x0003002d
def require_capture_core(cfg):
 with registers() as mem:
  build_id=struct.unpack_from('<I',mem,0x1406c)[0]
  version=struct.unpack_from('<I',mem,0x14070)[0]
  mode=struct.unpack_from('<I',mem,0x1403c)[0]
 if build_id==0xffffffff or version==0xffffffff:
  raise ValueError('FPGA PCIe BAR 无响应（读到 0xffffffff）。Program Device 后先确保旧 DMA 已停止，再重新枚举 PCIe 并绑定驱动；当前不能启动采集。')
 if build_id!=0x52474233:raise ValueError('未识别到 RGB3 采集核心，请检查 bitstream')
 if version==0xffffffff or version<FIFO_SAFE_BUILD:
  raise ValueError('当前 FPGA 位流版本 0x%08x 未满足连续采集的 FIFO 保护/恢复要求（最低 0x%08x）；请先下载与当前软件配套且已通过时序门禁的新版 bitstream，重新读取版本后再启动。'%(version,FIFO_SAFE_BUILD))
 if version<FULL_RATE_HOST_BUILD:
  raise ValueError('当前 FPGA 位流 0x%08x 不支持连续 ISP→PCIe 本地视频；需要版本 0x%08x 的新 bit。旧 bit 不会回退到 DDR→主机 DMA。'%(version,FULL_RATE_HOST_BUILD))
 if cfg['format']=='bayer' and mode&0xffff0000!=0x42590000:
  raise ValueError('当前 bitstream 没有 RAW Bayer 旁路')
 return dict(build_id=build_id,version=version)
