"""Pixel formats: preserve RAW for transport, demosaic only for local preview."""
import ctypes
from pathlib import Path
from PIL import Image
RGB=0x33424752
BGR=0x33524742
BAYER=0x42474752
_decoder=None
def to_image(pixels,w,h,fmt):
 global _decoder
 if fmt in (RGB,BGR):return Image.frombytes('RGB',(w,h),pixels,'raw','RGB' if fmt==RGB else 'BGR')
 if fmt!=BAYER or w<2 or h<2 or w%2 or h%2 or len(pixels)!=w*h:raise ValueError('Invalid BayerRG8 geometry')
 if _decoder is None:
  _decoder=ctypes.CDLL(str(Path(__file__).resolve().parent/'libvideo_jpeg.so')).video_bayer_rg8_to_rgb
  _decoder.argtypes=[ctypes.c_char_p,ctypes.c_size_t,ctypes.c_uint,ctypes.c_uint,ctypes.c_void_p,ctypes.c_size_t];_decoder.restype=ctypes.c_int
 out=ctypes.create_string_buffer(w*h*3)
 if _decoder(pixels,len(pixels),w,h,out,len(out)):raise ValueError('Bayer preview conversion failed')
 return Image.frombytes('RGB',(w,h),out.raw)
def format_name(fmt):
 return 'BayerRG8 · 本机去马赛克预览' if fmt==BAYER else 'RGB888'
