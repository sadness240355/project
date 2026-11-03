"""Lossless desktop snapshots stored only inside the local project."""
import json,os,shutil,time
from pathlib import Path
from video_pixels import BAYER

def save_frame(root,item,raw,side):
 root=Path(root).resolve();directory=root/'captures';directory.mkdir(exist_ok=True)
 if not directory.resolve().is_relative_to(root):raise ValueError('保存目录不能指向项目外')
 if shutil.disk_usage(directory).free<128*1024**2:raise RuntimeError('磁盘剩余空间不足 128 MiB')
 im=item[3];bayer=len(item)>9 and item[8]==BAYER and item[9] is not None
 name='desktop_'+('raw_' if raw else 'preview_')+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.monotonic_ns())
 path=directory/(name+('.png' if raw else '.jpg'))
 temp=path.with_suffix(path.suffix+'.part')
 meta={'source_frame':str(item[2]),'width':im.width,'height':im.height,'side':side,
       'source':'BayerRG8 original; PNG is demosaiced preview' if bayer else 'FPGA RGB lossless snapshot' if raw else 'decoded JPEG preview',
       'format':'BayerRG8' if bayer else 'RGB888','image_file':path.name,'reference_crc32':item[4]}
 try:
  im.save(temp,format='PNG' if raw else 'JPEG',**({} if raw else {'quality':90}))
  if bayer:
   if len(item[9])!=im.width*im.height:raise ValueError('原始 Bayer 帧长度不匹配')
   path.with_suffix('.bayer8').write_bytes(item[9]);meta['raw_file']=path.with_suffix('.bayer8').name
  path.with_suffix('.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2))
  os.replace(temp,path)
 except Exception:
  temp.unlink(missing_ok=True)
  raise
 return path
