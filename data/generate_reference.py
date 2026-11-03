#!/usr/bin/env python3
from pathlib import Path
import hashlib,zlib,json,sys
r=Path(__file__).resolve().parent
out=r/(sys.argv[1] if len(sys.argv)>1 else 'reference_v1')
if out.exists():raise SystemExit('refusing overwrite')
out.mkdir()
# Independent deterministic byte-pattern oracle; generate the whole stream including segment boundaries.
records=[]
for exponent in range(10,27):
 n=1<<exponent;sha=hashlib.sha256();crc=0
 for start in range(0,n,65536):
  b=bytes(((i*17+(i>>8)+0x5a)&255) for i in range(start,min(start+65536,n)))
  sha.update(b);crc=zlib.crc32(b,crc)
  if exponent==10 and start==0:(out/'dma_1KiB.bin').write_bytes(b)
 records.append(dict(bytes=n,seed=0x5a,sha256=sha.hexdigest(),crc32=f'{crc:08x}'))
colors=[(255,255,255),(255,255,0),(0,255,255),(0,255,0),(255,0,255),(255,0,0),(0,0,255),(0,0,0)]
w,h=2400,2048
row=b''.join(bytes(colors[min(x*8//w,7)]) for x in range(w))
frame=row*h
(out/'colorbar_2400x2048_rgb888.bin').write_bytes(frame)
manifest=dict(pattern_formula='byte[i]=(17*i+(i>>8)+0x5a)&255; i is global byte offset',dma=records,colorbar=dict(width=w,height=h,format='RGB888 byte order R,G,B',bytes=len(frame),sha256=hashlib.sha256(frame).hexdigest(),crc32=f'{zlib.crc32(frame):08x}'),reference_scope='fresh host-generated oracle, not a measured FPGA result')
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(manifest['colorbar']))
