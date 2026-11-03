"""Small detector metadata only; no frame copies or cross-host clock subtraction."""
import math,mmap,os,struct,time
HEADER=struct.Struct('<6I3Q4I');RECORD=struct.Struct('<5f3i');SIZE=1024

def decode(packet,seq,stamp,width,height,extra_ms=0):
 if len(packet)!=SIZE:return None
 magic,version,count,w,h,valid,frame_id,captured,pub,*_=HEADER.unpack_from(packet)
 if (magic,version,valid)!=(0x31425859,1,1) or (w,h)!=(width,height) or count>20:return None
 age=(stamp-captured)/1e6+extra_ms;frames=(int(seq)-frame_id)&0xffffffff
 if stamp<captured or not 0<=age<=250 or frames>=0x80000000:return None
 boxes=[]
 for i in range(count):
  b=RECORD.unpack_from(packet,64+i*32)
  if not all(math.isfinite(v) for v in b[:5]) or not (0<=b[0]<b[2]<=w and 0<=b[1]<b[3]<=h and 0<=b[4]<=1):return None
  boxes.append(b)
 return dict(boxes=boxes,frame_id=frame_id,frame_age=frames,age_ms=age,publish_seq=pub)

class BoxReader:
 def __init__(self):self.mem=None;self.path='';self.identity=None;self.checked=0
 def close(self):
  if self.mem:self.mem.close()
  self.mem=None;self.identity=None
 def packet(self,path):
  try:
   now=time.monotonic()
   if path!=self.path or now-self.checked>.1:
    self.checked=now;s=os.stat(path);identity=(s.st_dev,s.st_ino)
    if self.path!=path or identity!=self.identity:
     self.close();self.path=path
     if s.st_size!=20480:return bytes(SIZE)
     with open(path,'rb') as f:self.mem=mmap.mmap(f.fileno(),0,access=mmap.ACCESS_READ)
     self.identity=identity
   m=self.mem
   if m is None:return bytes(SIZE)
   magic,version,page,record,max_boxes,slots,w,h=struct.unpack_from('<8I',m)
   if (magic,version,page,record,max_boxes,slots)!=(0x584f4259,1,4096,32,256,2):return bytes(SIZE)
   pub=struct.unpack_from('<Q',m,32)[0];slot=struct.unpack_from('<I',m,40)[0]
   if not pub or slot>1:return bytes(SIZE)
   guard=struct.unpack_from('<Q',m,48+8*slot)[0]
   if guard&1:return bytes(SIZE)
   from video_viewer import _fence
   _fence()
   seq=struct.unpack_from('<Q',m,64+8*slot)[0];stamp=struct.unpack_from('<Q',m,80+8*slot)[0];count=struct.unpack_from('<I',m,96+4*slot)[0]
   if count>256:return bytes(SIZE)
   count=min(count,20);records=m[4096+slot*8192:4096+slot*8192+32*count]
   _fence()
   if struct.unpack_from('<Q',m,48+8*slot)[0]!=guard or struct.unpack_from('<Q',m,32)[0]!=pub:return bytes(SIZE)
   return HEADER.pack(0x31425859,1,count,w,h,1,seq,stamp,pub,0,0,0,0)+records+bytes(SIZE-64-len(records))
  except (OSError,ValueError,struct.error):self.close();return bytes(SIZE)
