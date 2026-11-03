#!/usr/bin/env python3
import os
"""Independent LISP MMIO test harness; aligned 32-bit register transactions."""
import argparse,ctypes,json,mmap,os,re,struct,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('id');p.add_argument('capture_id');a=p.parse_args()
assert all(re.fullmatch(r'[A-Za-z0-9_-]+',s) for s in [a.id,a.capture_id])
r=Path(__file__).resolve().parent.parent;out=r/'board/results'/a.id;out.mkdir(exist_ok=False);(out/'source.py').write_bytes(Path(__file__).read_bytes())
log=r/'board/results'/a.capture_id/'run.stderr';until=time.monotonic()+40
while time.monotonic()<until:
 if log.exists() and 'MEASUREMENT_STARTED' in log.read_text():break
 time.sleep(.05)
else:raise RuntimeError('capture not started; no writes')
fd=os.open(os.environ.get('FPGA_RESOURCE','/sys/bus/pci/devices/0005:01:00.0/resource2'),os.O_RDWR);m=mmap.mmap(fd,0x15000,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ|mmap.PROT_WRITE)
def rd(a):return struct.unpack_from('<I',m,a)[0]
def wr(a,v):ctypes.c_uint32.from_buffer(m,a).value=v;rd(0x14070)
assert rd(0x14100)==0x4c495350 and rd(0x14104)==0x10000
start=time.monotonic();events=[]
for i,mask in enumerate([15,0,15,0,15]):
 target=start+5+10*i
 while time.monotonic()<target:time.sleep(max(0,min(.05,target-time.monotonic())))
 s=rd(0x1410c);assert s&4 and not s&3
 gen=rd(0x14114);frame_before=rd(0x14024)
 fields=[mask,0,7782,7782,7987,4096,0,0,0,4096,0,0,0,4096]
 for j,v in enumerate(fields):wr(0x14140+4*j,v)
 assert [rd(0x14140+4*j) for j in range(14)]==fields
 t0=time.monotonic_ns();wr(0x14110,1);until=time.monotonic()+2
 while rd(0x14114)!=((gen+1)&0xffffffff) or rd(0x1410c)&1:
  assert not rd(0x1410c)&2 and time.monotonic()<until
  time.sleep(.001)
 assert [rd(0x14180+4*j) for j in range(14)]==fields
 events.append({'step':i+1,'isp_mask':mask,'generation_before':gen,'generation_after':rd(0x14114),'observed_submit_ns':t0,'observed_complete_ns':time.monotonic_ns(),'producer_before':frame_before,'producer_after':rd(0x14024),'active_Q12_fields':fields})
 (out/'events.json').write_text(json.dumps(events,indent=2)+'\n')
 print(json.dumps(events[-1]),flush=True)
(out/'result.json').write_text(json.dumps({'status':'FIVE_COMMIT_REGISTER_CHECK_PASS_CHECK_CAPTURE_LOGS','events':5,'scope':'new aligned32 MMIO harness; local capture only; no remote CRC or cycle-accurate SOF timing proof'},indent=2)+'\n')
