#!/usr/bin/env python3
"""Validate paired fresh static-colorbar records; refuse a partial result."""
import argparse,csv,hashlib,json,math,re,zlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('out',type=Path);a=p.parse_args()
r=a.root;manifest=json.loads((r/'data/reference_v1/manifest.json').read_text())['colorbar']
ref=(r/'data/reference_v1/colorbar_2400x2048_rgb888.bin').read_bytes()
assert len(ref)==manifest['bytes'] and hashlib.sha256(ref).hexdigest()==manifest['sha256']
assert f'{zlib.crc32(ref):08x}'==manifest['crc32']
rows=[];inputs=[]
def load(d,name):
 f=d/name;inputs.append({'path':str(f.relative_to(r)),'sha256':hashlib.sha256(f.read_bytes()).hexdigest()});return f.read_text()
for mode in ['dma','rdma']:
 for n in [1,2]:
  d=r/'board/results'/f'colorbar_{mode}_60s_round{n:03d}{"_tx" if mode=="rdma" else ""}'
  assert load(d,'exit_code.txt').strip()=='0'
  x=json.loads(load(d,'result.json'));frames=x['source_completed_frames' if mode=='rdma' else 'completed_frames']
  assert x['frame_bytes']==len(ref) and x['completed_bytes']==frames*len(ref)
  assert x['elapsed_ns']>=60_000_000_000 and x['CRC_errors']==0 and x['reference_crc32']==manifest['crc32']
  gbps=x['completed_bytes']*8/x['elapsed_ns'];assert math.isclose(gbps,x['Gbit_s'],rel_tol=1e-9)
  if mode=='dma':
   b=(d/'final_readback.bin').read_bytes();assert b==ref and x['initial_and_final_full_byte_comparison'] and x['CRC_samples']>=60
   inputs.append({'path':str((d/'final_readback.bin').relative_to(r)),'sha256':hashlib.sha256(b).hexdigest()})
   samples=x['CRC_samples']
  else:
   peer=r/'board/results'/f'colorbar_rdma_60s_round{n:03d}_rx'
   assert load(peer,'exit_code.txt').strip()=='0'
   checks=re.findall(r'VERIFY length=(\d+) batch=(\d+) checked=(\d+) mismatches=(\d+) first=\d+ crc32=([0-9a-f]+)',load(peer,'receiver.log'))
   assert len(checks)==x['total_CRC_samples'] and x['periodic_CRC_samples']>=60
   assert all(int(l)==len(ref) and int(batch)==1 and int(checked)==len(ref) and int(errors)==0 and crc==manifest['crc32'] for l,batch,checked,errors,crc in checks)
   samples=len(checks)
  rows.append({'path':mode.upper(),'round':n,'frame_bytes':len(ref),'completed_frames':frames,'completed_bytes':x['completed_bytes'],'elapsed_ns':x['elapsed_ns'],'wall_clock_Gbit_s':gbps,'CRC_checks':samples,'CRC_errors':0})
a.out.mkdir(parents=True,exist_ok=False)
with (a.out/'points.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
summary={'status':'ALL_FOUR_60_SECOND_RUNS_VERIFIED','points':rows,'scope':['DMA and RDMA measured separately, not simultaneous','Wall-clock throughput includes periodic verification/control overhead','DMA count is synchronous C2H completion count; initial/final full byte comparisons plus periodic CRC','RDMA count is source reliable RC completion count; 60 periodic remote full-frame comparisons plus initial/final checks','Static identical frames do not prove independent remote per-frame application event counting','No physical camera or inference in these colorbar runs'],'reference':manifest,'inputs':inputs}
(a.out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
print(summary['status'])
