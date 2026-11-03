#!/usr/bin/env python3
"""Audit fresh paired RDMA runs and summarize completion-window rates; exclude warmup."""
import argparse,csv,json,re,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('results',type=Path);p.add_argument('campaign');p.add_argument('out',type=Path);p.add_argument('--require-cpu',type=int);a=p.parse_args()
assert not a.out.exists(),'refusing overwrite'
points=[];inputs=[];total_checked=0
for batch in [1,50]:
 for exponent in range(7,24):
  length=1<<exponent;stem=f'{a.campaign}_{length}_b{batch}'
  tx=a.results/(stem+'_tx');rx=a.results/(stem+'_rx')
  if a.require_cpu is not None:
   assert (tx/'cpu_affinity.txt').read_text().strip()==f'CPU={a.require_cpu}' and (rx/'cpu_affinity.txt').read_text().strip()==f'CPU={a.require_cpu}'
  assert (tx/'exit_code.txt').read_text().strip()=='0' and (rx/'exit_code.txt').read_text().strip()=='0',stem
  assert 'SOURCE=FPGA_DDR REPEATS=100 WARMUP=1' in (tx/'sender.stderr').read_text(),stem
  rows=list(csv.DictReader((tx/'sender.csv').open()))
  checks=re.findall(r'VERIFY length=(\d+) batch=(\d+) checked=(\d+) mismatches=(\d+) first=(\d+)',(rx/'receiver.log').read_text())
  assert len(rows)==len(checks)==101,stem
  assert int(rows[0]['repeat'])==-1 and rows[0]['phase']=='warmup'
  for i,(r,c) in enumerate(zip(rows,checks)):
   ln,nb,nbytes,bad,first=map(int,c)
   assert ln==length and nb==batch and nbytes==length*batch and bad==0,stem
   assert int(r['repeat'])==i-1 and r['phase']==('warmup' if i==0 else 'measurement')
   assert int(r['length'])==length and int(r['batch'])==batch
   assert int(r['completed_bytes'])==int(r['remote_bytes_verified'])==nbytes
   ns=int(r['elapsed_ns']);assert ns>0
   assert abs(float(r['GBps'])-nbytes/ns)<1e-7
   assert abs(float(r['amortized_ns_per_WQE'])-ns/batch)<0.0011
   total_checked+=nbytes
  used=rows[1:];elapsed=sum(int(x['elapsed_ns']) for x in used);completed=length*batch*len(used)
  points.append({'length_bytes':length,'batch':batch,'measured_batches':100,'measured_WQEs':100*batch,'measured_completed_bytes':completed,'sum_completion_window_ns':elapsed,'payload_Gbit_s':completed/elapsed*8,'mean_completion_window_us':elapsed/100/1000,'mean_per_WQE_amortized_us':elapsed/(100*batch)/1000})
  for f in [tx/'sender.csv',tx/'sender.stderr',rx/'receiver.log',tx/'binary.sha256',rx/'binary.sha256']:
   inputs.append({'path':str(f),'sha256':hashlib.sha256(f.read_bytes()).hexdigest()})
a.out.mkdir(parents=True)
with (a.out/'points.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(points[0]));w.writeheader();w.writerows(points)
summary={'status':'ALL_34_PAIRED_CASES_VERIFIED','measurement_batches':3400,'required_CPU_affinity':a.require_cpu,'warmup_batches':34,'total_bytes_checked_including_warmup':total_checked,'measurement_WQEs':sum(x['measured_WQEs'] for x in points),'source':'FPGA DDR; initialized and fully read back before each point','timing_scope':'submission of prepared WQEs through strict CQ/doorbell/WRID completion; preparation, TCP control and remote byte verification excluded','rate_aggregation':'sum verified payload bytes divided by sum measured completion-window durations; decimal Gbit/s','single_semantics':'one WQE submission-to-completion duration','batch_semantics':'50-WQE whole-batch duration; per-WQE amortized duration is batch duration divided by50, not individual request latency','not_claimed':['continuous wall-clock bandwidth including remote verification','application camera latency','each batched request has the amortized latency'],'max_by_mode':{str(b):max((x for x in points if x['batch']==b),key=lambda x:x['payload_Gbit_s']) for b in [1,50]},'inputs':inputs}
(a.out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'status':summary['status'],'checked_bytes':total_checked,'max_by_mode':summary['max_by_mode']},ensure_ascii=False))
