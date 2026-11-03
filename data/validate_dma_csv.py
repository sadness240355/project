#!/usr/bin/env python3
"""Validate measured byte counts, timestamps and payload-verification flags without changing raw CSV."""
import csv,json,argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('files',nargs='+');a=p.parse_args();checks=[]
for name in a.files:
 with open(name,newline='') as f:rows=list(csv.DictReader(f))
 workers=[r for r in rows if r['direction']!='AGGREGATE'];total=[r for r in rows if r['direction']=='AGGREGATE']
 assert len(total)==1 and workers,name
 agg=total[0];n=int(agg['queue']);length=int(agg['length']);loops=int(agg['iterations'])
 assert len(workers)==n and len(set(r['queue'] for r in workers))==n
 for r in workers:
  assert int(r['completed_bytes'])==int(r['length'])*int(r['iterations'])
  assert int(r['full_buffer_verified'])==1 and int(r['elapsed_ns'])>0
  assert abs(float(r['GBps'])-int(r['completed_bytes'])/int(r['elapsed_ns']))<1e-7
 assert int(agg['completed_bytes'])==sum(int(r['completed_bytes']) for r in workers)==n*length*loops
 assert int(agg['full_buffer_verified'])==1
 assert abs(float(agg['GBps'])-int(agg['completed_bytes'])/int(agg['elapsed_ns']))<1e-7
 window_verified=False
 if all(r.get('start_ns') and r.get('end_ns') for r in workers):
  t0=min(int(r['start_ns']) for r in workers);t1=max(int(r['end_ns']) for r in workers)
  assert t1-t0==int(agg['elapsed_ns'])
  assert all(int(r['end_ns'])-int(r['start_ns'])==int(r['elapsed_ns']) for r in workers)
  window_verified=True
 checks.append(dict(file=name,completed_bytes=int(agg['completed_bytes']),GBps=float(agg['GBps']),aggregate_window_independently_verified=window_verified,status='CSV_CONSISTENT_SCOPE_REVIEW_STILL_REQUIRED'))
print(json.dumps(checks,indent=2))
