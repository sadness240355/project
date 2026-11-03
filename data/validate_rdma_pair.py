#!/usr/bin/env python3
"""Cross-check sender arithmetic and receiver full-payload verification; no scope promotion."""
import argparse,csv,json,re,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('sender',type=Path);p.add_argument('receiver_log',type=Path);a=p.parse_args()
rows=list(csv.DictReader((a.sender/'sender.csv').open()))
checks=re.findall(r'VERIFY length=(\d+) batch=(\d+) checked=(\d+) mismatches=(\d+) first=(\d+)',a.receiver_log.read_text())
assert len(rows)==len(checks)>0
for r,c in zip(rows,checks):
 length,batch,verified,bad,first=map(int,c)
 assert int(r['length'])==length and int(r['batch'])==batch
 assert int(r['completed_bytes'])==int(r['remote_bytes_verified'])==verified==length*batch and bad==0
 ns=int(r['elapsed_ns']);assert ns>0
 assert abs(float(r['GBps'])-verified/ns)<1e-7
 assert abs(float(r['amortized_ns_per_WQE'])-ns/batch)<0.0011
assert (a.sender/'exit_code.txt').read_text().strip()=='0'
scope=(a.sender/'bitstream_scope.txt').read_text().strip()
print(json.dumps({'status':'PAIRED_COUNTS_AND_PAYLOAD_VERIFICATION_CONSISTENT','cases':len(rows),'scope':scope,'paper_eligible':False,'note':'Requires separate bitstream identity, repeated measurement and protocol review before paper use.','sha256':{str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [a.sender/'sender.csv',a.receiver_log]}},indent=2))
