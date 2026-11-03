#!/usr/bin/env python3
"""Parse new Vivado hierarchy and timing reports; synthesis is never routed acceptance."""
from pathlib import Path
import argparse,json,re,hashlib,datetime
p=argparse.ArgumentParser();p.add_argument('--reports',type=Path,required=True);p.add_argument('--prefix',choices=['synthesis','implementation'],required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
if a.out.exists():raise SystemExit('refusing overwrite')
u=a.reports/(a.prefix+'_utilization.rpt');t=a.reports/(a.prefix+'_timing.rpt')
cells=[x.strip() for x in next(x for x in u.read_text().splitlines() if re.match(r'\|\s*open_nic_shell\s*\|',x)).split('|')[1:-1]]
numbers=next(x for x in t.read_text().split('Design Timing Summary',1)[1].splitlines() if re.match(r'\s*[-+]?\d+\.\d+\s+',x)).split()
v=dict(WNS_ns=float(numbers[0]),TNS_ns=float(numbers[1]),WHS_ns=float(numbers[4]),THS_ns=float(numbers[5]))
result={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'stage':'POST_SYNTHESIS' if a.prefix=='synthesis' else 'POST_ROUTE','resources':dict(zip(['total_LUT','logic_LUT','LUTRAM','SRL','FF','RAMB36','RAMB18','URAM','DSP'],map(int,cells[2:]))),'timing':v,'timing_met':v['WNS_ns']>=0 and v['WHS_ns']>=0 and v['TNS_ns']==0 and v['THS_ns']==0,'board_acceptance':False,'remaining_reviews':['routed timing required' if a.prefix=='synthesis' else 'DRC/CDC and controlled programming still required'],'reports':{str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [u,t]}}
a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result['timing']))
