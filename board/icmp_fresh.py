#!/usr/bin/env python3
import os
"""Fresh ST-network functional check; not an RDMA throughput measurement."""
import argparse,datetime,hashlib,json,re,subprocess,os
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('id');a=p.parse_args();assert re.fullmatch(r'[A-Za-z0-9_-]+',a.id)
root=Path(__file__).resolve().parent.parent;out=root/'board/results'/a.id;out.mkdir(exist_ok=False)
(out/'source.py').write_bytes(Path(__file__).read_bytes())
subprocess.run(['python3',str(root/'board/session_preflight.py'),'--url',os.environ.get('CONTROL_URL','http://10.23.68.0:8788')],stdout=(out/'application_idle.json').open('w'),check=True)
base=Path('/sys/class/net')/os.environ.get('FPGA_NETDEV','reconic1s0f0')/'statistics'
def stats():return {f.name:int(f.read_text()) for f in base.iterdir()}
before=stats();cmd=['ping','-n','-I',os.environ.get('FPGA_NETDEV','reconic1s0f0'),'-c','13','-i','0.2','-W','2','-s','1200','-p','5aa517e03c6912ff',os.environ.get('PEER_IP','192.168.1.20')]
(out/'command.json').write_text(json.dumps(cmd))
r=subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env={**os.environ,"LC_ALL":"C"})
after=stats();(out/'ping.stdout').write_text(r.stdout);(out/'ping.stderr').write_text(r.stderr)
(out/'exit_code.txt').write_text(str(r.returncode)+'\n')
replies=re.findall(r'icmp_seq=(\d+).*ttl=\d+.*time[=<]([\d.]+) ms',r.stdout)
passed=r.returncode==0 and [int(s) for s,_ in replies]==list(range(1,14)) and '0% packet loss' in r.stdout and not re.search('wrong|mismatch|duplicate|BAD',r.stdout,re.I)
result={'status':'PASS' if passed else 'FAIL','utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'interface':os.environ.get('FPGA_NETDEV','reconic1s0f0'),'destination':os.environ.get('PEER_IP','192.168.1.20'),'ICMP_payload_bytes':1200,'required_packets':13,'replies':len(replies),'before':before,'after':after,'deltas':{k:after[k]-before[k] for k in before},'verification':'iputils ping echo identifier, sequence and payload pattern checks; no external packet-capture oracle','scope':'ST netdev functional check; network counters may also include ARP/IPv6 background traffic; not RDMA or bandwidth evidence'}
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'status':result['status'],'replies':len(replies)}));raise SystemExit(0 if passed else 1)
