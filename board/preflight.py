#!/usr/bin/env python3
import os
import subprocess,json,os,glob,platform,datetime,sys
commands=[['uname','-a'],['lspci','-nn'],['lspci','-vv','-s','0005:01:00.0'],['ip','-details','link'],['rdma','link'],['ps','-eo','pid,comm']]
results=[]
for cmd in commands:
 try:
  p=subprocess.run(cmd,capture_output=True,text=True,timeout=15)
  results.append(dict(argv=cmd,exit_code=p.returncode,stdout=p.stdout,stderr=p.stderr))
 except Exception as e:results.append(dict(argv=cmd,error=str(e)))
sysfs={}
for name in ['current_link_speed','current_link_width','vendor','device','subsystem_vendor','subsystem_device']:
 path='/sys/bus/pci/devices/0005:01:00.0/'+name
 if os.path.isfile(path):sysfs[name]=open(path).read().strip()
print(json.dumps(dict(status='PREFLIGHT_ONLY_NOT_AN_EXPERIMENT_PASS',utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),hostname=platform.node(),commands=results,device_nodes=glob.glob('/dev/edev*'),pcie=sysfs),indent=2))
