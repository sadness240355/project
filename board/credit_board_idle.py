#!/usr/bin/env python3
import os
"""Require both station endpoints and FPGA requesters to be idle; read only."""
import json,urllib.request,os,mmap,struct,datetime
op=urllib.request.build_opener(urllib.request.ProxyHandler({}));d=json.load(op.open(os.environ.get('CONTROL_URL','http://10.23.68.0:8788')+'/status',timeout=8));c=d['console']
assert not c.get('busy'),'station operation is active'
roles={}
for role in ['capture','sender','receiver']:
 x=c.get(role,{});roles[role]={k:x.get(k) for k in ['pid','running','external_pid','safety_hold']}
 assert not any(x.get(k) for k in ['running','external_pid','safety_hold']),role+' still owns transport'
fd=os.open(os.environ.get('FPGA_RESOURCE','/sys/bus/pci/devices/0005:01:00.0/resource2'),os.O_RDONLY);m=mmap.mmap(fd,0x310000,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ)
rd=lambda a:struct.unpack_from('<I',m,a)[0]
values={hex(a):rd(a) for a in [0x14000,0x14a00,0x14a10,0x14a1c,0x300000]}
assert not rd(0x14000)&1 and rd(0x14a00)==0 and rd(0x14a10)&1 and rd(0x14a1c)==0,'camera requester not disabled/idle'
assert rd(0x300000)&1==0,'LRDMA remains enabled; retain memory and drain/reset first'
m.close();os.close(fd)
print(json.dumps({'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'status':'BOTH_ENDPOINTS_AND_FPGA_IDLE','roles':roles,'registers':values},ensure_ascii=False,indent=2))
