#!/usr/bin/env python3
import os
import urllib.request,json,time,argparse,datetime
p=argparse.ArgumentParser();p.add_argument('--url',required=True);p.add_argument('--stop',action='store_true');a=p.parse_args()
op=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def get():return json.load(op.open(a.url+'/status',timeout=8))
def brief(d):
 c=d.get('console',{});tasks=c.get('tasks',{})
 allowed=['width','height','format','transport','fps','session','buffers','host_path','source','pressure_pattern','ring_base','frame_stride']
 return {'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'CURRENT_PRECONDITION_NOT_HISTORICAL_MEASUREMENT','state':d.get('state'),'busy':c.get('busy'),'tasks':{k:{'running':v.get('running'),'pid':v.get('pid'),'external_pid':v.get('external_pid'),'safety_hold':v.get('safety_hold',False),'config':{x:v.get('config',{}).get(x) for x in allowed if x in v.get('config',{})}} for k,v in tasks.items()}}
d=get();result={'before':brief(d)}
active=any(v.get('running') or v.get('external_pid') for v in d.get('console',{}).get('tasks',{}).values())
if a.stop and active:
 token=d.get('csrf','');req=urllib.request.Request(a.url+'/api/pipeline',data=json.dumps({'action':'stop'}).encode(),headers={'Content-Type':'application/json','X-Console-Token':token},method='POST')
 result['stop_response']=json.load(op.open(req,timeout=45))
 for _ in range(40):
  time.sleep(1);d=get();b=brief(d)
  if not b['busy'] and not any(x.get('running') or x.get('external_pid') for x in b['tasks'].values()):break
result['after']=brief(get());print(json.dumps(result,ensure_ascii=False,indent=2))
if any(v.get('running') or v.get('external_pid') or v.get('safety_hold') for v in result['after']['tasks'].values()):raise SystemExit(3)
