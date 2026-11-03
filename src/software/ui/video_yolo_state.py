"""Atomic, project-local control for a resident detector; no DMA ownership."""
import json,os,re
from pathlib import Path

def write_state(path, values):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 tmp=path.with_name(path.name+'.new')
 with tmp.open('w') as f:json.dump(values,f)
 os.replace(tmp,path)

def read_state(path):
 try:
  d=json.loads(Path(path).read_text())
  return d if isinstance(d,dict) else {}
 except (OSError,ValueError):return {}

def command(path):
 d=read_state(path);session=d.get('session','')
 if not isinstance(session,str) or not re.fullmatch('[0-9a-f]{24}',session):return {}
 return dict(session=session,enabled=d.get('enabled') is True,
             shm='/lrdma_detect_'+session,boxes_shm='/lrdma_boxes_'+session)

def gate(path,session):
 d=command(path)
 return 'stop' if d.get('session')!=session else ('run' if d['enabled'] else 'pause')
