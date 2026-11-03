#!/usr/bin/env python3
"""Persist the next capture's ISP mask inside the Jetson project."""
import json,os
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def read_isp_mask(root=ROOT):
 p=Path(root)/'capture_settings.json'
 if not p.exists():return 15
 value=json.loads(p.read_text())['isp_enable']
 if type(value) is not int or not 0<=value<=15:raise ValueError('ISP mask must be an integer from 0 to 15')
 return value
def read_isp_selection(root=ROOT):
 """Remember enabled sub-functions independently of the master bypass."""
 p=Path(root)/'capture_settings.json'
 if not p.exists():return 15
 data=json.loads(p.read_text())
 value=data.get('isp_selection',data.get('isp_enable',15)) or 15
 if type(value) is not int or not 1<=value<=15:raise ValueError('invalid ISP selection')
 return value
def save_isp_mask(value,root=ROOT,*,selection=None):
 if type(value) is not int or not 0<=value<=15:raise ValueError('invalid ISP mask')
 root=Path(root);selection=value or selection or read_isp_selection(root)
 if type(selection) is not int or not 1<=selection<=15:raise ValueError('invalid ISP selection')
 tmp=root/('.capture_settings.%d.tmp'%os.getpid())
 try:
  tmp.write_text(json.dumps({'isp_enable':value,'isp_selection':selection},indent=2)+'\n');tmp.replace(root/'capture_settings.json')
 finally:tmp.unlink(missing_ok=True)
if __name__=='__main__':print(read_isp_mask())
