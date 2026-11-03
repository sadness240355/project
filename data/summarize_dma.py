#!/usr/bin/env python3
"""Summarize and plot the fresh, verified synchronous MM matrix without historical data."""
import argparse,csv,json,hashlib
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('out',type=Path);a=p.parse_args()
a.out.mkdir(exist_ok=False,parents=True)
assert (a.run/'completed.txt').read_text().strip()=='MATRIX_PROCESS_COMPLETED'
points=[];raw=[]
for direction in ['h2c','c2h']:
 for exponent in range(12,21):
  for queues in range(1,5):
   f=a.run/f'{direction}_{1<<exponent}_q{queues}.csv'
   rows=list(csv.DictReader(f.open()));r=rows[-1]
   assert r['direction']=='AGGREGATE' and int(r['completed_bytes'])==(1<<exponent)*queues*100
   assert all(int(x['full_buffer_verified'])==1 for x in rows)
   points.append({'direction':direction.upper(),'length_bytes':1<<exponent,'queues':queues,'iterations_per_queue':100,'completed_bytes':int(r['completed_bytes']),'elapsed_ns':int(r['elapsed_ns']),'GBps':float(r['GBps'])})
   raw.append({'path':str(f),'sha256':hashlib.sha256(f.read_bytes()).hexdigest()})
with (a.out/'points.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(points[0]));w.writeheader();w.writerows(points)
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':10,'svg.fonttype':'none'})
for direction in ['H2C','C2H']:
 fig,ax=plt.subplots(figsize=(5.9,3.6),layout='constrained')
 for q,style,mark in zip(range(1,5),['-','--',':','-.'],['o','s','^','D']):
  subset=[x for x in points if x['direction']==direction and x['queues']==q]
  ax.plot([x['length_bytes']/1024 for x in subset],[x['GBps'] for x in subset],color='black',linestyle=style,marker=mark,markersize=4,linewidth=1.2,markerfacecolor='white',label=f'{q} 队列')
 ax.set_xscale('log',base=2);ax.set_xticks([4,8,16,32,64,128,256,512,1024],[str(x) for x in [4,8,16,32,64,128,256,512,1024]])
 ax.set_xlabel('单次传输长度（KiB）',fontproperties=font);ax.set_ylabel('完成窗口内有效吞吐率（GB/s）',fontproperties=font)
 ax.set_ylim(bottom=0);ax.grid(axis='y',color='0.85',linewidth=.6);ax.legend(prop=font,frameon=True)
 ax.spines[['right','top']].set_visible(False)
 for ext in ['png','svg','pdf']:fig.savefig(a.out/f'{direction.lower()}_bandwidth.{ext}',dpi=300)
 plt.close(fig)
summary={'scope':'APPLICATION_OBSERVED_SYNCHRONOUS_IOCTL_BANDWIDTH_NOT_RAW_PCIE_PEAK','source_run':str(a.run),'matrix_cells':len(points),'iterations_per_queue':100,'queue_outstanding_limit':1,'precheck_and_postcheck_outside_timing':True,'aggregate_window':'latest worker end minus earliest worker start','units':'decimal GB/s = completed bytes / elapsed nanoseconds','at_1MiB':{d:{str(q):next(x['GBps'] for x in points if x['direction']==d and x['queues']==q and x['length_bytes']==1048576) for q in range(1,5)} for d in ['H2C','C2H']},'raw_files':raw}
(a.out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(summary['at_1MiB']))
