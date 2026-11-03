#!/usr/bin/env python3
"""Snapshot production application sources; exclude existing tests, validation scripts and outputs."""
from pathlib import Path
import tarfile,json,hashlib,datetime,argparse,os
parser=argparse.ArgumentParser(description="按相对目录快照应用软件源码")
parser.add_argument("--source-root",default=os.environ.get("PRODUCTION_SOFTWARE_ROOT","software"),help="应用软件根目录")
parser.add_argument("--output-root",default=os.environ.get("SNAPSHOT_OUTPUT_ROOT","source"),help="输出目录")
args=parser.parse_args()
root=Path(args.source_root).expanduser()
out=Path(args.output_root).expanduser()
archive=out/'application_inputs_20261030.tar'
manifest=out/'application_inputs_20261030.json' 
assert not archive.exists() and not manifest.exists(),'refusing overwrite'
files=list((root/'src').glob('*.c'))+list((root/'src').glob('*.h'))
for name in ['edev_camera_io.c','edev_camera_io.h','hik_camera_cfg.c','camera_viewer_bayerrg8_egl.c','video_host_yolo_sidecar.h']:
 p=root/'capture_reference'/name
 if p.is_file():files.append(p)
files+=list((root/'lib').glob('*.c'))+list((root/'lib').glob('*.h'))
exclude={'video_bandwidth.py','video_pressure.py'}
files += [p for p in root.glob('video_*.py') if p.name not in exclude]
for name in ['console.html','console.css','console.js','dual.html']:
 p=root/name
 if p.is_file():files.append(p)
records=[]
with tarfile.open(archive,'w') as tf:
 for p in sorted(set(files)):
  rel=Path('application/software')/p.relative_to(root)
  tf.add(p,arcname=str(rel),recursive=False)
  records.append({'path':str(rel),'origin':str(p.relative_to(root.parent)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size})
manifest.write_text(json.dumps({'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'PRODUCTION_DUT_SOURCE_ONLY_NOT_EXISTING_VALIDATION_HARNESS','excluded':['tests/','validation/','captures/','models/','old build scripts/results','video_bandwidth.py','video_pressure.py'],'files':records},indent=2)+'\n')
print(json.dumps({'files':len(records),'archive':str(archive),'sha256':hashlib.sha256(archive.read_bytes()).hexdigest()}))
