#!/usr/bin/env python3
"""Prepare the project-local YOLO11n TensorRT engine once, then reuse it."""
import argparse
import fcntl
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MODEL_DIR = ROOT / 'models'
WEIGHTS = MODEL_DIR / 'yolo11n.pt'
ENGINE = MODEL_DIR / 'yolo11n_640x384.engine'
MANIFEST = MODEL_DIR / 'yolo11n_640x384.json'


def sha256(path):
 h = hashlib.sha256()
 with Path(path).open('rb') as stream:
  for block in iter(lambda: stream.read(1024 * 1024), b''):
   h.update(block)
 return h.hexdigest()


def usable_engine():
 if not ENGINE.is_file() or ENGINE.stat().st_size < 4096:
  return False
 if not WEIGHTS.is_file() or not MANIFEST.is_file():
  return True  # An externally supplied, board-specific engine is valid to try.
 try:
  record = json.loads(MANIFEST.read_text())
  return record.get('weights_sha256') == sha256(WEIGHTS) and \
         record.get('engine_sha256') == sha256(ENGINE)
 except (OSError, ValueError):
  return False


def ensure():
 MODEL_DIR.mkdir(parents=True, exist_ok=True)
 with (MODEL_DIR / '.deploy.lock').open('a+') as lock:
  fcntl.flock(lock, fcntl.LOCK_EX)
  if usable_engine():
   print('YOLO_ENGINE_READY ' + str(ENGINE), flush=True)
   return 0
  if not WEIGHTS.is_file():
   print('YOLO_MODEL_MISSING: place yolo11n.pt in ' + str(MODEL_DIR), flush=True)
   return 2
  import torch
  from ultralytics import YOLO
  if not torch.cuda.is_available():
   print('YOLO_DEPLOY_FAILED: CUDA is unavailable', flush=True)
   return 1
  with tempfile.TemporaryDirectory(prefix='.yolo-build-', dir=MODEL_DIR) as tmp:
   source = Path(tmp) / WEIGHTS.name
   shutil.copy2(WEIGHTS, source)
   print('YOLO_EXPORT_BEGIN imgsz=384x640 precision=FP16', flush=True)
   product = Path(YOLO(str(source), task='detect').export(
    format='engine', imgsz=(384, 640), device=0, half=True,
    batch=1, dynamic=False, simplify=False, workspace=2))
   if not product.is_file() or product.stat().st_size < 4096:
    raise RuntimeError('TensorRT export did not produce a usable engine')
   candidate = MODEL_DIR / '.yolo11n_640x384.engine.new'
   shutil.copy2(product, candidate)
   os.replace(candidate, ENGINE)
   record = dict(weights_sha256=sha256(WEIGHTS), engine_sha256=sha256(ENGINE),
                 input_height=384, input_width=640, precision='FP16')
   temp_manifest = MODEL_DIR / '.yolo11n_640x384.json.new'
   temp_manifest.write_text(json.dumps(record, indent=2) + '\n')
   os.replace(temp_manifest, MANIFEST)
  print('YOLO_ENGINE_READY ' + str(ENGINE), flush=True)
  return 0


def main():
 parser = argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--ensure', action='store_true', help='export missing/stale engine from local weights')
 args = parser.parse_args()
 if args.ensure:
  try:return ensure()
  except Exception as exc:
   print('YOLO_DEPLOY_FAILED: ' + str(exc), flush=True)
   return 1
 print(json.dumps(dict(weights=str(WEIGHTS),weights_present=WEIGHTS.is_file(),
                       engine=str(ENGINE),engine_ready=usable_engine()),ensure_ascii=False))
 return 0


if __name__ == '__main__':
 raise SystemExit(main())
