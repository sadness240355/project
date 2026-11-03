#!/usr/bin/env python3
"""Copy portable application inputs, excluding all locally generated binaries."""
from pathlib import Path
import shutil, sys
root = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]).resolve() / 'application/software'
out.mkdir(parents=True, exist_ok=False)
for directory, target in [(root/'src/software/ui',out),(root/'src/software/runtime_lib',out/'lib')]:
    for src in directory.rglob('*'):
        rel = src.relative_to(directory)
        if any(part in {'build','runtime','models','__pycache__','.tmp','.git','lib'} for part in rel.parts):
            continue
        if src.is_file() and src.suffix in {'.c','.h','.py','.sh','.css','.html','.js','.json','.svg','.png','.jpg'}:
            dst = target / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
