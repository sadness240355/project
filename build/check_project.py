#!/usr/bin/env python3
"""Offline submission audit: source closure, portable links and script syntax."""
from pathlib import Path
import ast, shutil, subprocess, sys, tarfile, xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parents[1]
errors = []
xpr = ROOT/'src/rtl/vivado_project/lrdma_cmac_v5.xpr'
tree = ET.parse(xpr)
count = 0
for fs in tree.findall('./FileSets/FileSet'):
    if fs.get('Name') == 'utils_1':
        continue  # Removed incremental checkpoint is deliberately not an input.
    for f in fs.findall('File'):
        value = f.get('Path')
        if not value.startswith('$PPRDIR/'):
            errors.append('nonportable XPR source: '+value)
            continue
        src = xpr.parent/value[len('$PPRDIR/'):]
        if not src.is_file(): errors.append('missing XPR source: '+str(src.relative_to(ROOT)))
        count += 1
repo = xpr.parent/'reconic_zdma.srcs/sources_1/ip/bayer8_to_rgba8888'
if not (repo/'component.xml').is_file(): errors.append('missing HLS IP component.xml')
for component in repo.rglob('component.xml'):
    component_tree = ET.parse(component)
    ns = {'s':'http://www.spiritconsortium.org/XMLSchema/SPIRIT/1685-2009'}
    for name in component_tree.findall('.//s:file/s:name',ns):
        value = name.text or ''
        if '$' not in value and not (component.parent/value).is_file():
            errors.append('missing packaged HLS IP file: '+value)
for link in ROOT.rglob('*'):
    if not link.is_symlink(): continue
    try: link.resolve().relative_to(ROOT)
    except ValueError: errors.append('external symlink: '+str(link.relative_to(ROOT)))
    if not link.exists(): errors.append('broken symlink: '+str(link.relative_to(ROOT)))
script_count = 0
for directory in ['build','sim','board','data','src/software','src/rtl']:
    for f in (ROOT/directory).rglob('*'):
        if any(x in {'results','application_build','driver_build','runtime_build','rdma_build','__pycache__','regenerated'} for x in f.relative_to(ROOT).parts): continue
        if f.suffix not in {'.sh','.py'} or not f.is_file(): continue
        try:
            if f.suffix == '.py': ast.parse(f.read_text(), filename=str(f))
            else:
                r = subprocess.run(['bash','-n',str(f)],capture_output=True,text=True)
                if r.returncode: errors.append(r.stderr.strip())
        except SyntaxError as e: errors.append(str(e))
        script_count += 1
for archive in (ROOT/'src/software/archives').glob('*.tar*'):
    try:
        with tarfile.open(archive) as contents:
            for item in contents:
                if item.isfile(): contents.extractfile(item).read()
    except (OSError, tarfile.TarError, EOFError) as e:
        errors.append('invalid archive '+archive.name+': '+str(e))
print('XPR inputs:',count,'; scripts checked:',script_count)
for name in ['python3','bash','iverilog','vvp','cc','ar','vivado','vitis_hls','make']:
    print('TOOL',name,shutil.which(name) or 'MISSING (install for the corresponding stage)')
if errors:
    print('\n'.join(errors), file=sys.stderr)
    sys.exit(1)
print('PROJECT_CHECK_PASS (source and syntax audit; not synthesis or board acceptance)')
