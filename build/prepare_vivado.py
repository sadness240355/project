#!/usr/bin/env python3
"""Convert the submitted XPR source manifest to a fresh, cache-independent project."""
from pathlib import Path
import argparse, xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
XPR = ROOT / 'src/rtl/vivado_project/lrdma_cmac_v5.xpr'

def tcl(value):
    return '{' + str(value).replace('\\', '/') + '}'

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(XPR)
    def path(value):
        if not value.startswith('$PPRDIR/'):
            raise ValueError('unexpected nonportable source: ' + value)
        f = XPR.parent / value[len('$PPRDIR/'):] 
        if not f.is_file():
            raise FileNotFoundError(f)
        return f
    lines = ['# Generated from the packaged XPR; do not edit.',
             'create_project lrdma_repro ' + tcl(out / 'vivado') + ' -part xcku15p-ffve1517-2-i',
             'set_property target_language Verilog [current_project]',
             'set_property XPM_LIBRARIES {XPM_CDC XPM_FIFO XPM_MEMORY} [current_project]',
             'set_property ip_repo_paths ' + '[list ' + tcl(XPR.parent / 'reconic_zdma.srcs/sources_1/ip/bayer8_to_rgba8888') + '] [current_project]',
             'update_ip_catalog']
    for fs in tree.findall('./FileSets/FileSet'):
        name = fs.get('Name')
        if name not in ('sources_1', 'constrs_1') and not fs.get('Type', '').startswith('BlockSrcs'):
            continue
        target = 'constrs_1' if name == 'constrs_1' else 'sources_1'
        for f in fs.findall('File'):
            src = path(f.get('Path'))
            if src.suffix == '.xci':
                lines.append('import_ip -files [list ' + tcl(src) + ']')
                continue
            lines.append('add_files -norecurse -fileset ' + target + ' [list ' + tcl(src) + ']')
            attrs = f.findall('./FileInfo/Attr')
            uses = [a.get('Val') for a in attrs if a.get('Name') == 'UsedIn']
            if uses:
                lines.append('set_property USED_IN ' + tcl(' '.join(uses)) + ' [get_files [list ' + tcl(src) + ']]')
            for a in attrs:
                if a.get('Name') == 'ProcessingOrder':
                    lines.append('set_property PROCESSING_ORDER ' + tcl(a.get('Val')) + ' [get_files [list ' + tcl(src) + ']]')
        if name == 'sources_1':
            config = fs.find('Config')
            includes = [XPR.parent / o.get('Val')[len('$PPRDIR/'):]  for o in config.findall('Option') if o.get('Name') == 'VerilogDir']
            lines.append('set_property include_dirs [list ' + ' '.join(map(tcl, includes)) + '] [get_filesets sources_1]')
            defines = [d.get('Name') for d in config.findall('Define')]
            lines.append('set_property verilog_define [list ' + ' '.join(map(tcl, defines)) + '] [get_filesets sources_1]')
            generics = [g.get('Name') + '=' + g.get('Val') for g in config.findall('Generic')]
            lines.append('set_property generic [list ' + ' '.join(map(tcl, generics)) + '] [get_filesets sources_1]')
    lines += ['set_property top open_nic_shell [get_filesets sources_1]',
              'set_property strategy Performance_Explore [get_runs impl_1]',
              'set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]',
              'set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE Explore [get_runs impl_1]',
              'set_property STEPS.OPT_DESIGN.TCL.PRE ' + tcl(ROOT / 'build/pre_opt_margin_v7.tcl') + ' [get_runs impl_1]',
              'set_property STEPS.WRITE_BITSTREAM.TCL.PRE ' + tcl(ROOT / 'build/pre_bitstream_timing_gate.tcl') + ' [get_runs impl_1]',
              'update_compile_order -fileset sources_1',
              'report_ip_status -file ' + tcl(out / 'ip_status.rpt')]
    (out / 'create_project.tcl').write_text('\n'.join(lines) + '\n')
    print(out / 'create_project.tcl')

if __name__ == '__main__':
    main()
