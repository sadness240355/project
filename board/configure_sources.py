#!/usr/bin/env python3
"""Apply host-specific literals only to disposable build copies, never submitted C."""
import json, os, sys
from pathlib import Path
root = Path(sys.argv[1]).resolve()
package = Path(__file__).resolve().parents[1]
if root == package or root in (package/'src', package/'board'):
    raise SystemExit('Only a generated build source directory may be configured')
def env(name, default):
    return os.environ.get(name, default)
replacements = {
    '/sys/bus/pci/devices/0005:01:00.0/resource2': env('FPGA_RESOURCE', '/sys/bus/pci/devices/0005:01:00.0/resource2'),
    '/dev/edev0': env('DMA_DEVICE', '/dev/edev0'),
    '192.168.1.10': env('FPGA_IP', '192.168.1.10'),
    '192.168.1.20': env('PEER_IP', '192.168.1.20'),
    '10.23.66.188': env('PEER_CONTROL_IP', '10.23.66.188'),
    'mlx5_0': env('RDMA_DEVICE', 'mlx5_0'),
    'enp1s0f0np0': env('PEER_NETDEV', 'enp1s0f0np0'),
}
for f in root.rglob('*'):
    if f.suffix not in ('.c', '.h') or not f.is_file():
        continue
    text = f.read_text()
    # Single pass prevents one replacement from matching another replacement's value.
    import re
    def replace(match):
        value = match.group(0)
        for old, new in replacements.items():
            value = value.replace(old, new)
        return value
    # Match C string literals only, leaving identifiers and expressions untouched.
    text = re.sub(r'"(?:\\.|[^"\\])*"', replace, text)
    f.write_text(text)
(root/'host_config.json').write_text(json.dumps(replacements, indent=2)+'\n')
