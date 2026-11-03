#!/usr/bin/env python3
"""Cross-check newly captured host, sender and receiver records; no old data."""
import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('output')
p.add_argument('sessions', nargs='+')
a = p.parse_args()
root = Path(__file__).resolve().parent.parent
out = root / 'data' / a.output
out.mkdir(exist_ok=False)
records = []
evidence = {}
for sid in a.sessions:
    paths = {role: root / 'board/results' / (sid + '_' + role)
             for role in ['owner', 'tx', 'rx', 'orchestration']}
    for role in ['owner', 'tx', 'rx']:
        assert (paths[role] / 'exit_code.txt').read_text().strip() == '0'
    assert (paths['orchestration'] / 'owner_exit_code.txt').read_text().strip() == '0'
    assert (paths['orchestration'] / 'sender_exit_code.txt').read_text().strip() == '0'
    host = json.loads((paths['owner'] / 'result.json').read_text())
    logs = {role: (paths[role] / 'run.stderr').read_text() for role in ['owner', 'tx', 'rx']}
    assert all('FAIL:' not in s and 'SAFETY_HOLD' not in s for s in logs.values())
    rows = list(csv.DictReader((paths['owner'] / 'frames.csv').open()))
    assert len(rows) == host['frames']
    ids = [int(row['source_frame_seq']) for row in rows]
    assert all(b == a + 1 for a, b in zip(ids, ids[1:]))
    assert all(int(row['bytes']) == 14745600 for row in rows)
    assert host['source_sequence_gaps'] == host['hardware_drop_count'] == 0
    tx = re.search(r'PASS: video frames=(\d+) bytes_per_frame=(\d+) source_skipped=(\d+) seconds=([\d.]+)', logs['tx'])
    rx = re.search(r'PASS: received and CRC-verified (\d+) video frames; last_seq=(\d+) crc_errors=(\d+) boxes_received=(\d+)', logs['rx'])
    final = re.search(r'DDR_FINAL published=(\d+) released=(\d+) sender_state=(\d+)', logs['owner'])
    drained = int(re.search(r'DRAINED_AFTER_MEASUREMENT=(\d+)', logs['owner'])[1])
    assert tx and rx and final
    n = int(tx[1])
    assert n == int(rx[1]) == int(rx[2]) == int(final[1]) == int(final[2])
    assert n == len(rows) + 1 + drained  # one pre-window warmup frame
    assert int(tx[2]) == 14745600 and int(tx[3]) == int(rx[3]) == 0
    assert int(final[3]) == 2
    assert 'crc_mode=full' in logs['rx']
    assert 'CLEAN_STOP gated=1 host_idle=1 ddr_empty=1' in logs['owner']
    assert 'VIDEO_QPS_RETIRED_RDMA_DISABLED_BEFORE_POOL_RELEASE' in logs['tx']
    idle = json.loads((paths['orchestration'] / 'idle_after.json').read_text())
    assert idle['status'] == 'BOTH_ENDPOINTS_AND_FPGA_IDLE'
    fps = len(rows) * 1e9 / host['elapsed_ns']
    records.append({'session': sid, 'measurement_frames': len(rows),
                    'measurement_seconds': host['elapsed_ns'] / 1e9,
                    'host_delivered_FPS': fps,
                    'host_measurement_payload_Gbit_s': fps * 14745600 * 8 / 1e9,
                    'all_session_remote_full_CRC_frames': n,
                    'remote_CRC_errors': 0, 'source_sequence_gaps_in_window': 0,
                    'hardware_drops_in_window': 0,
                    'sender_session_seconds_including_start_stop': float(tx[4]),
                    'sender_session_payload_Gbit_s': n * 14745600 * 8 / float(tx[4]) / 1e9,
                    'warmup_frames': 1, 'drained_after_window': drained,
                    'boxes_received': int(rx[4]), 'clean_stop': True})
    for directory in paths.values():
        for f in directory.rglob('*'):
            if f.is_file():
                evidence[str(f.relative_to(root))] = hashlib.sha256(f.read_bytes()).hexdigest()
result = {'status': 'FRESH_DUAL_CAMERA_HOST_AND_REMOTE_FULL_CRC_PASS',
          'bitstream_sha256': '3e0eb779522e7252b3cf74a948b188ed1610e2761c9254bc41c7c4404eac725e',
          'sender_candidate': 'board/application_build/video_sender_final',
          'records': records,
          'limitations': [
              'Isolated software candidate; frozen production inputs are unchanged.',
              'Full-frame software CRC and preview publication are enabled; results do not establish 140 FPS.',
              'Window counts exclude warmup and shutdown; receiver total includes both.',
              'Drop and source-gap zero claims are limited to the measurement window.',
              'No inference boxes were generated; empty box-channel teardown is not an inference test.',
              'Software timestamps do not establish sensor exposure-to-display latency or cycle-accurate SOF timing.',
              'CRC compares both branches for the same source frame; it does not establish independent image-quality correctness.'
          ]}
(out / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
(out / 'evidence.sha256').write_text(''.join(f'{v}  {k}\n' for k, v in sorted(evidence.items())))
(out / 'summarizer_used.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps(result, indent=2))
