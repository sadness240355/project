#!/usr/bin/env python3
"""Own only the detector process and its session-specific SHM, never DMA."""
import runpy,signal
from pathlib import Path
def stop(*_):raise SystemExit(0)
signal.signal(signal.SIGTERM,stop)
runpy.run_path(str(Path(__file__).resolve().parent/'yolo/track_shm.py'),run_name='__main__')
