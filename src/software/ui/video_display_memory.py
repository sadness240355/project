#!/usr/bin/env python3
"""Prepare a RAM-only display FIFO under this project, never on the system disk."""
import os, subprocess, fcntl, stat
from pathlib import Path
ROOT=Path(__file__).resolve().parent
QUEUE=ROOT/'runtime/display_memory'
def clear_inactive(path, lock_path=None):
 # Only cache files are removed. DDR ownership and DMA allocations are untouched.
 if not path.exists():return False
 target=lock_path or path
 
 try:fd=os.open(target,(os.O_RDWR if lock_path else os.O_RDONLY)|os.O_NOFOLLOW|os.O_CLOEXEC)
 except FileNotFoundError:return False
 try:
  try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
  except BlockingIOError:return False
  current=path.lstat()
  if not stat.S_ISREG(current.st_mode):return False
  if not lock_path:
   held=os.fstat(fd)
   if (held.st_dev,held.st_ino)!=(current.st_dev,current.st_ino):return False
  path.unlink()
  return True
 finally:
  fcntl.flock(fd,fcntl.LOCK_UN);os.close(fd)
def prepare():
 QUEUE.mkdir(parents=True,exist_ok=True)
 kind=subprocess.check_output(['findmnt','-n','-o','FSTYPE','-T',str(QUEUE)],text=True).strip()
 if kind!='tmpfs':
  if os.path.ismount(QUEUE):raise RuntimeError('显示缓存目录已被其他文件系统占用')
  if os.geteuid()!=0:raise RuntimeError('首次建立内存显示队列需要通过 ./video.sh station 启动后台（程序会请求 sudo）')
  owner=ROOT.stat()
  subprocess.run(['mount','-t','tmpfs','-o',f'size=1536M,mode=0700,uid={owner.st_uid},gid={owner.st_gid}','lrdma-display',str(QUEUE)],check=True)
 if subprocess.check_output(['findmnt','-n','-o','FSTYPE','-T',str(QUEUE)],text=True).strip()!='tmpfs':
  raise RuntimeError('显示队列必须位于 tmpfs，拒绝把实时视频写入磁盘')
 clear_inactive(QUEUE/'frames.bin',QUEUE/'frames.bin.producer.lock')
 clear_inactive(QUEUE/'latest_frames.bin')
 return QUEUE/'frames.bin'
if __name__=='__main__':print(prepare())
