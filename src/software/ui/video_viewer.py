#!/usr/bin/env python3
"""Display the receiver's verified CSHM v3 frames on the Mellanox x86 host."""
import argparse
import mmap
import os
import struct
import time
import zlib
import ctypes
import fcntl
try:
    _crc=ctypes.CDLL(os.path.join(os.path.dirname(__file__),"libvideo_crc.so")).video_crc32
    _crc.argtypes=[ctypes.c_char_p,ctypes.c_size_t];_crc.restype=ctypes.c_uint32
except OSError:
    _crc=None
from PIL import Image
from video_pixels import to_image,BAYER

_fence=ctypes.CDLL(os.path.join(os.path.dirname(__file__),"libvideo_jpeg.so")).video_memory_fence
_fence.argtypes=[];_fence.restype=None

def writer_active(fd):
    # Producers hold an exclusive flock for the lifetime of each mailbox.
    # An unlocked old file is historical data, never a new live frame.
    try: fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError: return True
    fcntl.flock(fd, fcntl.LOCK_UN)
    return False

def snapshot(mem, previous, raw=False, metadata=False):
    magic, version, page, fmt, w, h, stride, size, slots = struct.unpack_from('<9I', mem)
    if (magic, version, page, slots) != (0x4d485343, 3, 4096, 2):
        raise ValueError('Invalid CSHM v3 header')
    if fmt not in (0x33424752, 0x33524742, BAYER) or not (0 < w <= 4096 and 0 < h <= 4096) or stride != w*(1 if fmt==BAYER else 3) or (fmt==BAYER and (w%2 or h%2)) or size != stride*h or len(mem) != page+2*size:
        raise ValueError('Invalid RGB geometry')
    pub = struct.unpack_from('<Q', mem, 40)[0]
    slot = struct.unpack_from('<I', mem, 48)[0]
    if not pub or pub == previous or slot != pub % 2:
        return None
    guard = struct.unpack_from('<Q', mem, 56+8*slot)[0]
    if guard % 2 or guard != 2*((pub+1)//2):
        return None
    _fence()  # Acquire data after the producer's slot guard, including on ARM.
    seq = struct.unpack_from('<Q', mem, 72+8*slot)[0]
    stamp=struct.unpack_from('<Q',mem,88+8*slot)[0]
    boxes=mem[512+slot*1024:512+(slot+1)*1024] if metadata else None
    pixels = mem[page+slot*size:page+(slot+1)*size]
    _fence()  # Finish all payload reads before rechecking the slot guard.
    # A newer publication in the other slot does not invalidate this stable slot.
    if struct.unpack_from('<Q', mem, 56+8*slot)[0] != guard:
        return None
    image = (pixels,w,h,fmt) if raw else to_image(pixels,w,h,fmt)
    result=(pub,seq,image,(_crc(pixels,len(pixels)) if _crc else zlib.crc32(pixels)))
    return result+(stamp,boxes) if metadata else result

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--file', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'received_frames.bin'))
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--seconds', type=float, default=0)
    ap.add_argument('--save', help='save last displayed image')
    args = ap.parse_args()
    path=os.path.abspath(args.file)
    import tkinter as tk
    from PIL import ImageTk
    if not args.headless and not os.environ.get('DISPLAY'):
        print('当前终端没有图形桌面。远程浏览器预览请运行 ./video.sh web，然后访问 http://10.23.66.188:8788/')
        return 2
    root = None if args.headless else tk.Tk()
    if root:
        root.title('ERNIC RDMA camera: waiting for verified frames')
        label=tk.Label(root, text='Waiting for verified camera frames...'); label.pack()
    mem=None; inode=None; previous=0; seen=0; changes=0; last_crc=None; last=None; started=time.monotonic(); last_update=started
    try:
        while not args.seconds or time.monotonic()-started < args.seconds:
            if root: root.update()
            try:
                st=os.stat(path)
                ident=(st.st_dev, st.st_ino)
                if ident != inode:
                    if mem: mem.close()
                    with open(path,'rb') as f: mem=mmap.mmap(f.fileno(),0,access=mmap.ACCESS_READ)
                    inode=ident;previous=0
                got=snapshot(mem,previous)
            except (OSError,ValueError,struct.error):
                got=None
            if got:
                previous,seq,last,crc=got;seen+=1;changes+=crc!=last_crc;last_crc=crc;last_update=time.monotonic()
                if root:
                    shown=last.copy();shown.thumbnail((1280,720))
                    photo=ImageTk.PhotoImage(shown);label.configure(image=photo,text='');label.image=photo
                    root.title(f'ERNIC RDMA {last.width}x{last.height} | frame {seq} | displayed {seen}')
                if seen==1 or seen%30==0: print(f'VIEW frames={seen} content_changes={changes} source_seq={seq} crc={crc:08x}',flush=True)
            elif root and time.monotonic()-last_update>2:
                root.title('ERNIC RDMA camera: waiting / stale frame')
            time.sleep(.005)
    except (KeyboardInterrupt,tk.TclError):
        pass
    finally:
        if args.save and last: last.save(args.save)
        if mem: mem.close()
        print(f'VIEW complete frames={seen} content_changes={changes}; display sampling can skip received frames',flush=True)
    return 0 if seen else 1
if __name__=='__main__': raise SystemExit(main())
