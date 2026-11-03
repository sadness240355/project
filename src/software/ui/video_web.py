#!/usr/bin/env python3
"""Browser preview of CRC-verified RDMA frames. No X11/DISPLAY required."""
import argparse
import errno
import fcntl
import ctypes
from video_pixels import to_image,BAYER
from PIL import Image
import io
import json
import mmap
import os
import signal
import struct
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, parse_qs
from video_console import Console, control, ROOT
import secrets
import subprocess
import sys
import urllib.request
import re
from video_viewer import snapshot,writer_active
_jpeg=ctypes.CDLL(str(Path(__file__).resolve().parent/'libvideo_jpeg.so'))
_jpeg.video_jpeg_encode.argtypes=[ctypes.c_char_p,ctypes.c_size_t,ctypes.c_uint,ctypes.c_uint,ctypes.c_uint,ctypes.c_int,ctypes.POINTER(ctypes.c_void_p),ctypes.POINTER(ctypes.c_ulong)]
_jpeg.video_jpeg_encode.restype=ctypes.c_int
_jpeg.video_jpeg_free.argtypes=[ctypes.c_void_p]


PAGE = (ROOT/'console.html').read_bytes()

class Frames:
    def __init__(self, args):
        self.args=args;self.condition=threading.Condition();self.stop=threading.Event()
        self.preview_skipped=0;self.encoded_skipped=0;self.jpeg=None;self.revision=0;self.last=0;self.width=0;self.height=0;self.seq=None
        self.present=False;self.changes=0;self.last_crc=None;self.times=deque(maxlen=90);self.error='waiting for receiver';self.job_id=0;self.last_job=0;self.jobs=set();self.jobs_lock=threading.Lock();self.pool=ThreadPoolExecutor(max_workers=4)
    def status(self):
        with self.condition:
            age=time.monotonic()-self.last if self.last else None
            live=self.present and age is not None and age<2
            fps=(len(self.times)-1)/(self.times[-1]-self.times[0]) if live and len(self.times)>1 else 0
            return dict(state='streaming' if live else 'waiting',width=self.width,height=self.height,
                        source_seq=self.seq,preview_frames=self.revision,content_changes=self.changes,preview_skipped=self.preview_skipped+self.encoded_skipped,
                        preview_fps=fps,target_fps=self.args.fps,side='Jetson' if os.uname().machine=='aarch64' else 'Mellanox',frame_age_s=age,error=self.error)
    def encode(self,job,seq,picture,crc,captured):
        pixels,w,h,fmt=picture
        if fmt==BAYER:
            im=to_image(pixels,w,h,fmt);pixels=im.tobytes();fmt=0x33424752
        if w>self.args.width:
            image=Image.frombytes('RGB',(w,h),pixels,'raw','RGB' if fmt==0x33424752 else 'BGR')
            image.thumbnail((self.args.width,4096));pw,ph=image.size;pixels=image.tobytes();fmt=0x33424752
        else:pw,ph=w,h
        out=ctypes.c_void_p();length=ctypes.c_ulong()
        if _jpeg.video_jpeg_encode(pixels,len(pixels),pw,ph,fmt,self.args.quality,ctypes.byref(out),ctypes.byref(length)):
            raise RuntimeError('JPEG encode failed')
        try:encoded=ctypes.string_at(out,length.value)
        finally:_jpeg.video_jpeg_free(out)
        with self.condition:
            if job<=self.last_job or self.stop.is_set():
                self.encoded_skipped+=1;return
            self.last_job=job;self.jpeg=encoded;self.revision+=1;self.seq=seq;self.width=w;self.height=h
            self.changes+=crc!=self.last_crc;self.last_crc=crc;self.last=captured;self.times.append(time.monotonic())
            self.condition.notify_all()
        if hasattr(self,"console"):self.console.on_frame(seq,picture,encoded,job)
    def completed(self,f):
        try:f.result()
        except Exception as e:
            with self.condition:self.error=str(e)
        with self.jobs_lock:self.jobs.discard(f)
    def run(self):
        mem=None;identity=None;previous=0;next_sample=time.monotonic()
        try:
            while not self.stop.is_set():
                started=time.monotonic()
                if started<next_sample:
                    self.stop.wait(min(.001,next_sample-started));continue
                got=None
                try:
                    st=os.stat(self.args.file);ident=(st.st_dev,st.st_ino,st.st_size)
                    if ident!=identity:
                        if mem:mem.close()
                        mem=None
                        if st.st_size<4096 or st.st_size>4096+2*16*1024*1024:raise ValueError('invalid mailbox length')
                        with open(self.args.file,'rb') as f:
                            if not writer_active(f.fileno()):raise ValueError('旧画面缓存无活动生产者，等待新采集会话')
                            mem=mmap.mmap(f.fileno(),0,access=mmap.ACCESS_READ)
                        identity=ident;previous=0
                    with self.jobs_lock:busy=len(self.jobs)>=4
                    got=None if busy else snapshot(mem,previous,raw=True)
                    with self.condition:self.present=True;self.error=''
                    if got:
                        pub,seq,picture,crc=got
                        if previous and pub>previous:self.preview_skipped+=max(0,pub-previous-1)
                        previous=pub;self.job_id+=1
                        next_sample=max(next_sample+1/self.args.fps,started-1/self.args.fps)
                        future=self.pool.submit(self.encode,self.job_id,seq,picture,crc,time.monotonic())
                        with self.jobs_lock:self.jobs.add(future)
                        future.add_done_callback(self.completed)
                except (OSError,ValueError,struct.error) as e:
                    with self.condition:self.present=False;self.error=str(e)
                    next_sample=max(next_sample,started+.05)  # No producer: poll at 20 Hz, not a busy loop.
                self.stop.wait(.0002)
        finally:
            if mem:mem.close()
            self.pool.shutdown(wait=True)

class Server(ThreadingHTTPServer):
    daemon_threads=True
    allow_reuse_address=True

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):pass
    def send(self, code, content, kind):
        self.send_response(code);self.send_header('Content-Type',kind)
        self.send_header('Content-Length',str(len(content)));self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff');self.end_headers();self.wfile.write(content)
    def do_GET(self):
        frames=self.server.frames;route=urlsplit(self.path).path
        try:
            if route=='/':self.send(200,PAGE,'text/html; charset=utf-8')
            elif route=='/dual':self.send(200,(ROOT/'dual.html').read_bytes(),'text/html; charset=utf-8')
            elif route=='/status':
                d=frames.status();d.update(console=self.server.console.status(),csrf=self.server.console.csrf)
                self.send(200,json.dumps(d).encode(),'application/json')
            elif route in ('/console.css','/console.js'):
                self.send(200,(ROOT/route[1:]).read_bytes(),'text/css' if route.endswith('.css') else 'text/javascript')
            elif route=='/api/local/status':
                if not self.peer_authorized():return
                self.send_json(200,self.server.console.local_status())
            elif route=='/api/saved':self.send_json(200,self.server.console.saved())
            elif route=='/api/analysis':
                q=parse_qs(urlsplit(self.path).query)
                self.send_json(200,self.server.console.analysis(q.get('name',[''])[0],q.get('x',[0])[0],q.get('y',[0])[0]))
            elif re.fullmatch(r'/api/history/[0-9]+\.jpg',route):
                ident=int(route.rsplit('/',1)[1][:-4]);data=None
                with self.server.console.lock:
                    for item in self.server.console.history:
                        if item['id']==ident:data=item['jpeg'];break
                self.send(200 if data else 404,data or b'Frame expired','image/jpeg' if data else 'text/plain')
            elif re.fullmatch(r'/captures/(snapshot_[0-9_]+\.png|desktop_raw_[0-9_]+\.png|desktop_preview_[0-9_]+\.jpg|preview_[0-9_]+\.jpg|sequence_[0-9a-f_]+/frame_[0-9_]+\.jpg)',route):
                self.send(200,(ROOT/route[1:]).read_bytes(),'image/png' if route.endswith('.png') else 'image/jpeg')
            elif route=='/frame.jpg':
                with frames.condition:
                    data=frames.jpeg if frames.status()['state']=='streaming' else None
                self.send(200 if data else 503,data or b'Waiting for a fresh RDMA frame','image/jpeg' if data else 'text/plain')
            elif route=='/stream.mjpg':
                self.connection.settimeout(10)
                self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control','no-store');self.end_headers();previous=0
                while not frames.stop.is_set():
                    with frames.condition:
                        frames.condition.wait_for(lambda:frames.revision!=previous or frames.stop.is_set(),timeout=2)
                        if frames.stop.is_set() or frames.status()['state']!='streaming':return
                        if previous==frames.revision:continue
                        previous=frames.revision;data=frames.jpeg
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(data)).encode()+b'\r\n\r\n'+data+b'\r\n');self.wfile.flush()
            else:self.send(404,b'Not found','text/plain')
        except (BrokenPipeError,ConnectionResetError,TimeoutError):pass
        except (ValueError,OSError,RuntimeError) as e:self.send_json(400,{'ok':False,'error':str(e)})
    def send_json(self,code,data):self.send(code,json.dumps(data,ensure_ascii=False).encode(),'application/json; charset=utf-8')
    def peer_authorized(self):
        if not secrets.compare_digest(self.headers.get('X-Station-Key',''),self.server.console.token):
            self.send_json(403,{'ok':False,'error':'对端控制认证失败'});return False
        return True
    def do_POST(self):
        route=urlsplit(self.path).path
        try:
            if route in ('/api/local/control','/api/peer/pipeline','/api/peer/test'):
                if not self.peer_authorized():return
            else:
                origin=self.headers.get('Origin','')
                if origin and urlsplit(origin).netloc!=self.headers.get('Host'):
                    self.send_json(403,{'ok':False,'error':'跨站控制被拒绝'});return
                if not secrets.compare_digest(self.headers.get('X-Console-Token',''),self.server.console.csrf):
                    self.send_json(403,{'ok':False,'error':'请刷新页面后重试'});return
            length=int(self.headers.get('Content-Length','0'))
            if not 0<length<=4096:raise ValueError('请求长度无效')
            d=json.loads(self.rfile.read(length));c=self.server.console
            if route=='/api/local/control':result=control(d)
            elif route in ('/api/pipeline','/api/peer/pipeline'):
                if d.get('action') not in ('start','stop','reconfigure'):raise ValueError('无效的采集操作')
                result=c.launch(d['action'],d.get('config',{}))
            elif route in ('/api/test','/api/peer/test'):result=c.test_command(d.get('action'),d.get('config',{}))
            elif route=='/api/yolo':result=c.yolo_toggle(d.get('enabled'))
            elif route=='/api/isp/live':result=c.live_isp(d.get('values',{}))
            elif route=='/api/camera/read':result=c.camera_read()
            elif route=='/api/snapshot':result=c.save()
            elif route=='/api/history/save':result=c.save_history(d.get('id',-1))
            elif route=='/api/config':result=c.config(d)
            elif route=='/api/record':result=c.record(d.get('start') is True)
            else:self.send_json(404,{'ok':False,'error':'未知操作'});return
            self.send_json(200,{'ok':True,**result})
        except (Exception,) as e:self.send_json(400,{'ok':False,'error':str(e)})

def bind_when_address_ready(bind, port):
    # station may start before DHCP assigns the management address. Keep the
    # supervisor/web process alive until the requested address can be bound.
    wait_s = float(os.environ.get('LRDMA_WEB_BIND_WAIT_S', '120'))
    deadline = time.monotonic() + max(0.0, wait_s)
    announced = False
    while True:
        try:
            return Server((bind, port), Handler)
        except OSError as exc:
            if exc.errno != errno.EADDRNOTAVAIL or time.monotonic() >= deadline:
                raise
            if not announced:
                print(f'Waiting for management address {bind} before binding port {port}', flush=True)
                announced = True
            time.sleep(0.5)


def main():
    root=Path(__file__).resolve().parent
    ap=argparse.ArgumentParser(description=__doc__)
    jetson=os.uname().machine=='aarch64'
    ap.add_argument('--file',default='/dev/shm/fpga_video_input' if jetson else str(root/'runtime/display_memory/latest_frames.bin'))
    ap.add_argument('--bind',default=os.environ.get('STATION_LOCAL_IP','10.23.68.0' if jetson else '10.23.66.188'));ap.add_argument('--port',type=int,default=8788)
    ap.add_argument('--fps',type=int,default=120);ap.add_argument('--width',type=int,default=1920)
    ap.add_argument('--background',action='store_true');ap.add_argument('--quality',type=int,default=80);ap.add_argument('--stop',action='store_true')
    args=ap.parse_args();pidfile=root/'web.pid'
    if args.background:
        with (root/'console_web.log').open('a') as f:
            child=subprocess.Popen([sys.executable,str(root/'video_web.py')]+[a for a in sys.argv[1:] if a!='--background'],cwd=root,stdin=subprocess.DEVNULL,stdout=f,stderr=f,start_new_session=True)
        time.sleep(0.5)
        if child.poll() is not None:
            try:
                opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
                with opener.open(f'http://{args.bind}:{args.port}/status',timeout=2) as response:
                    if response.status==200:
                        print('工作站已运行：http://'+args.bind+':'+str(args.port)+'/');return 0
            except (OSError,ValueError):
                pass
            print('界面服务启动失败，查看 '+str(root/'console_web.log'),file=sys.stderr)
            return 1
        print('工作站启动中：http://'+args.bind+':'+str(args.port)+'/');return 0
    if args.stop:
        try:
            pid=int(pidfile.read_text());cmd=Path('/proc')/str(pid)/'cmdline'
            if str(root/'video_web.py').encode() not in cmd.read_bytes().split(b'\0'):raise ValueError('stale PID')
            os.kill(pid,signal.SIGTERM);print('Requested web preview stop');return 0
        except (OSError,ValueError):print('Web preview is not running');return 0
    if not(1<=args.fps<=120 and 1<=args.quality<=95 and 32<=args.width<=4096 and 1<=args.port<=65535):ap.error('invalid fps, quality, width or port')
    lock=pidfile.open('a+')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('Web preview already running; use existing URL or --stop');return 1
    lock.seek(0);lock.truncate();lock.write(str(os.getpid()));lock.flush()
    frames=Frames(args)
    try:server=bind_when_address_ready(args.bind,args.port)
    except OSError as e:print('Cannot start preview:',e);return 1
    server.frames=frames
    server.console=Console(frames);frames.console=server.console
    worker=threading.Thread(target=frames.run,daemon=True);worker.start()
    def stop(*_):
        frames.stop.set()
        with frames.condition:frames.condition.notify_all()
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    print(f'WEB ready: http://{args.bind}:{args.port}/ ; waiting page is available before frames arrive',flush=True)
    try:server.serve_forever(poll_interval=.2)
    finally:
        frames.stop.set();server.console.stop.set();worker.join(timeout=3);server.server_close();lock.close()
    return 0
if __name__=='__main__':raise SystemExit(main())
