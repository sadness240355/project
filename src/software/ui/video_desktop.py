#!/usr/bin/env python3
"""Native Tk workstation. Raw video is read locally; HTTP carries control/status only."""
import argparse,fcntl,io,json,mmap,os,queue,signal,threading,time,urllib.request,urllib.error
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from video_config import resolution_pair,automatic_exposure_limit_us,available_resolutions
from pathlib import Path
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk,messagebox
from PIL import Image,ImageTk,ImageStat,ImageGrab
from video_viewer import snapshot,writer_active
from video_stages import stage_counts,stage_detail,STAGE_COUNT_GUIDE
from video_analysis import analyze_frame
from video_errors import fault_report,event_text
from video_isp import read_isp_mask,read_isp_selection,save_isp_mask
from video_config import validate_capture,frame_bytes,STREAMS,RESOLUTIONS,FRAME_RATES
from video_pixels import to_image,BAYER,format_name
from video_gpu import Renderer,screen_rates,gpu_failure_detail
from video_panels import sidebar,test_panel,processing_panel
from video_bandwidth import BandwidthHistory
ROOT=Path(__file__).resolve().parent
JETSON=os.uname().machine=='aarch64'
os.environ.setdefault('VIDEO_DISPLAY_VSYNC','0')
LOCAL=os.environ.get('STATION_LOCAL_IP','10.23.68.0' if JETSON else '10.23.66.188')
SIDE='Jetson 发送端' if JETSON else 'Mellanox 接收端'
VISIBLE_TITLE=('异构边缘智能感知与高速传输控制软件' if JETSON
               else 'RDMA 远程数据管理与可视化监控软件')
RESAMPLE=getattr(Image,'Resampling',Image)
FONT_SCALE=1.0
UI_SCALE=1.0
def font_px(n):return -max(1,round(n*UI_SCALE/FONT_SCALE))
BG='#0b1220';PANEL='#142035';TEXT='#e4eef9';MUTED='#8ca2bd';ACCENT='#59d5c6'

class RawReader:
 def __init__(self,path,fps):
  self.path=path;self.fps=fps;self.stop=threading.Event();self.lock=threading.Lock()
  self.latest=None;self.error='等待完整帧';self.epoch=0;self.pause=False
  self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
 def run(self):
  mem=None;identity=None;previous=0
  try:
   while not self.stop.is_set():
    start=time.monotonic()
    try:
     path=self.path;st=path.stat();ident=(str(path),st.st_dev,st.st_ino,st.st_size)
     if ident!=identity:
      if mem:mem.close()
      mem=None
      if not 4096<=st.st_size<=4096+2*16*1024**2:raise ValueError('无效图像缓冲大小')
      with path.open('rb') as f:
       if not writer_active(f.fileno()):raise ValueError('旧画面缓存无活动生产者，等待新采集会话')
       mem=mmap.mmap(f.fileno(),0,access=mmap.ACCESS_READ)
      identity=ident;previous=0;self.epoch+=1
     if not self.pause:
      got=snapshot(mem,previous,raw=True,metadata=True)
      if got:
       pub,seq,(pixels,w,h,fmt),crc,stamp,boxes=got
       if previous and pub<previous:self.epoch+=1
       previous=pub
       im=to_image(pixels,w,h,fmt)
       revision=0;shown=im
       with self.lock:self.latest=(self.epoch,pub,seq,im,crc,time.monotonic(),shown,revision,fmt,pixels if fmt==BAYER else None,stamp,boxes);self.error=''
    except (OSError,ValueError) as e:
     self.error=str(e)
    self.stop.wait(max(.001,1/max(1,self.fps)-(time.monotonic()-start)))
  finally:
   if mem:mem.close()

class Workstation:
 def __init__(self,root,args):
  self.root=root;self.args=args;self.pool=ThreadPoolExecutor(max_workers=3);self.results=queue.Queue()
  self.pending_futures=set();self.future_lock=threading.Lock()
  self.closed=threading.Event();self.preview_configured=False;self.token='';self.status={};self.last_status=0;self.net_error='';self.window_error=''
  self.current=None;self.current_raw=True;self.stream_epoch=None;self.frozen=False;self.last_draw_key=None;self.display_epoch=None;self.display_pub=None
  self.display_count=0;self.display_skipped=0;self.display_times=deque(maxlen=120)
  self.bandwidth=BandwidthHistory()
  self.last_cache=0;self.cache=deque();self.history_limit=16;self.history_bytes=0;self.history_meta={}
  self.raw_path=Path('/dev/shm/fpga_video_input') if JETSON else ROOT/'runtime/display_memory/latest_frames.bin'
  self.display_path=ROOT/'runtime/display_memory/frames.bin'
  self.reader=RawReader(self.raw_path,2)
  self.last_gpu_report=0
  self.gpu=None;self.gpu_failed=False;self.gpu_stats={};self.last_gpu_update=0
  self.zoom='适应窗口';self.photo=None;self.drawn=None;self.dirty=True;self.busy=False;self.analysis_version=0
  self.analysis_busy=False;self.analysis_pending=None;self.last_analysis_time=0;self.last_analysis_key=None;self.analysis_updates=0;self.current_analysis=None;self.selected_point=None;self.test_state=0;self.test_started=time.monotonic()
  self.saved_before=set((ROOT/'captures').glob('*.png'));self.test_results={}
  from video_boxes import BoxReader
  self.box_reader=BoxReader()
  self.reconfigure_after=None;self.reconfigure_hold=False;self.reconfigure_was_frozen=False
  self.config_session=None;self.stop_pending=False;self.resolution_dirty=False;self.reconfigure_queued=False;self.yolo_pending=False;self.isp_sync=False;self.live_isp_busy=False;self.live_isp_after=None;self.live_isp_queued=None
  self.build()
  from video_responsive import Responsive
  def apply_scale(value):
   global UI_SCALE
   UI_SCALE=value
   self.checkmarks.resize(value)
  self.responsive=Responsive(self,apply_scale)
  root.after(100,self.init_gpu)
  threading.Thread(target=self.poll_status,daemon=True).start()
  root.after(10,self.tick);root.after(200,self.poll_raise);root.after(300,self.bring_to_front)
  root.protocol('WM_DELETE_WINDOW',self.close)
  if args.self_test:root.after(1000,self.self_test)
  if args.ui_check:root.after(4500,self.ui_snapshot)
  if args.console_check:self.console_check_started=time.monotonic();self.console_check_phase=0;root.after(1000,self.console_check)
  if args.analysis_check:
   self.analysis_check_start=time.monotonic();self.analysis_check_phase=0;root.after(500,self.analysis_check)
 def build(self):
  r=self.root;r.title(VISIBLE_TITLE+' · '+SIDE)
  r.configure(bg=BG);r.geometry('%dx%d'%(min(1660,r.winfo_screenwidth()-60),min(1020,r.winfo_screenheight()-70)));r.minsize(1120,730)
  if not JETSON:
   # NVIDIA multi-monitor vsync otherwise follows the first (possibly slower) screen.
   try:
    import subprocess,re
    text=subprocess.check_output(['xrandr','--current'],text=True,timeout=2)
    screens=re.findall(r'^(\S+) connected (primary )?(\d+)x(\d+)\+(-?\d+)\+(-?\d+)',text,re.M)
    preferred=os.environ.get('VIDEO_DISPLAY_DEVICE')
    selected=next((x for x in screens if x[0]==preferred),None) if preferred else next((x for x in screens if x[1]),None)
    if selected:
     dev,_,sw,sh,sx,sy=selected;sw,sh,sx,sy=map(int,(sw,sh,sx,sy))
     os.environ.setdefault('__GL_SYNC_DISPLAY_DEVICE',dev)
     self.display_center=(sx+sw//2,sy+sh//2)
     w,h=min(1660,sw-60),min(1020,sh-70)
     r.geometry('%dx%d+%d+%d'%(w,h,sx+(sw-w)//2,sy+(sh-h)//2))
   except (OSError,ValueError,subprocess.SubprocessError):pass
  s=ttk.Style(r);s.theme_use('clam')
  s.configure('.',background=PANEL,foreground=TEXT,font=('Noto Sans CJK SC',font_px(13)))
  s.configure('TFrame',background=PANEL);s.configure('TLabel',background=PANEL,foreground=TEXT)
  s.configure('Muted.TLabel',foreground=MUTED,font=('Noto Sans CJK SC',font_px(12)))
  s.configure('Title.TLabel',font=('Noto Sans CJK SC',font_px(16),'bold'))
  s.configure('TButton',background='#243951',foreground=TEXT,borderwidth=0,padding=(9,7));s.map('TButton',background=[('active','#355774')])
  s.configure('TCheckbutton',background=PANEL,foreground=TEXT,padding=3);s.map('TCheckbutton',background=[('active',PANEL)],foreground=[('disabled',MUTED)])
  from video_checkmarks import Checkmarks
  self.checkmarks=Checkmarks(r,s,PANEL,ACCENT,MUTED)
  s.configure('Start.TButton',background='#197e73',foreground='#edfff9');s.map('Start.TButton',background=[('active','#239b8c')])
  s.configure('TEntry',fieldbackground='#0d1829',foreground=TEXT,padding=5)
  s.configure('TSpinbox',fieldbackground='#0d1829',foreground=TEXT,arrowcolor=TEXT,padding=4)
  s.map('TSpinbox',fieldbackground=[('disabled','#17283e')],foreground=[('disabled',MUTED)])
  s.configure('TCombobox',fieldbackground='#0d1829',foreground=TEXT,padding=4);s.map('TCombobox',fieldbackground=[('readonly','#0d1829')],foreground=[('readonly',TEXT)])
  s.configure('TNotebook',background=BG,borderwidth=0)
  s.configure('TNotebook.Tab',background='#192b43',padding=(8,8));s.map('TNotebook.Tab',background=[('selected','#294962')],foreground=[('selected',ACCENT)])
  s.configure('Treeview',background='#101b2c',fieldbackground='#101b2c',foreground='#c8d9ec',rowheight=28,borderwidth=0,font=('Noto Sans CJK SC',font_px(12)))
  s.configure('Treeview.Heading',background='#20354e',foreground='#9fbbd8',font=('Noto Sans CJK SC',font_px(12)))
  s.map('Treeview',background=[('selected','#275273')])
  head=tk.Frame(r,bg=BG);head.pack(fill='x',padx=20,pady=(14,12))
  tk.Label(head,text=VISIBLE_TITLE,font=('Noto Sans CJK SC',font_px(21),'bold'),bg=BG,fg=ACCENT).pack(side='left')
  tk.Label(head,text='  /  '+SIDE+' · 本地桌面',bg=BG,fg=MUTED,font=('Noto Sans CJK SC',font_px(15))).pack(side='left',padx=15)
  self.connection=tk.Label(head,text='连接控制服务…',bg=BG,fg='#e3b774',font=('Noto Sans CJK SC',font_px(13)));self.connection.pack(side='right')
  cards=tk.Frame(r,bg=BG);cards.pack(fill='x',padx=20,pady=(0,12));self.metrics={}
  for key,title in [('resolution','采集分辨率'),('capture','有效采集 / PCIe'),('receive','RDMA 接收'),('display','逐帧呈现提交'),('sent_skip','发送取帧跳过'),('crc','接收 CRC 错帧')]:
   box=tk.Frame(cards,bg='#17283e',padx=13,pady=9);box.pack(side='left',fill='x',expand=True,padx=(0,8))
   tk.Label(box,text=title,bg='#17283e',fg=MUTED,font=('Noto Sans CJK SC',font_px(12))).pack(anchor='w')
   v=tk.Label(box,text='—',bg='#17283e',fg=TEXT,font=('Noto Sans CJK SC',font_px(25),'bold'));v.pack(anchor='w');self.metrics[key]=v
  body=tk.Frame(r,bg=BG);body.pack(fill='both',expand=True,padx=20)
  left=sidebar(self,body,PANEL)
  self.vars={}
  ttk.Label(left,text='设备与采集',style='Title.TLabel').pack(anchor='w',pady=(0,8))
  self.field(left,'source','数据源','Camera Link 相机',['Camera Link 相机','相机内置测试图'])
  self.field(left,'model','相机型号','未读取',['未读取'])
  self.field(left,'camera','采集通道','Camera Link · 通道 0',['Camera Link · 通道 0'])
  self.field(left,'mode','采集模式','连续采集',['连续采集'])
  self.field(left,'stream','输出像素格式',next(iter(STREAMS)),list(STREAMS))
  self.stream_note=ttk.Label(left,text='',style='Muted.TLabel',wraplength=300);self.stream_note.pack(fill='x',pady=7)
  ttk.Separator(left).pack(fill='x',pady=8)
  if JETSON:
   self.isp_var=tk.BooleanVar(value=bool(read_isp_mask()))
   self.isp_check=ttk.Checkbutton(left,text='启用硬件 ISP（总开关）',variable=self.isp_var,command=self.set_isp);self.isp_check.pack(anchor='w',pady=(5,3))
   self.isp_label=ttk.Label(left,text='黑电平 / 白平衡 / CCM / Gamma',style='Muted.TLabel',wraplength=300);self.isp_label.pack(fill='x')
  else:
   ttk.Label(left,text='ISP 色彩校正由发送端设置',style='Muted.TLabel').pack(anchor='w')
  ttk.Label(left,text='相机参数',style='Title.TLabel').pack(anchor='w',pady=(15,4))
  self.field(left,'size','分辨率 / 宽×高','1920×1080',['%d×%d'%size for size in available_resolutions()])
  self.field(left,'fps','采集目标 / fps','120',[str(n) for n in reversed(FRAME_RATES)])
  self.field(left,'exposure_us','曝光时间 / μs','7500')
  self.field(left,'pattern','诊断图案 / 测试源专用','灰度条',['灰度条','彩色条','棋盘格'])
  self.pattern_note=ttk.Label(left,text='',style='Muted.TLabel',wraplength=345);self.pattern_note.pack(fill='x',pady=(2,5))
  self.camera_info=ttk.Label(left,text='读取型号与可用模式：回读相机与当前链路上限，筛选可选分辨率；不会启动采集。',style='Muted.TLabel',wraplength=345);self.camera_info.pack(fill='x',pady=9)
  self.auto_parameter_note=ttk.Label(left,text='ISP 实时更新需新 bit；尺寸/曝光/直写缓冲/扩展参数自动短暂停采重配。',style='Muted.TLabel',wraplength=345);self.auto_parameter_note.pack(fill='x',pady=7)
  self.yolo_var=tk.BooleanVar(value=False)
  self.yolo_check=ttk.Checkbutton(left,text='YOLO 人员检测 · 不阻塞视频',variable=self.yolo_var,command=self.toggle_yolo);self.yolo_check.pack(anchor='w',pady=6)
  self.yolo_note=ttk.Label(left,text='引擎：yolo11n 640×384 · 视频与推理帧率独立',style='Muted.TLabel',wraplength=345);self.yolo_note.pack(fill='x',pady=4)
  self.vars['source'].trace_add('write',lambda *_:self.source_changed())
  perf=self.performance_page
  ttk.Label(perf,text='传输与性能',style='Title.TLabel').pack(anchor='w',pady=(5,10))
  ttk.Label(perf,text='主视频按序逐帧呈现；图像分析与 YOLO 可独立低频采样。',style='Muted.TLabel',wraplength=300).pack(fill='x',pady=(0,6))
  self.field(perf,'buffers','主机直写缓冲数','16',['16','8','4','2','32'])
  ttk.Label(perf,text='整帧 CRC32：发送端逐帧计算，接收端逐帧复核。',style='Muted.TLabel',wraplength=300).pack(fill='x',pady=5)
  self.crc_checked_note=ttk.Label(perf,text='等待逐帧 CRC 校验',style='Muted.TLabel',wraplength=300)
  self.crc_checked_note.pack(fill='x',pady=(0,8))
  self.field(perf,'display_fps','显示模式','跟随采集',['跟随采集'])
  ttk.Label(perf,text='直写缓冲：2/4/8/16/32，2400×2048@140 RGB 至少 16；普通视频不提交 D2H DMA。\n每帧按序交给 GPU，不覆盖未呈现帧；实际可见帧率还受屏幕刷新率限制。',style='Muted.TLabel',wraplength=300).pack(fill='x',pady=12)
  self.screen_note=ttk.Label(perf,text='正在读取屏幕刷新能力…',style='Muted.TLabel',wraplength=300);self.screen_note.pack(fill='x',pady=8)
  self.render_note=ttk.Label(perf,text='渲染器准备中',style='Muted.TLabel',wraplength=300);self.render_note.pack(fill='x',pady=8)
  self.payload_note=ttk.Label(perf,text='',style='Muted.TLabel',wraplength=300);self.payload_note.pack(fill='x',pady=10)
  ttk.Label(perf,text='实时传输带宽 · 最近 60 秒',style='Title.TLabel').pack(anchor='w',pady=(8,4))
  self.bandwidth_note=ttk.Label(perf,text='等待 PCIe 主机写入 / RDMA 计数',style='Muted.TLabel',wraplength=300)
  self.bandwidth_note.pack(fill='x',pady=(0,4))
  self.bandwidth_canvas=tk.Canvas(perf,height=158,bg='#0d1829',highlightthickness=0)
  self.bandwidth_canvas.pack(fill='x',pady=(0,8))
  self.bandwidth_canvas.bind('<Configure>',lambda e:self.draw_bandwidth())
  self.latency_note=ttk.Label(perf,text='',style='Muted.TLabel',wraplength=300);self.latency_note.pack(fill='x',pady=6)
  self.vars['display_fps'].trace_add('write',lambda *_:self.invalidate())
  for key in ('stream','size','fps'):self.vars[key].trace_add('write',lambda *_:self.stream_changed())
  self.vars['fps'].trace_add('write',lambda *_:self.limit_exposure())
  self.stream_changed()
  self.start_button=ttk.Button(self.sidebar_actions,text='▶ 开始连续采集',style='Start.TButton',command=lambda:self.pipeline('start'));self.start_button.grid(row=0,column=0,sticky='ew',padx=3,pady=4)
  self.stop_button=ttk.Button(self.sidebar_actions,text='■ 安全停止两端',command=lambda:self.pipeline('stop'));self.stop_button.grid(row=0,column=1,sticky='ew',padx=3,pady=4)
  self.single_button=ttk.Button(self.sidebar_actions,text='单帧保存 PNG',command=lambda:self.freeze(True));self.single_button.grid(row=1,column=1,sticky='ew',padx=3,pady=4)
  self.camera_read_button=ttk.Button(self.sidebar_actions,text='读取型号与可用模式',command=self.read_camera);self.camera_read_button.grid(row=1,column=0,sticky='ew',padx=3,pady=4)
  self.sidebar_actions.columnconfigure(0,weight=1);self.sidebar_actions.columnconfigure(1,weight=1)
  ttk.Label(left,text='运行中修改尺寸会自动安全重配。\n安全保持时只预选，下次启动生效。',style='Muted.TLabel',wraplength=220).pack(anchor='w',pady=8)
  ttk.Separator(left).pack(fill='x',pady=6)
  ttk.Label(left,text='网络与保存',style='Title.TLabel').pack(anchor='w',pady=5)
  self.network=ttk.Label(left,text='读取网络状态…',style='Muted.TLabel',wraplength=220,justify='left');self.network.pack(fill='x',pady=5)
  self.path_label=ttk.Label(left,text='保存到项目 captures/ 目录',style='Muted.TLabel',wraplength=220);self.path_label.pack(fill='x',pady=7)
  test_panel(self,self.test_page);processing_panel(self,self.processing_page);self.stream_changed()
  right=tk.Frame(body,bg=BG);right.pack(side='left',fill='both',expand=True)
  self.tools_collapsed=True;self.tools_tab_seen=None;self.tools_custom_heights={}
  self.statistics_panel_height=1
  self.video_workspace=ttk.Panedwindow(right,orient='vertical');self.video_workspace.pack(fill='both',expand=True)
  self.notebook=ttk.Notebook(self.video_workspace,height=1)
  upper=ttk.Panedwindow(self.video_workspace,orient='horizontal')
  self.video_workspace.add(upper,weight=1);self.video_workspace.add(self.notebook,weight=0)
  self.video_workspace.bind('<Configure>',lambda e:self.root.after_idle(self.layout_tools))
  self.video_workspace.bind('<ButtonRelease-1>',self.tools_dragged)
  self.image_panel=ttk.Frame(upper);upper.add(self.image_panel,weight=4)
  tools=ttk.Frame(self.image_panel,padding=(8,7));tools.pack(fill='x')
  self.view_badge=ttk.Label(tools,text='LIVE · 完整帧',foreground=ACCENT);self.view_badge.pack(side='left')
  self.yolo_rate_badge=ttk.Label(tools,text='YOLO 0.0 fps',foreground='#7bdcff');self.yolo_rate_badge.pack(side='left',padx=(14,8))
  for title,cmd in [('实时',self.resume),('冻结快照 / 分析',lambda:self.freeze(False)),('适应窗口',lambda:self.set_zoom('适应窗口')),('1:1',lambda:self.set_zoom('100%')),('画面全屏',self.fullscreen)]:
   ttk.Button(tools,text=title,command=cmd).pack(side='left',padx=3)
  self.tools_toggle_button=ttk.Button(tools,text='展开下方工具',command=self.toggle_tools);self.tools_toggle_button.pack(side='left',padx=3)
  self.zoom_var=tk.StringVar(value='适应窗口');z=ttk.Combobox(tools,textvariable=self.zoom_var,values=['适应窗口','50%','100%','200%','400%'],state='readonly',width=9);z.pack(side='right');z.bind('<<ComboboxSelected>>',lambda e:self.set_zoom(self.zoom_var.get()))
  viewport=ttk.Frame(self.image_panel);viewport.pack(fill='both',expand=True)
  self.canvas=tk.Canvas(viewport,bg='#070e19',highlightthickness=0)
  xs=ttk.Scrollbar(viewport,orient='horizontal',command=lambda *a:self.scroll('x',*a));ys=ttk.Scrollbar(viewport,orient='vertical',command=lambda *a:self.scroll('y',*a))
  self.canvas.configure(xscrollcommand=xs.set,yscrollcommand=ys.set);ys.pack(side='right',fill='y');xs.pack(side='bottom',fill='x');self.canvas.pack(fill='both',expand=True)
  self.canvas.bind('<Configure>',lambda e:self.invalidate());self.canvas.bind('<Button-1>',self.pixel_click)
  self.canvas.bind('<ButtonPress-2>',lambda e:self.canvas.scan_mark(e.x,e.y));self.canvas.bind('<B2-Motion>',self.pan)
  self.canvas.bind('<MouseWheel>',lambda e:self.scroll('y','scroll',-1 if e.delta>0 else 1,'units'))
  self.view_status=ttk.Label(self.image_panel,text='等待新的完整图像',style='Muted.TLabel',padding=6);self.view_status.pack(fill='x')
  analysisbox=ttk.Frame(self.notebook);self.notebook.add(analysisbox,text='像素 / RGB 直方图')
  ac=tk.Canvas(analysisbox,height=185,bg=PANEL,highlightthickness=0)
  asc=ttk.Scrollbar(analysisbox,orient='vertical',command=ac.yview);asc.pack(side='right',fill='y');ac.configure(yscrollcommand=asc.set);ac.pack(side='left',fill='both',expand=True)
  analysis=ttk.Frame(ac,padding=10);aw=ac.create_window((0,0),window=analysis,anchor='nw')
  analysis.bind('<Configure>',lambda e:ac.configure(scrollregion=ac.bbox('all')))
  ac.bind('<Configure>',lambda e:ac.itemconfigure(aw,width=e.width))
  summary=ttk.Frame(analysis);summary.grid(row=0,column=0,sticky='nsew',padx=(0,12))
  ttk.Label(summary,text='图像像素分析',style='Title.TLabel').pack(anchor='w')
  self.analysis_label=ttk.Label(summary,text='等待完整帧 · 实时分析 2 Hz',style='Muted.TLabel',wraplength=230);self.analysis_label.pack(fill='x',pady=5)
  self.pixel_label=ttk.Label(summary,text='X — / Y —\nR —   G —   B —',font=('Noto Sans CJK SC',font_px(15)));self.pixel_label.pack(fill='x',pady=3)
  self.analysis_source=tk.StringVar(value='原始图像')
  self.stat_label=ttk.Label(summary,text='均值 —  标准差 —\n范围 —  饱和比例 —',style='Muted.TLabel',wraplength=230);self.stat_label.pack(fill='x',pady=5)
  self.charts={}
  for col,items in enumerate(([('rgb_histogram','RGB 直方图 · R / G / B'),('histogram','灰度直方图')],[('horizontal','水平灰度曲线'),('vertical','垂直灰度曲线')]),1):
   group=ttk.Frame(analysis);group.grid(row=0,column=col,sticky='nsew',padx=6);analysis.columnconfigure(col,weight=1)
   for name,title in items:
    ttk.Label(group,text=title,style='Muted.TLabel').pack(anchor='w',pady=(0,3))
    cv=tk.Canvas(group,width=245,height=62,bg='#0a1526',highlightthickness=1,highlightbackground='#2a4059');cv.pack(fill='x',pady=(0,5));self.charts[name]=cv
  phase=ttk.Frame(self.notebook,padding=8);self.notebook.add(phase,text='分阶段丢帧 / 跳帧');self.notebook.select(phase)
  columns=('stage','where','count','kind','note')
  tablebox=ttk.Frame(phase);tablebox.pack(fill='x')
  self.drop_table=ttk.Treeview(tablebox,columns=columns,show='headings',height=5,selectmode='browse')
  for key,title,width in [('stage','统计项目',125),('where','发生位置',180),('count','数量',70),('kind','性质',95),('note','简要说明',400)]:
   self.drop_table.heading(key,text=title);self.drop_table.column(key,width=width,minwidth=50,stretch=key=='note')
  scroll=ttk.Scrollbar(tablebox,orient='vertical',command=self.drop_table.yview);scroll.pack(side='right',fill='y');self.drop_table.configure(yscrollcommand=scroll.set);self.drop_table.pack(side='left',fill='both',expand=True);self.drop_table.bind('<<TreeviewSelect>>',self.stage_selected)
  xscroll=ttk.Scrollbar(phase,orient='horizontal',command=self.drop_table.xview);xscroll.pack(fill='x');self.drop_table.configure(xscrollcommand=xscroll.set)
  detail_header=ttk.Frame(phase);detail_header.pack(fill='x',pady=(7,3))
  ttk.Label(detail_header,text='详细说明 · 选中上方项目查看，可滚轮阅读',style='Title.TLabel').pack(side='left')
  ttk.Button(detail_header,text='查看全部项目说明',command=self.show_all_stage_help).pack(side='right')
  detailbox=ttk.Frame(phase);detailbox.pack(fill='both',expand=True)
  self.drop_note=tk.Text(detailbox,height=7,wrap='word',font='TkDefaultFont',bg=PANEL,fg=TEXT,insertbackground=TEXT,selectbackground='#26516a',relief='flat',padx=10,pady=7,state='disabled')
  detail_scroll=ttk.Scrollbar(detailbox,orient='vertical',command=self.drop_note.yview);detail_scroll.pack(side='right',fill='y');self.drop_note.configure(yscrollcommand=detail_scroll.set);self.drop_note.pack(side='left',fill='both',expand=True)
  self.stage_help_all=True;self.stage_help_text=None
  def detail_wheel(e):
   num=getattr(e,'num',None);delta=getattr(e,'delta',0)
   step=-3 if num==4 else 3 if num==5 else -3 if delta>0 else 3 if delta<0 else 0
   if step:self.drop_note.yview_scroll(step,'units')
   return 'break'
  for sequence in ('<Button-4>','<Button-5>','<MouseWheel>'):self.drop_note.bind(sequence,detail_wheel)
  cache=ttk.Frame(self.notebook,padding=10);self.notebook.add(cache,text='图像缓存与保存')
  row=ttk.Frame(cache);row.pack(fill='x')
  ttk.Label(row,text='完整帧缓存（2 fps采样）').pack(side='left');self.cache_var=tk.StringVar(value='16');ttk.Spinbox(row,from_=1,to=32,width=4,textvariable=self.cache_var).pack(side='left',padx=5)
  ttk.Label(row,text='帧　保存采样').pack(side='left');self.record_var=tk.StringVar(value='5');ttk.Combobox(row,textvariable=self.record_var,values=['1','2','5','10'],state='readonly',width=4).pack(side='left')
  ttk.Label(row,text='fps').pack(side='left',padx=5)
  ttk.Button(row,text='应用设置',command=self.apply_cache).pack(side='left',padx=6)
  ttk.Button(row,text='保存当前帧 PNG',command=self.save_current).pack(side='left',padx=4)
  self.record_button=ttk.Button(row,text='● 连续保存 JPEG',command=self.toggle_record);self.record_button.pack(side='left',padx=4)
  browse=ttk.Frame(cache);browse.pack(fill='x',pady=(6,3))
  ttk.Button(browse,text='浏览实时缓存',command=self.browse_cache).pack(side='left',padx=(0,4))
  ttk.Button(browse,text='刷新已保存文件',command=self.load_saved).pack(side='left',padx=4)
  ttk.Button(browse,text='打开保存目录',command=self.open_save_directory).pack(side='left',padx=4)
  ttk.Button(browse,text='复制保存路径',command=self.copy_save_directory).pack(side='left',padx=4)
  self.gallery_mode=ttk.Label(browse,text='当前列表：实时缓存',style='Muted.TLabel');self.gallery_mode.pack(side='left',padx=8)
  location=ttk.Frame(cache);location.pack(fill='x',pady=(2,3))
  ttk.Label(location,text='保存根目录：').pack(side='left')
  self.save_directory_var=tk.StringVar(value=str(ROOT/'captures'))
  ttk.Entry(location,textvariable=self.save_directory_var,state='readonly').pack(side='left',fill='x',expand=True)
  self.last_saved_var=tk.StringVar(value='最近保存：尚未保存单帧')
  ttk.Label(cache,textvariable=self.last_saved_var,style='Muted.TLabel',wraplength=1100,justify='left').pack(fill='x',pady=2)
  gallerybox=ttk.Frame(cache);gallerybox.pack(fill='both',expand=True,pady=3)
  self.gallery=ttk.Treeview(gallerybox,columns=('seq','type','size'),show='headings',height=4)
  for key,title,w in [('seq','帧号 / 文件',380),('type','来源',120),('size','大小',130)]:
   self.gallery.heading(key,text=title);self.gallery.column(key,width=w)
  gy=ttk.Scrollbar(gallerybox,orient='vertical',command=self.gallery.yview);gy.pack(side='right',fill='y');self.gallery.configure(yscrollcommand=gy.set)
  self.gallery.pack(side='left',fill='both',expand=True);self.gallery.bind('<<TreeviewSelect>>',self.open_gallery)
  gx=ttk.Scrollbar(cache,orient='horizontal',command=self.gallery.xview);gx.pack(fill='x');self.gallery.configure(xscrollcommand=gx.set)
  self.record_status=ttk.Label(cache,text='缓存图像在独立快照窗口浏览，不暂停主视频。PNG 为完整分辨率；Bayer 同时保存 .bayer8。连续保存为采样 JPEG 图像序列。',style='Muted.TLabel');self.record_status.pack(fill='x',before=gallerybox,pady=(2,3))
  logs=ttk.Frame(self.notebook,padding=8);self.notebook.add(logs,text='运行日志')
  self.log=tk.Text(logs,height=10,bg='#0c1728',fg='#a9bfd8',font=('Noto Sans CJK SC',font_px(12)),relief='flat',wrap='word');self.log.pack(fill='both',expand=True)
  self.footer=tk.StringVar(value='GPU 直接呈现完整帧 · 图像分析低频采样独立执行')
  self.footer_label=tk.Label(r,textvariable=self.footer,bg=BG,fg=MUTED,anchor='w',justify='left',wraplength=1500,font=('Noto Sans CJK SC',font_px(12)))
  self.footer_label.pack(side='bottom',before=body,fill='x',padx=20,pady=9)
  r.bind('<Configure>',lambda e:self.footer_label.configure(wraplength=max(200,e.width-40)) if e.widget is r else None,add='+')
  # Keep the same canvas/native GPU drawable; only hide surrounding UI.
  self.video_fullscreen=False
  self.video_layout_path=(body,right,self.video_workspace,upper,self.image_panel,viewport,self.canvas)
  r.bind('<Escape>',lambda e:self.leave_fullscreen())
  r.bind('<F11>',lambda e:self.fullscreen())
  self.canvas.bind('<Double-Button-1>',lambda e:self.fullscreen())
  self.notebook.bind('<<NotebookTabChanged>>',self.tools_tab_changed)
  self.notebook.bind('<ButtonRelease-1>',lambda e:self.show_tools())
  from video_panels import install_settings_wheel
  install_settings_wheel(self);self.source_changed()
  for key in ('source','pattern','size','fps','exposure_us','buffers','stream'):
   self.vars[key].trace_add('write',lambda *_,key=key:self.parameter_changed(key))
  if JETSON:
   for var in self.ext_bits.values():var.trace_add('write',lambda *_:self.queue_reconfigure())
   for key in ('threshold','defect_threshold','sharp_coring','sharp_strength','gamma','contrast'):self.vars[key].trace_add('write',lambda *_:self.queue_reconfigure())
   for key in ('black_level','wb_r','wb_g','wb_b'):self.vars[key].trace_add('write',lambda *_:self.queue_live_isp())
   for var in self.ccm_vars+self.isp_bits:var.trace_add('write',lambda *_:self.queue_live_isp())
 def limit_exposure(self):
  try:
   maximum=automatic_exposure_limit_us(self.vars['fps'].get())
   if int(self.vars['exposure_us'].get())>maximum:self.vars['exposure_us'].set(str(maximum))
  except ValueError:pass
 def init_gpu(self):
  self.submit(screen_rates,lambda text:self.screen_note.configure(text='屏幕当前刷新：'+text+'\nGPU 帧率统计为新帧呈现提交；不等于屏幕扫描输出。'))
  if os.environ.get('VIDEO_GPU_DISABLE')=='1' or not hasattr(self.reader,'path'):return
  try:
   self.root.update_idletasks();self.gpu=Renderer(self.canvas.winfo_id(),self.display_path)
  except (OSError,RuntimeError) as e:
   self.gpu_failed=True;self.reader.fps=2;self.render_note.configure(text='逐帧显示不可用：'+str(e))
 def update_gpu(self,now):
  if now-self.last_gpu_update<.05:return
  self.last_gpu_update=now
  limit=0  # Ordered presentation follows acquisition; no preview rate cap.
  z=0 if self.zoom=='适应窗口' else float(self.zoom.strip('%'))/100
  if self.video_fullscreen and self.gpu_stats.get('width') and self.gpu_stats.get('height'):
   z=min(self.canvas.winfo_width()/self.gpu_stats['width'],self.canvas.winfo_height()/self.gpu_stats['height'])
  self.gpu.configure(self.canvas.winfo_width(),self.canvas.winfo_height(),self.frozen,limit,z,max(0,self.canvas.canvasx(0)),max(0,self.canvas.canvasy(0)))
  c=self.status.get('console',{});session=c.get('capture',{}).get('config',{}).get('session','')
  path='/dev/shm/lrdma_boxes_'+session if JETSON and session else (str(ROOT/'received_boxes.bin') if c.get('receiver',{}).get('boxes_channel') else '')
  self.gpu.boxes(path, bool(c.get('yolo',{}).get('running')))
  st=self.gpu.status();self.gpu_stats=st
  if st.get('boxes_valid'):self.yolo_note.configure(text='YOLO %.1f fps · %d 框 · 框龄 %d 帧 / %.1f ms（相对显示帧）'%(c.get('yolo',{}).get('detect_fps',0),st.get('boxes_count',0),st.get('boxes_frame_age',0),st.get('boxes_age_ms',0)))
  else:
   detector=c.get('yolo',{})
   if detector.get('phase')=='model_missing':
    text='YOLO 模型未部署 · 将 yolo11n.pt 放入软件 models/，下次启动自动生成引擎'
   elif detector.get('error') and not detector.get('ready'):
    text='YOLO 预热失败：'+str(detector['error'])[:100]
   elif not detector.get('ready') and detector.get('worker_running'):
    text='YOLO 正在后台加载 / 预热，视频持续显示'
   elif not detector.get('running') and detector.get('ready'):
    text='YOLO 模型已预热 · 勾选后立即检测最新帧'
   else:text='YOLO %.1f fps · 等待新结果 / 超过 250 ms 的框已隐藏'%detector.get('detect_fps',0)
   self.yolo_note.configure(text=text)
  if st['error']:
   self.gpu.close();self.gpu=None;self.reader.fps=2;self.gpu_failed=True;self.render_note.configure(text='逐帧显示故障：\n'+gpu_failure_detail(st['error']));return
  if st['ready']:self.render_note.configure(text='GPU：'+st['renderer']+'\n按序呈现，未呈现帧保留；不是屏幕扫描率。')
  if now-self.last_gpu_report>=.5:
   self.last_gpu_report=now
   try:
    report={**st,'pid':os.getpid(),'updated_ns':time.monotonic_ns(),'display_path':str(self.display_path)}
    tmp=ROOT/'runtime/display_gpu_status.tmp';tmp.write_text(json.dumps(report));tmp.replace(ROOT/'runtime/display_gpu_status.json')
   except OSError:pass
  self.display_count=st['count'];self.display_skipped=st['skipped']
  self.metrics['display'].configure(text='%.1f fps'%(st['fps'] if not self.frozen else 0))
  if st['width'] and not self.frozen and st['age']<2:
   w,h=st['width'],st['height'];cw,ch=self.canvas.winfo_width(),self.canvas.winfo_height()
   scale=z or min(cw/w,ch/h);rw,rh=w*scale,h*scale
   self.canvas.configure(scrollregion=(0,0,max(cw,rw),max(ch,rh)))
   self.drawn=dict(scale=scale,offset_x=max(0,(cw-rw)/2),offset_y=max(0,(ch-rh)/2),image_width=w,image_height=h,rendered_width=rw,rendered_height=rh)
   cfg=c.get('capture',{}).get('config',{})
   if not cfg or (cfg.get('width'),cfg.get('height'))==(w,h):
    self.metrics['resolution'].configure(text='%d × %d'%(w,h))
   self.view_status.configure(text='%s · %d × %d | 按序帧 #%d | 已呈现 %d / 源序号缺口 %d | 待呈现 %d 帧'%(format_name(st['format']),w,h,st['seq'],st['count'],st['skipped'],st.get('queue_depth',0)))
 def field(self,parent,key,label,value,choices=None):
  row=ttk.Frame(parent);row.pack(fill='x',pady=4)
  ttk.Label(row,text=label,style='Muted.TLabel',width=15,wraplength=130,justify='left').pack(side='left')
  var=tk.StringVar(value=value);self.vars[key]=var
  widget=ttk.Combobox(row,textvariable=var,values=choices,state='readonly',width=18) if choices else ttk.Entry(row,textvariable=var,width=18)
  widget.pack(side='right',fill='x',expand=True);setattr(self,key+'_widget',widget)

 def scroll(self,axis,*args):
  (self.canvas.xview if axis=='x' else self.canvas.yview)(*args);self.dirty=True
 def pan(self,e):
  self.canvas.scan_dragto(e.x,e.y,gain=1);self.dirty=True
 def invalidate(self):self.dirty=True
 def request(self,path,data=None):
  headers={}
  if data is not None:headers={'Content-Type':'application/json','X-Console-Token':self.token}
  req=urllib.request.Request('http://'+LOCAL+':8788'+path,data=None if data is None else json.dumps(data).encode(),headers=headers)
  try:
   with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req,timeout=8 if data is None else 45) as r:result=json.load(r)
  except urllib.error.HTTPError as e:
   try:raise RuntimeError(json.load(e).get('error',str(e))) from e
   except ValueError:raise RuntimeError(str(e)) from e
  if result.get('ok') is False:raise RuntimeError(result.get('error','请求失败'))
  return result
 def submit(self,fn,callback=None):
  with self.future_lock:
   if self.closed.is_set():return
   f=self.pool.submit(fn);self.pending_futures.add(f)
  def done(f):
   with self.future_lock:self.pending_futures.discard(f)
   if self.closed.is_set() or f.cancelled():return
   try:self.results.put((callback,f.result(),None))
   except Exception as e:self.results.put((None,None,str(e)))
  f.add_done_callback(done)
 def poll_status(self):
  while not self.closed.is_set():
   try:
    d=self.request('/status');self.token=d.get('csrf','');self.results.put((self.receive_status,d,None))
   except Exception as e:self.results.put((self.status_error,str(e),None))
   self.closed.wait(1)
 def status_error(self,error):
  self.report_window_error('本机界面服务连接失败：'+error)
  self.net_error=error;self.connection.configure(text='本机界面服务未连接 · 执行 ./video.sh station',fg='#e5b77a')
 def draw_bandwidth(self):
  canvas=self.bandwidth_canvas
  canvas.delete('all')
  width=max(100,canvas.winfo_width());height=max(100,canvas.winfo_height())
  left,right,top,bottom=46,width-10,10,height-26
  # Fill the unused part of the 60-second window with idle zero samples so
  # both coloured traces remain visible before and after capture.
  samples=[(0.0,0.0)]*max(0,60-len(self.bandwidth.samples))+list(self.bandwidth.samples)
  peak=max(100.0,max((max(pair) for pair in samples),default=0.0)*1.15)
  for fraction in (0,.5,1):
   y=bottom-(bottom-top)*fraction
   canvas.create_line(left,y,right,y,fill='#28405a')
   canvas.create_text(left-5,y,text=str(round(peak*fraction)),anchor='e',
                      fill=MUTED,font=('Noto Sans CJK SC',font_px(9)))
  canvas.create_text(left, height-7,text='MiB/s',anchor='w',fill=MUTED,
                     font=('Noto Sans CJK SC',font_px(9)))
  for index,color in ((0,'#59d5c6'),(1,'#f4b66a')):
   points=[]
   for i,pair in enumerate(samples):
    x=right-(len(samples)-1-i)*(right-left)/59
    y=bottom-min(pair[index]/peak,1)*(bottom-top)
    if pair[index]==0:y+=-2 if index==0 else 2  # Separate equal zero traces visually.
    points.extend((x,y))
   if len(points)>=4:canvas.create_line(*points,fill=color,width=2,smooth=False)
   elif len(points)==2:canvas.create_oval(points[0]-2,points[1]-2,points[0]+2,points[1]+2,fill=color,outline='')
  canvas.create_text(right-4,top+5,text=getattr(self,'bandwidth_source','PCIe直写'),anchor='ne',fill='#59d5c6',
                     font=('Noto Sans CJK SC',font_px(9)))
  canvas.create_text(right-4,top+22,text='RDMA',anchor='ne',fill='#f4b66a',
                     font=('Noto Sans CJK SC',font_px(9)))
 def receive_status(self,d):
  if self.window_error.startswith('本机界面服务连接失败：'):self.window_error=''
  self.status=d;self.camera_status(d.get('console',{}).get('camera_readback',{}));self.last_status=time.monotonic();self.net_error='';c=d.get('console',{})
  detector=c.get('yolo',{});self.yolo_rate_badge.configure(text='YOLO %.1f fps'%float(detector.get('detect_fps',0) if detector.get('running') else 0))
  if self.token and not self.preview_configured:
   self.preview_configured=True
   self.submit(lambda:self.request('/api/config',{'preview_fps':15}),lambda v:self.footer.set('后台网页采样15 fps；本地窗口直接读取原始帧'))
  test=c.get('pressure',{});testing=test.get('state') in ('PREPARING','RUNNING','STOPPING');self.update_pressure(test)
  if testing and test.get('config',{}).get('pressure_pattern'):
   badge='PRESSURE · FPGA DDR 读回彩条' if JETSON else ('PRESSURE · RDMA 实收彩条' if test.get('config',{}).get('pressure_rdma') else 'PRESSURE · 本地彩条参考图')
  else:badge='LIVE · 完整帧'
  self.view_badge.configure(text=badge)
  if JETSON:
   self.reader.path=(ROOT/'runtime/display_memory/latest_frames.bin') if testing and test.get('config',{}).get('pressure_pattern') else Path('/dev/shm/fpga_video_input')
  running=c.get('capture',{}).get('running',False);busy=c.get('busy',False) or testing;hold=any(c.get(k,{}).get('safety_hold',False) for k in ('capture','sender','receiver'))
  wait_text='RDMA 传输已中断\n发送端处于 DMA 安全保持' if hold else '等待新的完整图像\n两端就绪后点击“开始连续采集”'
  if wait_text!=getattr(self,'wait_message',None):
   self.wait_message=wait_text
   if not self.current or time.monotonic()-self.current[5]>2:self.dirty=True
  self.connection.configure(text='采集 / RDMA 运行中' if c.get('sender',{}).get('running') else '设备就绪 / 等待开始',fg=ACCENT if running else '#e5b77a')
  if hold:self.connection.configure(text='DMA 安全保持 · 查看下方具体原因',fg='#ff9c80')
  elif c.get('sender',{}).get('running') and not c.get('receiver',{}).get('running'):self.connection.configure(text='RDMA 接收已中断 · 查看运行日志',fg='#ff9c80')
  cfg=c.get('capture',{}).get('config',{})
  applying=c.get('reconfiguring',False)
  if applying and not self.reconfigure_hold:
   self.reconfigure_was_frozen=self.frozen;self.reconfigure_hold=True;self.dirty=True
   self.view_badge.configure(text='自动重配中 · 正在排空顺序显示队列')
  elif not applying and self.reconfigure_hold:
   self.reconfigure_hold=False
   if not self.reconfigure_was_frozen:self.resume()
  # Reflect toggles made by the peer console or API, even in the same session.
  if running and cfg and not self.yolo_pending:self.yolo_var.set(bool(cfg.get('yolo',False)))
  if running and cfg and cfg.get('session')!=self.config_session:
   self.isp_sync=True;self.config_session=cfg.get('session')
   # A new hardware session must not inherit frozen images, sampled frames or display counters.
   self.cache.clear();self.update_cache();self.current=None;self.resume()
   self.display_count=0;self.display_skipped=0;self.stream_epoch=None
   with self.reader.lock:self.reader.latest=None
   self.vars['source'].set('相机内置测试图' if cfg.get('source')=='camera_pattern' else 'Camera Link 相机')
   if cfg.get('test_pattern') in (11,12,13):self.vars['pattern'].set({11:'灰度条',12:'彩色条',13:'棋盘格'}[cfg['test_pattern']])
   for key in ('fps','buffers','exposure_us','black_level','wb_r','wb_g','wb_b'):
    if key in self.vars and key in cfg:self.vars[key].set(str(cfg[key]))
   if JETSON and cfg.get('ccm'):
    for v,value in zip(self.ccm_vars,cfg['ccm']):v.set(str(value))
   if cfg.get('format'):self.vars['stream'].set(next((k for k,v in STREAMS.items() if v==cfg['format']),next(iter(STREAMS))))
   self.sync_active_resolution(cfg)
   if JETSON and 'isp_enable' in cfg:
    self.isp_var.set(bool(cfg['isp_enable']))
    # Bypass must not erase which functions the master switch will restore.
    if cfg['isp_enable']:
     for i,var in enumerate(self.isp_bits):var.set(bool(cfg['isp_enable']&(1<<i)))
   self.isp_sync=False
  blocked=self.update_capture_controls(c,busy,hold)
  self.yolo_check.configure(state='disabled' if self.yolo_pending or busy or hold or (cfg.get('format') if running else STREAMS[self.vars['stream'].get()])!='rgb' else 'normal')
  self.controls_blocked=blocked
  self.camera_read_button.configure(state='disabled' if blocked or running or c.get('sender',{}).get('running') else 'normal')
  self.source_changed();self.stream_changed()
  online=c.get('online_isp',{});live_ok=JETSON and running and online.get('available') and online.get('allowed') and not busy and not hold and cfg.get('format')=='rgb'
  if JETSON and running and not live_ok:
   for widget in self.isp_widgets+[self.isp_check]+[getattr(self,k+'_widget') for k in ('black_level','wb_r','wb_g','wb_b')]:widget.configure(state='disabled')
  if live_ok:
   for widget in self.isp_widgets+[self.isp_check]+[getattr(self,k+'_widget') for k in ('black_level','wb_r','wb_g','wb_b')]:widget.configure(state='normal')
  if JETSON and hasattr(self,'online_note'):
   # Controller config is updated only after readback succeeds; mailbox may
   # retain an earlier session, while legacy diag.isp is stopped-only CSR.
   mask=cfg.get('isp_enable',online.get('isp_enable',0))
   state=('旁路：未启用处理' if not mask else '已启用：'+' / '.join(name for i,name in enumerate(('黑电平','白平衡','CCM','Gamma')) if mask&(1<<i)))
   self.online_note.configure(text=(('在线调参可用 · 硬件 '+state+' · mask=0x%x · generation=%s'%(mask,online.get('generation','—'))) if live_ok else '当前无法在线调参；停止后设置，下次采集生效'))
  if JETSON:
   actual=cfg.get('isp_enable') if running and online.get('available') else c.get('capture',{}).get('diag',{}).get('isp')
   raw=STREAMS[self.vars['stream'].get()]=='bayer'
   pp=c.get('preprocess',{})
   if pp.get('available'):
    self.isp_label.configure(text='Bayer 原始流旁路全部 ISP' if raw else '基础 ISP %s · 扩展 0x%x · 版本 %s'%('未读取' if actual is None else '0x%x'%actual,pp.get('enable',0),pp.get('generation','—')))
   else:self.isp_label.configure(text='Bayer 原始流旁路 ISP' if raw else '基础 FPGA ISP：'+('未读取' if actual is None else '0x%x'%actual))
  fill=c.get('capture',{}).get('diag',{}).get('fill')
  target=cfg.get('fps')
  if fill is not None and target:
   warning='\nDDR 持续积压：当前处理低于采集速度，优先降低目标帧率。' if fill>=8 else ''
   self.latency_note.configure(text='DDR 待释放 %s / 32 槽\n队列等效时长约 %.1f ms（估算，非端到端测量）%s'%(fill,fill*1000/target,warning))
  else:self.latency_note.configure(text='DDR 队列状态：等待采集')
  self.metrics['resolution'].configure(text=('%s × %s'%(cfg['width'],cfg['height'])) if cfg.get('width') and cfg.get('height') else self.vars['size'].get())
  for key,val in [('capture',c.get('capture_fps')),('receive',c.get('receive_fps'))]:self.metrics[key].configure(text='—' if val is None else '%.1f fps'%val)
  self.metrics['sent_skip'].configure(text=str(c.get('sender',{}).get('source_skipped','—'))+' 帧')
  receiver=c.get('receiver',{})
  checked=receiver.get('crc_checked',0) if receiver.get('crc_mode',cfg.get('crc_mode'))=='full' else 0
  self.metrics['crc'].configure(text=str(receiver.get('crc_errors',0))+' 帧' if checked else '未检')
  self.crc_checked_note.configure(text='接收端已逐帧校验：%s 帧'%format(checked,',') if checked else '等待逐帧 CRC 校验；旧会话须停止后重启')
  dma=c.get('capture',{});sender=c.get('sender',{})
  rdma=sender if JETSON else receiver
  pattern=testing and test.get('config',{}).get('pressure_pattern')
  if pattern:
   selected=test['config'];sample_session=cfg.get('session') if dma.get('running') and (not selected['pressure_rdma'] or rdma.get('running')) else None
   dma_total=dma.get('dma_bytes_total',0) if selected['pressure_dma'] else 0
   rdma_total=(rdma.get('received',0) if not JETSON else rdma.get('submitted',0))*(rdma.get('bytes') or frame_bytes(cfg)) if selected['pressure_rdma'] else 0
   self.bandwidth_source='DMA压测'
   changed=self.bandwidth.update_totals(time.monotonic(),sample_session,dma_total,rdma_total,
                                         dma.get('updated'),rdma.get('updated'))
  else:
   sample_session=cfg.get('session') if dma.get('running') and rdma.get('running') else None
   sample_bytes=frame_bytes(cfg) if cfg else 0
   self.bandwidth_source='PCIe直写'
   changed=self.bandwidth.update(time.monotonic(),sample_session,dma.get('frames'),
                                 rdma.get('frames') if JETSON else rdma.get('received'),
                                 sample_bytes,rdma.get('bytes') or sample_bytes,
                                 dma.get('updated'),rdma.get('updated'))
  if changed:self.draw_bandwidth()
  if self.bandwidth.samples:
   dma_rate,rdma_rate=self.bandwidth.samples[-1]
   self.bandwidth_note.configure(text='%s → Jetson  %.0f MiB/s    RDMA %s  %.0f MiB/s'%(
    self.bandwidth_source,dma_rate,'发送' if JETSON else '接收',rdma_rate))
  else:self.bandwidth_note.configure(text='%s → Jetson  0 MiB/s    RDMA %s  0 MiB/s'%(self.bandwidth_source,'发送' if JETSON else '接收'))
  self.network.configure(text='本端 RDMA  '+c.get('local_rdma_ip','—')+'\n对端 RDMA  '+c.get('peer_rdma_ip','—')+'\n管理地址  '+c.get('local_management_ip','—')+'\n网口  '+c.get('network',{}).get('interface','—')+'\n链路 '+c.get('network',{}).get('state','—')+'  MTU '+str(c.get('network',{}).get('mtu','—')))
  self.path_label.configure(text='保存位置：\n'+str(ROOT/'captures'))
  rows=stage_counts(c.get('capture',{}),c.get('sender',{}),c.get('receiver',{}),self.display_skipped,c.get('record_drops'))
  self.stage_rows={x['key']:x for x in rows}
  for row in rows:
   vals=(row['stage'],row['where'],('—' if row['value'] is None else str(row['value']))+' '+row['unit'],row['kind'],row['brief'])
   if self.drop_table.exists(row['key']):self.drop_table.item(row['key'],values=vals)
   else:self.drop_table.insert('','end',iid=row['key'],values=vals)
  self.stage_selected()
  self.record_button.configure(text='■ 停止连续保存' if c.get('recording') else '● 连续保存 JPEG')
  directory=c.get('record_directory') or str(ROOT/'captures')
  self.record_status.configure(text=('正在保存' if c.get('recording') else '连续保存已停止')+' · %s 帧 / %.1f MiB · 队列跳帧 %s · %s fps JPEG 序列\n目录：%s%s'%(c.get('record_count',0),c.get('record_bytes',0)/1048576,c.get('record_drops',0),self.record_var.get(),directory,('\n保存错误：'+str(c['record_error'])) if c.get('record_error') else ''),wraplength=1100,justify='left')
  self.show_fault(c)
  events=event_text(c.get('events',[]))
  report=fault_report(c)
  if report:events+='\n[当前故障详情]\n'+report['detail']
  if self.window_error:events+='\n[窗口操作失败] '+self.window_error
  if events!=getattr(self,'last_events',None):
   self.last_events=events;self.log.delete('1.0','end');self.log.insert('end',events);self.log.see('end')
  if not fault_report(c) and not self.window_error and not self.busy:
   self.footer_label.configure(fg=MUTED)
   phase=test.get('state','IDLE')
   if phase in ('PREPARING','RUNNING','STOPPING'):
    label={'PREPARING':'正在启动','RUNNING':'测试中','STOPPING':'正在排空'}[phase]
    self.footer.set('压力测试'+label+' · '+str(test.get('elapsed_s',0))+' / '+str(test.get('config',{}).get('seconds','—'))+' 秒 · '+str(test.get('message','')))
   else:
    # Finished pressure results remain in the pressure panel. The footer
    # always describes the current capture/pipeline operation, including
    # resolution changes and their failures.
    self.footer.set(c.get('message','就绪')+' · 阶段计数取最近日志，不直接相加')
 def show_fault(self,c):
  report=fault_report(c)
  if not report and not self.window_error:return
  text=report['message'] if report else '窗口操作失败：'+self.window_error
  if report and report['safety_hold']:text+='\nDMA 安全保持：先复位 FPGA，再恢复采集。详细原因与两端日志路径见“运行日志”。'
  if self.window_error and report:text+='\n窗口操作失败：'+self.window_error
  self.footer_label.configure(fg='#ff9c80')
  self.footer.set(text)
 def report_window_error(self,reason):
  self.busy=False;self.stop_pending=False;self.window_error=str(reason)
  self.show_fault(self.status.get('console',{}))
  self.log.insert('end','\n[窗口操作失败] '+self.window_error);self.log.see('end')
 def capture_config(self,fps=None):
  w,h=resolution_pair(self.vars['size'].get())
  cfg={k:self.vars[k].get() for k in ('fps','buffers','exposure_us','black_level','wb_r','wb_g','wb_b') if k in self.vars}
  cfg['queues']=min(8,int(cfg['buffers']))  # Legacy ABI; direct-host camera does not submit D2H queues.
  cfg.update(width=w,height=h,format=STREAMS[self.vars['stream'].get()],source='camera_pattern' if self.vars['source'].get()=='相机内置测试图' else 'camera')
  cfg['crc_mode']='full'
  cfg['yolo']=self.yolo_var.get()
  cfg['test_pattern']={'灰度条':11,'彩色条':12,'棋盘格':13}[self.vars['pattern'].get()] if cfg['source']=='camera_pattern' else 0
  if fps is not None:
   cfg['fps']=fps;cfg['exposure_us']=min(int(cfg['exposure_us']),automatic_exposure_limit_us(fps))
  if JETSON:
   cfg['isp_enable']=self.selected_isp_mask()
   cfg['ccm']=[float(v.get()) for v in self.ccm_vars]
   if self.status.get('console',{}).get('preprocess',{}).get('abi')==0x10001:
    # Preserve the existing HLS BLC/WB/CCM/fixed Gamma by default.
    mask=sum(bit for bit,var in self.ext_bits.items() if var.get()) if self.isp_var.get() else 0
    if cfg['format']=='bayer':mask=0
    if not (cfg['isp_enable']&8):mask&=~8
    cfg['preprocess']=dict(enable=mask)
    for key in ('threshold','defect_threshold','sharp_coring'):cfg['preprocess'][key]=int(self.vars[key].get())
    for key in ('sharp_strength','gamma','contrast'):cfg['preprocess'][key]=float(self.vars[key].get())
    # Tone has one owner; the original fixed curve remains the default.
    if mask&8:cfg['isp_enable']&=~8
  return validate_capture(cfg)
 def stream_changed(self):
  raw=STREAMS.get(self.vars['stream'].get())=='bayer'
  if raw and hasattr(self,'yolo_var'):self.yolo_var.set(False)
  self.stream_note.configure(text='原始 BayerRG8 · 1 字节/像素\n直接传输相机像素，GPU 去马赛克预览（不改变传输字节）。' if raw else 'FPGA 输出 RGB888 · 3 字节/像素\nISP 校正按发送端设置生效。')
  try:
   w,h=resolution_pair(self.vars['size'].get());n=w*h*(1 if raw else 3);fps=int(self.vars['fps'].get())
   self.payload_note.configure(text='图像有效载荷\n每帧 %s 字节 · %.2f MiB\n%d fps 时 %.2f Gbit/s\n仅图像数据，不含协议开销。'%(format(n,','),n/1048576,fps,n*fps*8/1e9))
  except ValueError:pass
  if JETSON:
   self.isp_label.configure(text='Bayer 原始流旁路 ISP；RGB 校正设置保留。' if raw else '黑电平 / 白平衡 / CCM / Gamma · 停止后设置')
   for widget in getattr(self,'isp_widgets',[]):widget.configure(state='disabled' if raw or getattr(self,'controls_blocked',False) else 'normal')
   self.isp_check.configure(state='disabled' if raw or getattr(self,'controls_blocked',False) else 'normal')
   pp=self.status.get('console',{}).get('preprocess',{});available=pp.get('abi')==0x10001 and pp.get('available',False)
   for widget in getattr(self,'ext_widgets',[]):widget.configure(state='normal' if available and not raw and not getattr(self,'controls_blocked',False) else 'disabled')
   if hasattr(self,'isp_capability'):self.isp_capability.configure(text=('UIP2 已识别：参数在开始采集时统一提交。' if available else '当前使用基础 ISP；扩展选项需要重新综合并加载配套 UIP2 位流。Gamma 为原硬件固定曲线。'))
   for key in ('black_level','wb_r','wb_g','wb_b'):
    if not hasattr(self,key+'_widget'):continue
    getattr(self,key+'_widget').configure(state='disabled' if raw or getattr(self,'controls_blocked',False) else 'normal')
 def source_changed(self):
  blocked=getattr(self,'controls_blocked',False)
  testing=self.vars['source'].get()=='相机内置测试图'
  self.pattern_widget.configure(state='readonly' if testing and not blocked else 'disabled')
  self.pattern_note.configure(text='相机生成条纹/棋盘格，替代镜头画面；经过同一 Camera Link、FPGA、DDR、RDMA 路径。' if testing else '真实相机源不使用诊断图案。切换数据源为“相机内置测试图”后，图案选择才生效。')
 def camera_status(self,info):
  values=info.get('values',{})
  if not values:return
  model=values.get('DeviceModelName','未读取');self.model_widget.configure(values=[model]);self.vars['model'].set(model)
  supported=available_resolutions(info)
  maximum=('%d×%d'%supported[0]) if supported else '无兼容预设'
  self.camera_info.configure(text='采集卡：Xilinx · 0005:01:00.0\n相机：'+model+'\n采集链路最大预设：'+maximum+'\n相机当前模式上限回读：'+values.get('WidthMax','—')+'×'+values.get('HeightMax','—')+'\n相机 ROI 回读：'+values.get('Width','—')+'×'+values.get('Height','—')+'\n采集使用 10-tap · 停止后修改')
  try:
   mw,mh=int(values['WidthMax']),int(values['HeightMax'])
   pairs=['%d×%d'%p for p in available_resolutions(info)]
   self.size_widget.configure(values=pairs)
   # A stale unsupported setting is not a selected/active acquisition mode.
   # Clear it; require an explicit new preset instead of silently cropping.
   if self.vars['size'].get() not in pairs:
    self.vars['size'].set('')
    self.footer.set('已移除当前采集链路不支持的分辨率，请重新选择完整组合（最大 '+maximum+'）')
  except (KeyError,ValueError):pass
 def read_camera(self):
  self.footer.set('正在读取相机型号与参数…');self.camera_read_button.configure(state='disabled')
  def done(d):self.camera_status(d.get('camera_readback',{}));self.footer.set(d.get('message','相机回读完成'))
  self.submit(lambda:self.request('/api/camera/read',{}),done)
 def pipeline(self,action):
  self.window_error=''
  if action=='stop':
   for name in ('reconfigure_after','live_isp_after'):
    pending=getattr(self,name,None)
    if pending:self.root.after_cancel(pending);setattr(self,name,None)
   self.reconfigure_queued=False;self.live_isp_queued=None
   c=self.status.get('console',{})
   held=[(name,c.get(name,{}).get('pid')) for name in ('capture','sender','receiver') if c.get(name,{}).get('safety_hold')]
   if held:
    detail='、'.join(name+' PID '+str(pid) for name,pid in held)
    self.report_window_error('无法完成安全停止：'+detail+' 处于 DMA 安全保持。必须先复位 FPGA；当前保留进程和 DMA 内存，分辨率预选不会应用。')
    return
   if getattr(self,'stop_pending',False):return
   self.stop_pending=True
  try:config=self.capture_config() if action in ('start','reconfigure') else {}
  except (ValueError,KeyError) as e:self.report_window_error('参数错误：'+str(e));return
  self.footer.set('正在自动重配并恢复' if action=='reconfigure' else '正在'+('启动两端采集' if action=='start' else '请求安全停止两端'));self.busy=True
  def finished(d):
   self.busy=False
   if action=='stop':self.stop_pending=False
   self.footer.set(d.get('message','操作已提交'))
  self.submit(lambda:self.request('/api/pipeline',{'action':action,'config':config}),finished)
 def show_all_stage_help(self):
  self.stage_help_all=True;self.stage_help_text=None;self.stage_selected()
  self.drop_note.yview_moveto(0)
 def stage_selected(self,e=None):
  if e is not None:self.stage_help_all=False
  rows=getattr(self,'stage_rows',{})
  sel=self.drop_table.selection()
  selected=rows.get(sel[0]) if sel else None
  if self.stage_help_all or selected is None:
   text=STAGE_COUNT_GUIDE+'\n\n'+'\n\n'.join(stage_detail(r) for r in rows.values())
  else:text=stage_detail(selected)+'\n\n统计总则：'+STAGE_COUNT_GUIDE
  key='all' if self.stage_help_all or selected is None else selected['key']
  if text==self.stage_help_text:return
  position=self.drop_note.index('@0,0') if key==getattr(self,'stage_help_key',None) else '1.0'
  self.stage_help_key=key;self.stage_help_text=text
  self.drop_note.configure(state='normal');self.drop_note.delete('1.0','end');self.drop_note.insert('1.0',text);self.drop_note.configure(state='disabled');self.drop_note.yview(position)
 def resume(self):
  self.clear_analysis('实时分析 · 等待新的完整帧');self.last_analysis_time=0;self.last_analysis_key=None;self.display_times.clear()
  self.frozen=False;self.reader.pause=False;self.display_pub=None;self.display_epoch=None;self.last_draw_key=None
  self.view_badge.configure(text='LIVE · 完整帧');self.dirty=True
 def freeze(self,save=False):
  running=self.status.get('console',{}).get('capture',{}).get('running')
  if running and not save:
   with self.reader.lock:item=self.reader.latest
   if not item or time.monotonic()-item[5]>2:
    self.footer.set('当前没有新鲜完整帧可冻结，请检查采集/接收状态');return
   self.open_snapshot(item,True,'冻结快照');return
  if self.gpu and not self.frozen:
   try:
    with self.raw_path.open('rb') as f:
     with mmap.mmap(f.fileno(),0,access=mmap.ACCESS_READ) as mem:got=snapshot(mem,0,raw=True)
    if got:
     pub,seq,(pixels,w,h,fmt),crc=got;im=to_image(pixels,w,h,fmt)
     self.current=(self.reader.epoch,pub,seq,im,crc,time.monotonic(),im,0,fmt,pixels if fmt==BAYER else None)
   except (OSError,ValueError):pass
  if not self.current or (not self.frozen and time.monotonic()-self.current[5]>2):
   self.footer.set('当前没有新鲜完整帧，请先开始采集');return
  if running and save:
   self.save_current();return
  self.current_analysis=None;self.frozen=True;self.reader.pause=True;self.view_badge.configure(text='冻结帧 #%s'%self.current[2]);self.dirty=True
  self.analyze(self.current[3].width//2,self.current[3].height//2)
  if save:self.save_current()
 def set_zoom(self,value):
  self.zoom=value;self.zoom_var.set(value);self.dirty=True;self.canvas.xview_moveto(0);self.canvas.yview_moveto(0)
 def fullscreen(self):
  if self.video_fullscreen:
   self.leave_fullscreen();return
  self.fullscreen_tools_visible=self.notebook in [self.root.nametowidget(str(p)) for p in self.video_workspace.panes()]
  if self.fullscreen_tools_visible:self.video_workspace.forget(self.notebook)
  self.fullscreen_zoom=self.zoom
  self.fullscreen_scroll=(self.canvas.xview()[0],self.canvas.yview()[0])
  self.fullscreen_window_state=bool(self.root.attributes('-fullscreen'))
  self.fullscreen_layout=[]
  for child in self.video_layout_path:
   parent=child.master
   packed=[(widget,dict(widget.pack_info())) for widget in parent.pack_slaves()]
   if not packed:continue  # The image pane is managed by Panedwindow.
   self.fullscreen_layout.append(packed)
   for widget,options in packed:
    if widget!=child:widget.pack_forget()
   child.pack_configure(padx=0,pady=0,ipadx=0,ipady=0)
  self.video_fullscreen=True
  self.set_zoom('适应窗口')
  self.root.attributes('-fullscreen',True)
  self.canvas.focus_set();self.dirty=True
 def leave_fullscreen(self):
  if not self.video_fullscreen:return
  self.video_fullscreen=False
  self.root.attributes('-fullscreen',self.fullscreen_window_state)
  if getattr(self,'fullscreen_tools_visible',False):self.video_workspace.add(self.notebook,weight=0)
  self.root.after_idle(self.layout_tools)
  for packed in self.fullscreen_layout:
   for widget,options in packed:widget.pack_forget()
   for widget,options in packed:widget.pack(**options)
  self.set_zoom(self.fullscreen_zoom)
  self.root.update_idletasks()
  self.canvas.xview_moveto(self.fullscreen_scroll[0])
  self.canvas.yview_moveto(self.fullscreen_scroll[1])
  self.dirty=True
 def tick(self):
  if self.closed.is_set():return
  for _ in range(20):
   try:callback,value,error=self.results.get_nowait()
   except queue.Empty:break
   if error:self.report_window_error(error)
   elif callback:callback(value)
  now=time.monotonic()
  if not self.frozen:
   with self.reader.lock:item=self.reader.latest
   if item and now-item[5]<2 and (item[0],item[1])!=self.last_draw_key:
    if not self.gpu and self.stream_epoch!=item[0]:
     self.display_count=0;self.display_skipped=0;self.display_times.clear();self.stream_epoch=item[0]
    self.current=item;self.current_raw=True;self.last_draw_key=(item[0],item[1]);self.dirty=True
    if not self.gpu and self.display_epoch==item[0] and self.display_pub is not None and item[1]>self.display_pub:self.display_skipped+=item[1]-self.display_pub-1
    self.display_epoch=item[0];self.display_pub=item[1];self.display_count+=int(not self.gpu);self.display_times.append(now)
    if now-self.last_cache>=.5:
     self.cache.append(item);self.last_cache=now
     while len(self.cache)>self.history_limit or sum(x[3].width*x[3].height*((6 if len(x)>6 and x[6] is not x[3] else 3)+(1 if len(x)>8 and x[8]==BAYER else 0)) for x in self.cache)>192*1024**2:self.cache.popleft()
     self.update_cache()
   elif (not item or now-item[5]>=2) and self.last_draw_key is not None:
    self.canvas.delete('all');self.canvas.create_text(max(150,self.canvas.winfo_width()/2),max(80,self.canvas.winfo_height()/2),text='等待新的完整图像\n采集已停止或暂无新帧',fill=MUTED,font=('Noto Sans CJK SC',font_px(20)),justify='center')
    self.view_status.configure(text='当前画面已过期；未将旧帧当作实时画面');self.last_draw_key=None;self.dirty=False
  if not self.gpu and not self.frozen and self.current and now-self.current[5]>.25:
   # Expire only overlay items even if the video producer stops publishing.
   self.canvas.delete('yolo_overlay')
  if self.analysis_pending and not self.analysis_busy:
   point=self.analysis_pending;self.analysis_pending=None;self.analyze(*point)
  if not self.frozen:
   if self.current and now-self.current[5]<2:
    if now-self.last_analysis_time>=.5 and self.current[:2]!=self.last_analysis_key and not self.analysis_busy:
     point=self.selected_point or (self.current[3].width//2,self.current[3].height//2);self.analyze(*point,live=True)
   elif self.current_analysis is not None or self.analysis_busy:
    self.clear_analysis('暂无新帧 / 接收中断，实时直方图已清空')
  if self.dirty and self.current and (self.frozen or now-self.current[5]<2):
   self.draw();self.dirty=False
  elif self.dirty:
   self.canvas.delete('all');self.canvas.create_text(max(150,self.canvas.winfo_width()/2),max(80,self.canvas.winfo_height()/2),text=getattr(self,'wait_message','等待新的完整帧'),fill=MUTED,font=('Noto Sans CJK SC',font_px(20)),justify='center');self.dirty=False
  fps=(len(self.display_times)-1)/(self.display_times[-1]-self.display_times[0]) if len(self.display_times)>1 and now-self.display_times[-1]<2 and not self.frozen else 0
  if self.gpu:self.update_gpu(now)
  else:self.metrics['display'].configure(text='不可用' if self.gpu_failed else '准备中')
  self.root.after(16,self.tick)
 def draw(self):
  if self.gpu_failed and not self.frozen:
   self.canvas.delete('all');self.canvas.create_text(300,150,text='逐帧显示不可用，请检查 GPU / 运行日志',fill=MUTED);return
  if self.gpu and not self.frozen:return
  im=self.display_image();cw=max(1,self.canvas.winfo_width());ch=max(1,self.canvas.winfo_height())
  scale=min(cw/im.width,ch/im.height) if self.video_fullscreen else (min(cw/im.width,ch/im.height) if self.zoom=='适应窗口' else float(self.zoom.strip('%'))/100)
  w=max(1,round(im.width*scale));h=max(1,round(im.height*scale))
  # At high zoom render only the visible crop: no 400%-sized full-frame allocation.
  maxw=max(cw,w);maxh=max(ch,h);self.canvas.configure(scrollregion=(0,0,maxw,maxh))
  if scale>1 and (w>cw or h>ch):
   left=max(0,int(self.canvas.canvasx(0)/scale));top=max(0,int(self.canvas.canvasy(0)/scale))
   right=min(im.width,left+int(cw/scale)+2);bottom=min(im.height,top+int(ch/scale)+2)
   shown=im.crop((left,top,right,bottom)).resize((max(1,round((right-left)*scale)),max(1,round((bottom-top)*scale))),RESAMPLE.NEAREST)
   x=left*scale;y=top*scale
  else:
   shown=im if (w,h)==im.size else im.resize((w,h),RESAMPLE.BILINEAR)
   x=max(0,(cw-w)/2);y=max(0,(ch-h)/2)
  self.photo=ImageTk.PhotoImage(shown);self.canvas.delete('all');self.canvas.create_image(x,y,image=self.photo,anchor='nw')
  self.draw_boxes(scale,max(0,(cw-w)/2),max(0,(ch-h)/2))
  if self.reconfigure_hold:self.canvas.create_text(20,20,anchor='nw',text='正在自动重配 · 暂停画面，非实时',fill='#ffda60',font=('Noto Sans CJK SC',font_px(20),'bold'))
  self.drawn={'scale':scale,'offset_x':max(0,(cw-w)/2),'offset_y':max(0,(ch-h)/2),'image_width':im.width,'image_height':im.height,'rendered_width':w,'rendered_height':h}
  self.view_status.configure(text='%s · %d × %d  |  显示 %.1f%%  |  帧号 %s  |  本窗口跳过 %d 帧'%(format_name(self.current[8]) if len(self.current)>8 else 'RGB888',im.width,im.height,scale*100,self.current[2],self.display_skipped))
 def draw_boxes(self,scale,ox,oy):
  if len(self.current)<12 or not self.status.get('console',{}).get('yolo',{}).get('running'):return
  c=self.status.get('console',{});packet=self.current[11]
  if JETSON:
   session=c.get('capture',{}).get('config',{}).get('session','')
   packet=self.box_reader.packet('/dev/shm/lrdma_boxes_'+session)
  elif c.get('receiver',{}).get('boxes_channel'):
   packet=self.box_reader.packet(str(ROOT/'received_boxes.bin'))
  from video_boxes import decode
  from video_labels import box_label
  result=decode(packet,self.current[2],self.current[10],self.current[3].width,self.current[3].height,0 if self.frozen else max(0,time.monotonic()-self.current[5])*1000)
  if not result:return
  for b in result['boxes']:
   x1,y1,x2,y2=[v*scale for v in b[:4]]
   color='#4aff30' if result['frame_age']==0 else '#ffd020'
   self.canvas.create_rectangle(ox+x1,oy+y1,ox+x2,oy+y2,outline=color,width=2,tags='yolo_overlay')
   cw,ch=self.canvas.winfo_width(),self.canvas.winfo_height()
   if ox+x2<=0 or oy+y2<=0 or ox+x1>=cw or oy+y1>=ch:continue
   label=box_label(b);font=tkfont.nametofont('TkDefaultFont');pad=max(3,round(3*UI_SCALE))
   if font.measure(label)>max(x2-x1,140*UI_SCALE):label=label.replace(' ID:','\nID:',1)
   lines=label.splitlines();lw=max(font.measure(line) for line in lines)+2*pad;lh=font.metrics('linespace')*len(lines)+2*pad
   lx=max(0,min(ox+x1,cw-lw));ly=oy+y1-lh if oy+y1>=lh else oy+y1+2;ly=max(0,min(ly,ch-lh))
   self.canvas.create_rectangle(lx,ly,lx+lw,ly+lh,fill='#080f18',outline='',tags='yolo_overlay')
   self.canvas.create_text(lx+pad,ly+pad,anchor='nw',text=label,font=font,fill=color,justify='left',tags='yolo_overlay')
  self.yolo_note.configure(text='YOLO %.1f fps · %d 框 · 框龄 %d 帧 / %.1f ms（相对显示帧）'%(c.get('yolo',{}).get('detect_fps',0),len(result['boxes']),result['frame_age'],result['age_ms']))
 def pixel_click(self,e):
  if not self.current or (not self.frozen and time.monotonic()-self.current[5]>2):return
  if not self.drawn:return
  d=self.drawn;x=int((self.canvas.canvasx(e.x)-d['offset_x'])/d['scale']);y=int((self.canvas.canvasy(e.y)-d['offset_y'])/d['scale'])
  if 0<=x<d['image_width'] and 0<=y<d['image_height']:
   self.selected_point=(x,y);self.analyze(x,y,live=not self.frozen)
 def sync_active_resolution(self,cfg):
  if cfg.get('width') and cfg.get('height'):
   active_size='%s×%s'%(cfg['width'],cfg['height'])
   if not getattr(self,'resolution_dirty',False) or self.vars['size'].get()==active_size:
    self.vars['size'].set(active_size);self.resolution_dirty=False
 def update_capture_controls(self,c,busy,hold):
  self.start_button.configure(state='disabled' if hold or busy or not c.get('controller') or not c.get('peer_controller') or c.get('sender',{}).get('running') else 'normal')
  stopping=c.get('operation_action')=='stop' or c.get('stop_requested',False)
  self.stop_button.configure(state='disabled' if getattr(self,'stop_pending',False) or stopping else 'normal',
                             text='■ 等待安全停止' if stopping else '■ 安全停止两端')
  external=bool(c.get('capture',{}).get('external_pid'));blocked=busy or hold or external
  for key in ('source','pattern','fps','buffers','crc_mode','stream','size','exposure_us','black_level','wb_r','wb_g','wb_b'):
   if key in self.vars:
    widget=getattr(self,key+'_widget')
    widget.configure(state='readonly' if key=='size' else ('disabled' if blocked else ('readonly' if isinstance(widget,ttk.Combobox) else 'normal')))
  if hold:self.auto_parameter_note.configure(text='DMA 安全保持：可以预选下次分辨率，当前不应用。先复位 FPGA 并安全结束旧会话，再开始采集。')
  elif busy:self.auto_parameter_note.configure(text='当前操作正在执行；分辨率选择会保留。安全停止优先于参数重配。')
  elif external:self.auto_parameter_note.configure(text='外部采集进程正在使用硬件；分辨率仅为下次预选，不能自动重配。')
  elif getattr(self,'reconfigure_queued',False) and c.get('capture',{}).get('running') and not getattr(self,'stop_pending',False):
   self.reconfigure_queued=False;self.queue_reconfigure()
  return blocked
 def parameter_changed(self,key):
  if self.isp_sync:return
  if key=='size':self.resolution_dirty=True
  self.queue_reconfigure()
 def queue_reconfigure(self):
  if self.isp_sync:return
  c=self.status.get('console',{})
  if self.reconfigure_after:self.root.after_cancel(self.reconfigure_after);self.reconfigure_after=None
  hold=any(c.get(k,{}).get('safety_hold') for k in ('capture','sender','receiver'))
  if hold or c.get('capture',{}).get('external_pid'):
   self.reconfigure_queued=False
   self.auto_parameter_note.configure(text='分辨率预选已保留；当前会话不能重配，安全停止后下次开始采集生效。')
   return
  if c.get('busy') or self.busy or getattr(self,'stop_pending',False):
   self.reconfigure_queued=not (getattr(self,'stop_pending',False) or c.get('stop_requested') or c.get('operation_action')=='stop')
   return
  if not c.get('capture',{}).get('running'):return
  self.reconfigure_queued=False
  self.reconfigure_after=self.root.after(700,self.send_reconfigure)
  self.auto_parameter_note.configure(text='参数待自动重配：输入完成后短暂停采并恢复；视频帧率可能短暂下降。')
 def send_reconfigure(self):
  self.reconfigure_after=None
  c=self.status.get('console',{})
  if c.get('busy') or getattr(self,'stop_pending',False):
   self.reconfigure_queued=not (getattr(self,'stop_pending',False) or c.get('stop_requested') or c.get('operation_action')=='stop')
   return
  if not c.get('capture',{}).get('running') or c.get('capture',{}).get('external_pid') or any(c.get(k,{}).get('safety_hold') for k in ('capture','sender','receiver')):return
  try:self.capture_config()
  except (ValueError,KeyError):self.auto_parameter_note.configure(text='请输入完整有效的参数；当前视频保持原设置');return
  self.pipeline('reconfigure')
 def toggle_yolo(self):
  c=self.status.get('console',{})
  if self.yolo_pending or not c.get('capture',{}).get('running'):return
  enabled=self.yolo_var.get();self.yolo_pending=True
  self.yolo_check.configure(state='disabled')
  def run():
   try:return self.request('/api/yolo',{'enabled':enabled})
   except Exception as e:return {'yolo_error':str(e)}
  def done(result):
   self.yolo_pending=False
   if result.get('yolo_error'):self.yolo_var.set(not enabled)
   self.yolo_note.configure(text=result.get('yolo_error',result.get('message','')))
  self.submit(run,done)
 def queue_live_isp(self,delay=250):
  if self.isp_sync or not JETSON:return
  c=self.status.get('console',{})
  if not c.get('capture',{}).get('running') or not c.get('online_isp',{}).get('available') or c.get('busy'):return
  if c.get('preprocess',{}).get('enable',0)&0x4b:self.queue_reconfigure();return
  if self.live_isp_after:self.root.after_cancel(self.live_isp_after)
  self.live_isp_after=self.root.after(delay,self.send_live_isp)
 def send_live_isp(self):
  self.live_isp_after=None
  try:
   values=dict(isp_enable=self.selected_isp_mask(),black_level=int(self.vars['black_level'].get()),wb_r=float(self.vars['wb_r'].get()),wb_g=float(self.vars['wb_g'].get()),wb_b=float(self.vars['wb_b'].get()),ccm=[float(v.get()) for v in self.ccm_vars])
   from video_live_isp import encode
   encode(values)
  except (ValueError,TypeError):self.online_note.configure(text='请输入完整、有效的 ISP 参数');return
  self.live_isp_queued=values
  if not self.live_isp_busy:self.flush_live_isp()
 def flush_live_isp(self):
  values=self.live_isp_queued;self.live_isp_queued=None;self.live_isp_busy=True
  selection=sum(1<<i for i,v in enumerate(self.isp_bits) if v.get()) or read_isp_selection()
  self.online_note.configure(text='正在等待硬件帧边界生效…')
  def run():
   try:return self.request('/api/isp/live',{'values':values})
   except Exception as e:return {'live_error':str(e)}
  def done(result):
   self.live_isp_busy=False
   self.online_note.configure(text=('更新失败：'+result['live_error']) if result.get('live_error') else '硬件已生效 · generation='+str(result.get('online_isp',{}).get('generation','—')))
   if not result.get('live_error'):
    try:save_isp_mask(values['isp_enable'],selection=selection)
    except (OSError,ValueError) as e:self.footer.set('硬件已生效，但保存 ISP 选择失败：'+str(e))
   if self.live_isp_queued is not None:self.flush_live_isp()
  self.submit(run,done)
 def selected_isp_mask(self):
  return sum(1<<i for i,v in enumerate(getattr(self,'isp_bits',[])) if v.get()) if self.isp_var.get() else 0
 def set_isp(self,child=False):
  if not JETSON or self.isp_sync:return
  selected=sum(1<<i for i,v in enumerate(self.isp_bits) if v.get())
  if child:
   # Selecting a function enables it; removing the last function means bypass.
   self.isp_var.set(bool(selected))
  elif self.isp_var.get() and not selected:
   # Legacy sessions could clear every child while disabling the master.
   self.isp_sync=True
   try:
    mask=read_isp_selection()
    for i,var in enumerate(self.isp_bits):var.set(bool(mask&(1<<i)))
   finally:self.isp_sync=False
  c=self.status.get('console',{});capture=c.get('capture',{})
  if capture.get('running') and c.get('online_isp',{}).get('available'):
   self.queue_live_isp(0);return
  if not c.get('controller') or capture.get('running') or capture.get('external_pid') or c.get('busy') or c.get('sender',{}).get('running'):
   self.isp_var.set(bool(read_isp_mask()));self.footer.set('先安全停止采集与发送，再改变 ISP 设置');return
  try:
   save_isp_mask(self.selected_isp_mask(),selection=selected or None)
   self.footer.set('ISP 设置已保存，下次开始采集生效；输出分辨率按相机参数设置')
  except (OSError,ValueError) as e:self.footer.set('ISP 保存失败：'+str(e))
 def clear_analysis(self,message):
  self.analysis_version+=1;self.analysis_pending=None;self.current_analysis=None
  self.analysis_label.configure(text=message)
  self.pixel_label.configure(text='X — / Y —\nR —   G —   B —')
  self.stat_label.configure(text='均值 —  标准差 —\n范围 —  饱和比例 —')
  for cv in self.charts.values():cv.delete('all')
 def analyze(self,x,y,live=False):
  if not self.current:return
  if self.analysis_busy:
   if not live:self.analysis_version+=1;self.analysis_pending=(x,y,False)
   return
  im=self.current[3];source=self.analysis_source.get();source+=' · Bayer软件去马赛克' if len(self.current)>8 and self.current[8]==BAYER else '';seq=self.current[2];self.analysis_version+=1;version=self.analysis_version
  self.analysis_busy=True;self.last_analysis_time=time.monotonic();self.last_analysis_key=self.current[:2]
  raw_sample=(self.current[9][max(0,min(im.height-1,y))*im.width+max(0,min(im.width-1,x))] if len(self.current)>9 and self.current[8]==BAYER else None)
  def calculate():
   try:return dict(analyze_frame(im,seq,x,y),version=version,live=live,source=source,raw_sample=raw_sample)
   except Exception as e:return dict(version=version,error=str(e))
  self.submit(calculate,self.show_analysis)
 def show_analysis(self,d):
  self.analysis_busy=False
  if d['version']!=self.analysis_version:return
  if d.get('error'):self.analysis_label.configure(text='分析失败：'+d['error']);return
  self.analysis_updates+=1;self.current_analysis=d
  self.analysis_label.configure(text=('实时分析 · 2 Hz' if d['live'] else '冻结分析')+' · 帧 #%s\n'%d['seq']+d.get('source','原始图像')+' · 点击图像选择行/列')
  self.pixel_label.configure(text=('X %s / Y %s\nR %s   G %s   B %s'%((d['x'],d['y'])+tuple(d['rgb'])))+('' if d.get('raw_sample') is None else '\n原始 Bayer 采样值：%d'%d['raw_sample']))
  self.stat_label.configure(text='均值 %.2f  标准差 %.2f\n最小 %d / 最大 %d  饱和 %.3f%%'%(d['mean'],d['stddev'],*d['extrema'],d['saturation']))
  self.rgb_chart(d['rgb_histogram'])
  for name,color in [('histogram',ACCENT),('horizontal','#7caff5'),('vertical','#b29aea')]:self.chart(self.charts[name],d[name],color,None if name=='histogram' else 255)
 def rgb_chart(self,channels):
  cv=self.charts['rgb_histogram'];cv.delete('all');w=max(2,cv.winfo_width());h=max(8,cv.winfo_height())
  peak=max(1,max(max(channel) for channel in channels))
  for values,color in zip(channels,('#ff6b78','#62d994','#65a9ff')):
   points=[]
   for px in range(w):
    begin=px*256//w;end=max(begin+1,(px+1)*256//w);v=max(values[begin:end]);points.extend((px,h-4-(h-8)*v/peak))
   cv.create_line(*points,fill=color,width=1)
 def display_image(self):
  return self.current[3] if self.current else None
 def refresh_analysis(self):
  self.clear_analysis('分析设置已改变，等待当前帧计算');self.last_analysis_key=None;self.last_analysis_time=0
  if self.current and self.frozen:self.analyze(*(self.selected_point or (self.current[3].width//2,self.current[3].height//2)))
 def pressure_action(self,action):
  if action=='start' and self.window_error.startswith('压力测试失败：'):
   self.window_error=''
  if action=='stop':
   self.submit(lambda:self.request('/api/test',{'action':'stop','config':{}}),lambda d:self.footer.set(d.get('message','正在停止')));return
  try:
   width,height=resolution_pair(self.test_vars['size'].get())
   cfg={'width':width,'height':height,'fps':30,'seconds':int(self.test_vars['seconds'].get()),
        'pressure_pattern':True,'pressure_dma':self.test_dma.get(),'pressure_rdma':self.test_rdma.get(),
        'transport':'ddr','format':'rgb','crc_mode':'transport','yolo':False}
   if not (cfg['pressure_dma'] or cfg['pressure_rdma']):raise ValueError('请至少勾选 DMA 或 RDMA')
  except ValueError as exc:self.footer.set('压力测试参数错误：'+str(exc));return
  self.submit(lambda:self.request('/api/test',{'action':action,'config':cfg}),lambda d:self.footer.set(d.get('message','已提交测试操作')))
 def update_pressure(self,t):
  state=t.get('state','IDLE');active=state in ('PREPARING','RUNNING','STOPPING')
  if state!=getattr(self,'last_pressure_state',None):
   if state=='FAIL':
    self.window_error='压力测试失败：'+str(t.get('message','未知错误'))
    self.log.insert('end','\n[压力测试失败] '+str(t.get('message','未知错误')));self.log.see('end')
   elif self.window_error.startswith('压力测试失败：') and state in ('PREPARING','RUNNING','PASS','CANCELLED'):
    self.window_error=''
   self.last_pressure_state=state
  c=self.status.get('console',{});blocked=c.get('busy') or not c.get('controller') or not c.get('peer_controller') or any(c.get(k,{}).get('running') or c.get(k,{}).get('external_pid') for k in ('capture','sender','receiver'))
  self.test_start.configure(state='disabled' if active or blocked else 'normal');self.test_stop.configure(state='normal' if active else 'disabled')
  names={'IDLE':'待测试','PREPARING':'正在启动','RUNNING':'测试中','STOPPING':'正在安全排空','PASS':'通过','FAIL':'失败','CANCELLED':'已取消','INTERRUPTED':'已中断'}
  self.test_progress['value']=t.get('progress',0);self.test_status.configure(text=names.get(state,state)+' · '+t.get('message','尚未开始测试'))
  n=t.get('metrics',{})
  cfg=t.get('config',{})
  if cfg.get('pressure_pattern') and state in ('PREPARING','RUNNING','STOPPING','PASS'):
   self.test_dma.set(bool(cfg.get('pressure_dma')));self.test_rdma.set(bool(cfg.get('pressure_rdma')))
   self.test_vars['seconds'].set(str(cfg.get('seconds',60)));self.test_vars['size'].set('%dx%d'%(cfg.get('width',2400),cfg.get('height',2048)))
  if t.get('config',{}).get('pressure_pattern'):
   self.test_metrics.configure(text='已用 %s s / 剩余 %s s\nDMA 当前 %.1f · 平均 %.1f · 峰值 %.1f MiB/s\nRDMA 当前 %.1f · 平均 %.1f · 峰值 %.1f MiB/s\nDMA 完成 %s MiB / RDMA 接收 %s MiB\n彩条 CRC 错误 %s / 故障 %s'%(
    t.get('elapsed_s',0),t.get('remaining_s','—'),t.get('dma_MiB_s',0),t.get('avg_dma_MiB_s',0),t.get('peak_dma_MiB_s',0),
    t.get('rdma_MiB_s',0),t.get('avg_rdma_MiB_s',0),t.get('peak_rdma_MiB_s',0),
    int(n.get('dma_bytes_total',0)/1048576),int(n.get('rdma_bytes_total',0)/1048576),
    n.get('crc_errors',0),n.get('faults',0)))
   if t.get('config',{}).get('pressure_rdma'):
    physical=t.get('phy_rx_Gbit_s')
    avg_physical=t.get('phy_rx_avg_Gbit_s')
    line=('网口物理接收 %.1f Gb/s'%physical) if physical is not None else '网口物理接收：尚未取得计数'
    if avg_physical is not None:line+=' · 平均 %.1f Gb/s'%avg_physical
    drops=t.get('rx_discards_phy_delta')
    if drops is not None:line+=' · 物理丢包 %d'%drops
    self.test_metrics.configure(text=self.test_metrics.cget('text')+'\n'+line)
  else:
   fps=t.get('measured_fps',0)
   self.test_metrics.configure(text='已用 %s s / 剩余 %s s\n接收 %s 帧 / 提交 %s 帧\n%.2f fps · %.1f MiB/s\nCRC错帧 %s / 失败事件 %s\n发送跳帧 %s / DMA旧帧 %s'%(t.get('elapsed_s',0),t.get('remaining_s','—'),n.get('received','—'),n.get('submitted','—'),fps,fps*frame_bytes(t.get('config',{}))/1048576,n.get('crc_errors','—'),n.get('faults','—'),n.get('source_skipped','—'),n.get('skip_host','—')))
  if t.get('directory'):self.test_report.configure(text='Mellanox 完整报告：\n'+t['directory'])
 def save_test_summary(self):
  d=self.status.get('console',{}).get('pressure',{})
  if not d or d.get('state')=='IDLE':self.footer.set('尚无测试报告');return
  folder=ROOT/'results';folder.mkdir(exist_ok=True);path=folder/('summary_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
  path.write_text(json.dumps(d,ensure_ascii=False,indent=2));self.footer.set('报告摘要已保存：'+str(path))
 def chart(self,cv,values,color,maximum):
  w=max(2,cv.winfo_width());h=max(8,cv.winfo_height());cv.delete('all')
  for i in range(1,4):cv.create_line(0,h*i/4,w,h*i/4,fill='#24364c')
  maximum=maximum or max(1,max(values));pts=[]
  for px in range(w):
   begin=px*len(values)//w;end=max(begin+1,(px+1)*len(values)//w);v=max(values[begin:end])
   pts.extend((px,h-4-(h-8)*v/maximum))
  cv.create_line(*pts,fill=color,width=1)
 def tools_tab_changed(self,e=None):
  tab=self.notebook.select()
  if self.tools_tab_seen is None:self.tools_tab_seen=tab;return
  if tab!=self.tools_tab_seen:
   self.tools_tab_seen=tab;self.show_tools()
 def show_tools(self):
  self.tools_collapsed=False;self.layout_tools()
 def toggle_tools(self):
  self.tools_collapsed=not self.tools_collapsed;self.layout_tools()
 def layout_tools(self):
  if getattr(self,'video_fullscreen',False):return
  if len(self.video_workspace.panes())<2:return
  tab=self.notebook.select()
  title=self.notebook.tab(tab,'text') if tab else ''
  defaults={'像素 / RGB 直方图':210,'分阶段丢帧 / 跳帧':330,'图像缓存与保存':350,'运行日志':210}
  base=1 if self.tools_collapsed else self.tools_custom_heights.get(title,defaults.get(title,210))
  self.statistics_panel_height=base
  factor=getattr(getattr(self,'responsive',None),'factor',1.)
  page=round(base*factor);self.notebook.configure(height=max(1,page))
  # Include only the tab strip when tools are collapsed; video gets the rest.
  header=max(27,round(34*factor));total=max(1,self.video_workspace.winfo_height())
  height=min(page+header,max(header,total//2)) if not self.tools_collapsed else header+1
  self.video_workspace.sashpos(0,max(80,total-height))
  self.tools_toggle_button.configure(text='展开下方工具' if self.tools_collapsed else '收起工具 · 视频优先')
 def tools_dragged(self,e=None):
  if getattr(self,'video_fullscreen',False) or len(self.video_workspace.panes())<2:return
  height=self.video_workspace.winfo_height()-self.video_workspace.sashpos(0)
  factor=getattr(getattr(self,'responsive',None),'factor',1.)
  self.tools_collapsed=height<max(60,60*factor)
  if not self.tools_collapsed:
   title=self.notebook.tab(self.notebook.select(),'text')
   self.tools_custom_heights[title]=max(90,round(height/factor)-34)
  self.tools_toggle_button.configure(text='展开下方工具' if self.tools_collapsed else '收起工具 · 视频优先')
 def open_snapshot(self,item,raw=True,title='冻结快照'):
  from video_snapshot import SnapshotWindow
  window=getattr(self,'snapshot_window',None)
  if window is None or window.closed:self.snapshot_window=SnapshotWindow(self)
  self.snapshot_window.set_frame(item,raw,title)
  self.footer.set('已打开固定帧 '+str(item[2])+'；主视频、采集和 RDMA 继续运行')
 def browse_cache(self):
  self.gallery_saved=False;self.saved_meta={};self.gallery.delete(*self.gallery.get_children());self.update_cache()
  self.footer.set('实时缓存按 2 fps 独立采样；选择一帧在快照窗口查看')
 def update_cache(self):
  if getattr(self,'gallery_saved',False):return
  self.history_meta={'c%s_%s'%(item[0],item[1]):item for item in reversed(self.cache)}
  wanted=set(self.history_meta)
  for ident in self.gallery.get_children():
   if ident not in wanted:self.gallery.delete(ident)
  for index,(ident,item) in enumerate(self.history_meta.items()):
   values=('#'+str(item[2]),'完整帧缓存','%d × %d'%item[3].size)
   if self.gallery.exists(ident):self.gallery.item(ident,values=values);self.gallery.move(ident,'',index)
   else:self.gallery.insert('',index,iid=ident,values=values)
  self.gallery_mode.configure(text='实时缓存：%d / %d 帧 · 2 fps采样 · 最大192 MiB'%(len(self.cache),self.history_limit))
 def open_gallery(self,e=None):
  sel=self.gallery.selection()
  if not sel:return
  ident=sel[0]
  if ident in self.history_meta:self.open_snapshot(self.history_meta[ident],True,'缓存快照')
  elif ident in getattr(self,'saved_meta',{}):
   data=self.saved_meta[ident];path=(ROOT/data['url'].lstrip('/')).resolve()
   if not path.is_relative_to((ROOT/'captures').resolve()):self.footer.set('拒绝项目外图像路径');return
   def load():
    with Image.open(path) as im:out=im.convert('RGB')
    return out
   def shown(im):
    item=(0,0,data['name'],im,0,time.monotonic())
    self.open_snapshot(item,data['raw'],'已保存图像')
    self.footer.set('正在查看：'+str(path))
   self.submit(load,shown)
 def save_item(self,item,raw=True,callback=None):
  from video_capture_files import save_frame
  def saved(path):
   self.last_saved=path;self.last_saved_var.set('最近保存：'+str(path));self.footer.set('已保存：'+str(path))
   if callback:callback(path)
   if getattr(self,'gallery_saved',False):self.load_saved()
  self.submit(lambda:save_frame(ROOT,item,raw,SIDE),saved)
 def save_current(self):
  if self.frozen:item=self.current
  else:
   with self.reader.lock:item=self.reader.latest
  if not item:self.footer.set('没有可保存的图像');return
  if not self.frozen and time.monotonic()-item[5]>2:self.footer.set('当前没有新鲜帧，拒绝保存过期实时图像');return
  self.save_item(item,self.current_raw if self.frozen else True)
 def open_save_directory(self):
  import subprocess
  directory=ROOT/'captures';directory.mkdir(exist_ok=True)
  if not directory.resolve().is_relative_to(ROOT.resolve()):self.footer.set('保存目录不能指向项目外');return
  try:
   subprocess.Popen(['xdg-open',str(directory)],stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
   self.footer.set('已请求打开保存目录：'+str(directory))
  except OSError as exc:self.footer.set('无法打开文件管理器：'+str(exc)+'；目录：'+str(directory))
 def copy_save_directory(self):
  self.root.clipboard_clear();self.root.clipboard_append(str(ROOT/'captures'));self.footer.set('已复制保存目录：'+str(ROOT/'captures'))
 def load_saved(self):
  def show(d):
   self.gallery_saved=True;self.history_meta={};self.gallery.delete(*self.gallery.get_children());self.saved_meta={}
   for i,item in enumerate(d['files']):
    key='s'+str(i);self.saved_meta[key]=item;rel=item['url'].removeprefix('/captures/')
    self.gallery.insert('','end',iid=key,values=(rel,'PNG 无损图像' if item['raw'] else 'JPEG 图像序列',str(round(item['bytes']/1024))+' KiB'))
   self.gallery_mode.configure(text='已保存文件：最近 %d 张 · 选择后打开分析窗口'%len(d['files']))
  self.submit(lambda:self.request('/api/saved'),show)
 def apply_cache(self):
  try:n=int(self.cache_var.get());fps=int(self.record_var.get())
  except ValueError:self.footer.set('缓存和保存帧率必须为整数');return
  if not 1<=n<=32 or not 1<=fps<=10:self.footer.set('缓存范围1–32（另有192MiB上限），保存采样1–10 fps');return
  self.history_limit=n
  while len(self.cache)>n:self.cache.popleft()
  self.browse_cache()
  self.submit(lambda:self.request('/api/config',{'history':n,'record_fps':fps}),lambda d:self.footer.set('缓存与连续保存参数已应用'))
 def toggle_record(self):
  if getattr(self,'record_pending',False):return
  try:fps=int(self.record_var.get())
  except ValueError:self.footer.set('保存帧率必须为整数');return
  if not 1<=fps<=10:self.footer.set('连续保存支持1–10 fps采样');return
  start=not self.status.get('console',{}).get('recording',False);self.record_pending=True
  def change():
   try:
    if start:self.request('/api/config',{'record_fps':fps})
    return self.request('/api/record',{'start':start})
   finally:self.record_pending=False
  def changed(d):
   self.footer.set(d.get('message',''));self.load_saved()
  self.submit(change,changed)
 def activate_window_manager(self):
  # EWMH activation is needed when the desktop denies ordinary lift/focus calls.
  try:
   import ctypes as ct
   class Data(ct.Union):
    _fields_=[('b',ct.c_char*20),('s',ct.c_short*10),('l',ct.c_long*5)]
   class Client(ct.Structure):
    _fields_=[('type',ct.c_int),('serial',ct.c_ulong),('send_event',ct.c_int),('display',ct.c_void_p),('window',ct.c_ulong),('message_type',ct.c_ulong),('format',ct.c_int),('data',Data)]
   class Event(ct.Union):
    _fields_=[('client',Client),('pad',ct.c_long*24)]
   x=ct.CDLL('libX11.so.6')
   x.XOpenDisplay.argtypes=[ct.c_char_p];x.XOpenDisplay.restype=ct.c_void_p
   x.XCloseDisplay.argtypes=[ct.c_void_p]
   x.XDefaultRootWindow.argtypes=[ct.c_void_p];x.XDefaultRootWindow.restype=ct.c_ulong
   x.XInternAtom.argtypes=[ct.c_void_p,ct.c_char_p,ct.c_int];x.XInternAtom.restype=ct.c_ulong
   x.XQueryTree.argtypes=[ct.c_void_p,ct.c_ulong,ct.POINTER(ct.c_ulong),ct.POINTER(ct.c_ulong),ct.POINTER(ct.POINTER(ct.c_ulong)),ct.POINTER(ct.c_uint)]
   x.XFree.argtypes=[ct.c_void_p]
   x.XSendEvent.argtypes=[ct.c_void_p,ct.c_ulong,ct.c_int,ct.c_long,ct.POINTER(Event)]
   x.XFlush.argtypes=[ct.c_void_p]
   d=x.XOpenDisplay(None)
   if not d:return
   try:
    root=x.XDefaultRootWindow(d);tree_root=ct.c_ulong();parent=ct.c_ulong();children=ct.POINTER(ct.c_ulong)();count=ct.c_uint()
    wid=self.root.winfo_id()
    if x.XQueryTree(d,wid,ct.byref(tree_root),ct.byref(parent),ct.byref(children),ct.byref(count)) and parent.value!=root:wid=parent.value
    if children:x.XFree(children)
    e=Event();e.client.type=33;e.client.send_event=1;e.client.display=d;e.client.window=wid
    e.client.message_type=x.XInternAtom(d,b'_NET_ACTIVE_WINDOW',0);e.client.format=32
    e.client.data.l[0]=2;e.client.data.l[1]=0
    x.XSendEvent(d,root,0,(1<<20)|(1<<19),ct.byref(e));x.XFlush(d)
   finally:x.XCloseDisplay(d)
  except (OSError,tk.TclError):pass
 def bring_to_front(self):
  """Activate on the receiver sync screen, or the mouse screen on Jetson."""
  r=self.root;r.deiconify();r.attributes('-zoomed',False);r.update_idletasks()
  try:
   import ctypes as ct
   class Screen(ct.Structure):
    _fields_=[('number',ct.c_int),('x',ct.c_short),('y',ct.c_short),('width',ct.c_short),('height',ct.c_short)]
   x=ct.CDLL('libX11.so.6');xi=ct.CDLL('libXinerama.so.1')
   x.XOpenDisplay.argtypes=[ct.c_char_p];x.XOpenDisplay.restype=ct.c_void_p
   x.XCloseDisplay.argtypes=[ct.c_void_p];x.XFree.argtypes=[ct.c_void_p]
   xi.XineramaQueryScreens.argtypes=[ct.c_void_p,ct.POINTER(ct.c_int)];xi.XineramaQueryScreens.restype=ct.POINTER(Screen)
   d=x.XOpenDisplay(None)
   if d:
    screens=None
    try:
     count=ct.c_int();screens=xi.XineramaQueryScreens(d,ct.byref(count));px,py=getattr(self,'display_center',r.winfo_pointerxy())
     for i in range(count.value if screens else 0):
      m=screens[i]
      if m.x<=px<m.x+m.width and m.y<=py<m.y+m.height:
       w=min(1660,max(800,m.width-80));h=min(1020,max(600,m.height-110))
       r.geometry('%dx%d+%d+%d'%(w,h,m.x+(m.width-w)//2,m.y+(m.height-h)//2));break
    finally:
     if screens:x.XFree(screens)
     x.XCloseDisplay(d)
  except (OSError,tk.TclError):pass
  if self.args.maximized:r.after(100,lambda:r.attributes('-zoomed',True) if not self.closed.is_set() else None)
  r.attributes('-topmost',True);r.lift();r.focus_force();self.activate_window_manager()
  r.after(250,self.activate_window_manager)
  r.after(800,lambda:r.attributes('-topmost',False) if not self.closed.is_set() else None)
  r.update_idletasks()
  (ROOT/'runtime/desktop.raised.json').write_text(json.dumps({'pid':os.getpid(),'display':os.environ.get('DISPLAY'),'geometry':r.geometry(),'state':r.state(),'at':time.time()}))
 def poll_raise(self):
  p=ROOT/'runtime/desktop.raise'
  if p.exists():
   p.unlink(missing_ok=True);self.bring_to_front()
  if not self.closed.is_set():self.root.after(200,self.poll_raise)
 def screenshot(self,name):
  self.root.update_idletasks()
  x=self.root.winfo_rootx();y=self.root.winfo_rooty();w=self.root.winfo_width();h=self.root.winfo_height()
  path=ROOT/'validation'/name;path.parent.mkdir(exist_ok=True)
  ImageGrab.grab(bbox=(x,y,x+w,y+h)).save(path);return path
 def console_check(self):
  try:
   if time.monotonic()-self.console_check_started>50:raise AssertionError('console QA timeout')
   phase=self.console_check_phase
   if phase==0 and self.current_analysis and self.current:
    import hashlib
    assert [sum(h) for h in self.current_analysis['rgb_histogram']]==[1920*1080]*3
    self.freeze(False);self.console_check_hash=hashlib.sha256(self.current[3].tobytes()).hexdigest()
    self.process_vars['enabled'].set(True);self.process_vars['red'].set('2');self.apply_processing();self.settings_tabs.select(2);self.console_check_phase=1
   elif phase==1 and self.processed_override:
    p=self.current[3].getpixel((0,0));q=self.display_image().getpixel((0,0));assert q==(min(255,p[0]*2),p[1],p[2])
    self.save_processed();self.analysis_source.set('本机处理后');self.refresh_analysis();self.console_check_phase=2
   elif phase==2 and getattr(self,'last_processed_saved',None) and self.current_analysis and self.current_analysis.get('source')=='本机处理后':
    with Image.open(self.last_processed_saved) as im:assert im.size==(1920,1080)
    self.screenshot('tabs_processing_jetson.png');self.settings_tabs.select(1);self.console_check_phase=3
   elif phase==3:
    import hashlib
    assert hashlib.sha256(self.current[3].tobytes()).hexdigest()==self.console_check_hash
    self.screenshot('tabs_pressure_jetson.png');self.reset_processing();self.analysis_source.set('原始图像');self.resume();self.settings_tabs.select(0)
    (ROOT/'validation/console_check.json').write_text(json.dumps({'status':'PASS','rgb_channel_pixels':[2073600]*3,'original_unchanged':True,'processed_png':self.last_processed_saved,'tabs':[self.settings_tabs.tab(x,'text') for x in self.settings_tabs.tabs()]},ensure_ascii=False,indent=2))
    print('TABS / RGB / PROCESSING / EXPORT PASS',flush=True);return
   self.root.after(200,self.console_check)
  except Exception as e:print('CONSOLE CHECK FAIL:',e,flush=True)
 def ui_snapshot(self):
  d={'checkmark_style':'solid','checkmark_size':self.checkmarks.images['on'].height(),'responsive':self.responsive.status(),'gpu':self.gpu.status() if self.gpu else None,'gpu_failed':self.gpu_failed,'render_note':self.render_note.cget('text'),'display_target':self.vars['display_fps'].get(),'tabs':[self.settings_tabs.tab(x,'text') for x in self.settings_tabs.tabs()],'rgb_histogram_pixels':[sum(h) for h in self.current_analysis.get('rgb_histogram',[])] if self.current_analysis else None,'title':self.root.title(),'side':SIDE,'analysis_updates':self.analysis_updates,'analysis_label':self.analysis_label.cget('text'),
   'analysis_seq':self.current_analysis.get('seq') if self.current_analysis else None,
   'histogram_pixels':sum(self.current_analysis['histogram']) if self.current_analysis else None,
   'isp_control':hasattr(self,'isp_check'),'isp_pending':read_isp_mask() if JETSON else None,
   'frozen':self.frozen,'raw_size':list(self.current[3].size) if self.current else None}
  (ROOT/'validation').mkdir(exist_ok=True)
  (ROOT/'validation'/('analysis_'+('jetson' if JETSON else 'mellanox')+'.json')).write_text(json.dumps(d,ensure_ascii=False,indent=2))
  self.screenshot('analysis_'+('jetson' if JETSON else 'mellanox')+'.png')
  return d
 def analysis_check(self):
  try:
   now=time.monotonic()
   if now-self.analysis_check_start>45:raise AssertionError('no advancing camera analysis')
   phase=self.analysis_check_phase
   if phase==0 and self.analysis_updates>=3 and self.current_analysis:
    self.analysis_check_seq=self.current_analysis['seq'];self.freeze(False);self.analysis_check_phase=1
   elif phase==1 and self.current_analysis and not self.analysis_busy and not self.analysis_pending:
    self.analysis_check_frozen_seq=self.current_analysis['seq'];self.analysis_check_updates=self.analysis_updates;self.analysis_check_at=now;self.analysis_check_phase=2
   elif phase==2 and now-self.analysis_check_at>=1.2:
    assert self.frozen and self.analysis_updates==self.analysis_check_updates
    assert self.current_analysis['seq']==self.analysis_check_frozen_seq
    self.resume();self.analysis_check_phase=3
   elif phase==3 and self.current_analysis and self.current_analysis['seq']!=self.analysis_check_frozen_seq:
    assert self.current_analysis['live'] and sum(self.current_analysis['histogram'])==1920*1080
    assert hasattr(self,'isp_check')==JETSON
    d=self.ui_snapshot();d['live_freeze_resume']='PASS'
    (ROOT/'validation/analysis_check.json').write_text(json.dumps(d,ensure_ascii=False,indent=2))
    print('LIVE / FREEZE / RESUME ANALYSIS PASS',flush=True);return
   self.root.after(150,self.analysis_check)
  except Exception as e:print('ANALYSIS CHECK FAIL:',e,flush=True)
 def self_test(self):
  try:
   age=time.monotonic()-self.test_started
   if age>140:raise AssertionError('native UI test timeout')
   c=self.status.get('console',{})
   if self.test_state==0 and c.get('controller') and c.get('peer_controller'):
    self.start_button.invoke();self.test_state=1
   elif self.test_state==1 and self.current and self.display_count>8:
    assert self.current[3].size==(1920,1080)
    self.freeze(True);self.set_zoom('100%');self.test_state=2
   elif self.test_state==2 and self.current_analysis and getattr(self,'last_saved',None):
    assert self.drawn['rendered_width']==1920 and self.drawn['rendered_height']==1080
    d=self.current_analysis;assert tuple(d['rgb'])==self.current[3].getpixel((d['x'],d['y']))
    assert sum(d['histogram'])==1920*1080
    self.test_results={'raw_size':self.current[3].size,'one_to_one':self.drawn,'pixel':d['rgb'],'histogram_sum':sum(d['histogram']),'saved_png':str(self.last_saved)}
    self.set_zoom('适应窗口');self.test_state=3
   elif self.test_state==3:
    self.screenshot('native_'+('jetson' if JETSON else 'mellanox')+'.png')
    self.test_results['fit']=self.drawn;self.test_results['display_count']=self.display_count
    self.test_results['stages']=list(self.stage_rows.values());self.test_results['status']='PASS'
    (ROOT/'validation'/('native_'+('jetson' if JETSON else 'mellanox')+'.json')).write_text(json.dumps(self.test_results,ensure_ascii=False,indent=2))
    print('NATIVE SELF TEST PASS',flush=True);self.resume();self.test_state=4
    if self.args.self_test_stop:self.stop_button.invoke()
   elif self.test_state==4 and self.args.self_test_stop and not c.get('busy') and not any(c.get(k,{}).get('running') for k in ('capture','sender','receiver')):
    assert c.get('sender',{}).get('passed') and c.get('receiver',{}).get('passed'), 'transfer did not finish cleanly'
    row=self.stage_rows['rdma_missing'];assert row['value']==0, row
    self.test_results['final_status']=c
    (ROOT/'validation/native_stop.json').write_text(json.dumps(self.test_results,ensure_ascii=False,indent=2))
    print('NATIVE STOP PASS: submitted == received; CRC errors = '+str(c['receiver'].get('crc_errors')),flush=True);self.test_state=5
   if self.test_state<4 or self.test_state==4 and self.args.self_test_stop:self.root.after(300,self.self_test)
  except Exception as e:
   print('NATIVE SELF TEST FAIL:',e,flush=True);self.footer.set('验证失败：'+str(e));self.test_state=-1
 def close(self):
  if self.closed.is_set():return
  c=getattr(self,'status',{}).get('console',{})
  queue_active=False
  try:
   import struct
   with self.display_path.open('rb') as f:
    header=f.read(104)
   queue_active=len(header)==104 and struct.unpack_from('<I',header,96)[0]==0
  except OSError:pass
  if queue_active or c.get('busy') or any(c.get(k,{}).get('running') for k in ('capture','sender','receiver')):
   if any(c.get(k,{}).get('safety_hold') for k in ('capture','sender','receiver')):
    self.close_pending=False;self.footer.set('DMA 安全保持：请先复位并安全停止，再关闭窗口');return
   if not getattr(self,'close_pending',False):
    self.close_pending=True;self.close_started=time.monotonic()
    self.footer.set('关闭前正在安全停止双端采集并排空显示队列')
    self.submit(lambda:self.request('/api/pipeline',{'action':'stop','config':{}}))
   if time.monotonic()-self.close_started>90:
    self.close_pending=False;self.footer.set('安全停止尚未完成，窗口和显示队列仍保留；请检查两端状态');return
   self.root.after(300,self.close);return
  self.box_reader.close()
  if self.gpu:self.gpu.close();self.gpu=None
  with self.future_lock:
   self.closed.set();pending=tuple(self.pending_futures)
  self.reader.stop.set()
  # Jetson ships Python 3.8: cancel_futures was added in Python 3.9.
  # Cancel tracked, unstarted UI jobs using the public Future API instead.
  for future in pending:future.cancel()
  self.pool.shutdown(wait=False)
  self.root.destroy()

def main():
 if os.geteuid()==0 and ROOT.stat().st_uid!=0:
  import pwd
  user=pwd.getpwuid(ROOT.stat().st_uid);os.initgroups(user.pw_name,user.pw_gid);os.setgid(user.pw_gid);os.setuid(user.pw_uid)
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--maximized',action='store_true');ap.add_argument('--self-test',action='store_true');ap.add_argument('--self-test-stop',action='store_true');ap.add_argument('--console-check',action='store_true');ap.add_argument('--ui-check',action='store_true');ap.add_argument('--analysis-check',action='store_true');args=ap.parse_args()
 (ROOT/'runtime').mkdir(exist_ok=True);(ROOT/'captures').mkdir(exist_ok=True)
 if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
  print('请在本端图形桌面的终端执行 ./video.sh viewer；SSH 不会自动获得桌面 DISPLAY。');return 2
 lock=(ROOT/'runtime/desktop.lock').open('a+')
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:
  ack=ROOT/'runtime/desktop.raised.json';ack.unlink(missing_ok=True)
  (ROOT/'runtime/desktop.raise').touch()
  for _ in range(40):
   if ack.exists():print('已激活本地原生窗口：RDMA（不是网页）');return 0
   time.sleep(.1)
  print('本地窗口进程未响应激活请求；请查看 runtime/desktop.log');return 1
 try:os.nice(5)
 except OSError:pass
 root=tk.Tk();root.tk.call('tk','scaling',4/3)
 global FONT_SCALE
 probe=tkfont.Font(root=root,family='DejaVu Sans',size=-100)
 FONT_SCALE=max(.75,min(3,probe.measure('M')/86.0))
 for name in ('TkDefaultFont','TkTextFont','TkFixedFont','TkMenuFont','TkHeadingFont','TkCaptionFont','TkSmallCaptionFont','TkIconFont','TkTooltipFont'):
  tkfont.nametofont(name).configure(size=font_px(13))
 app=Workstation(root,args);(ROOT/'runtime/desktop.pid').write_text(str(os.getpid()))
 if args.maximized:root.after(450,lambda:root.attributes('-zoomed',True))
 signal.signal(signal.SIGTERM,lambda *_:root.after_idle(app.close));signal.signal(signal.SIGINT,lambda *_:root.after_idle(app.close))
 try:root.mainloop()
 finally:lock.close()
 return 0
if __name__=='__main__':raise SystemExit(main())
