"""Tabbed controls inspired by KU15P, limited to this RDMA application's capabilities."""
import tkinter as tk
from tkinter import ttk
def sidebar(app,parent,panel):
 box=ttk.Frame(parent,width=400);box.pack(side='left',fill='y',padx=(0,12));box.pack_propagate(False);app.sidebar_box=box
 app.sidebar_actions=ttk.Frame(box,padding=(5,6));app.sidebar_actions.pack(side='bottom',fill='x')
 app.settings_tabs=ttk.Notebook(box);app.settings_tabs.pack(fill='both',expand=True)
 pages=[];app.settings_scroll_pages=[]
 for name in ('采集设置','压力测试','图像处理','传输与性能'):
  shell=ttk.Frame(app.settings_tabs);app.settings_tabs.add(shell,text=name)
  canvas=tk.Canvas(shell,bg=panel,highlightthickness=0,width=320)
  scroll=ttk.Scrollbar(shell,orient='vertical',command=canvas.yview);scroll.pack(side='right',fill='y')
  canvas.configure(yscrollcommand=scroll.set);canvas.pack(side='left',fill='both',expand=True)
  page=ttk.Frame(canvas,padding=10);ident=canvas.create_window((0,0),window=page,anchor='nw')
  page.bind('<Configure>',lambda e,c=canvas:c.configure(scrollregion=c.bbox('all')))
  canvas.bind('<Configure>',lambda e,c=canvas,i=ident:c.itemconfigure(i,width=e.width));pages.append(page);app.settings_scroll_pages.append((canvas,page))
 app.test_page=pages[1];app.processing_page=pages[2];app.performance_page=pages[3]
 return pages[0]
def test_panel(app,p):
 ttk.Label(p,text='DMA / RDMA 独立彩条带宽测试',style='Title.TLabel').pack(anchor='w',pady=(2,4))
 ttk.Label(p,text='FPGA DDR 固定 RGB 彩条 · 相机关闭\n按完成字节计速，接收端抽样 CRC',style='Muted.TLabel').pack(anchor='w',pady=3)
 app.test_dma=tk.BooleanVar(value=True);app.test_rdma=tk.BooleanVar(value=True)
 ttk.Checkbutton(p,text='PCIe DMA：DDR → Jetson',variable=app.test_dma).pack(anchor='w',pady=2)
 ttk.Checkbutton(p,text='LRDMA：DDR → 远端注册内存',variable=app.test_rdma).pack(anchor='w',pady=2)
 app.test_vars={'preset':tk.StringVar(value='自定义')}
 for key,label,value,choices in [('seconds','持续时间 / 秒','60',None),('size','彩条尺寸','2400x2048',['1920x1080','2400x2048'])]:
  row=ttk.Frame(p);row.pack(fill='x',pady=3);ttk.Label(row,text=label,width=14).pack(side='left');v=tk.StringVar(value=value);app.test_vars[key]=v
  w=ttk.Combobox(row,textvariable=v,values=choices,state='readonly',width=13) if choices else ttk.Entry(row,textvariable=v,width=15);w.pack(side='right',fill='x',expand=True)
 buttons=ttk.Frame(p);buttons.pack(fill='x',pady=5)
 app.test_start=ttk.Button(buttons,text='▶ 开始测试',style='Start.TButton',command=lambda:app.pressure_action('start'));app.test_start.pack(side='left',fill='x',expand=True)
 app.test_stop=ttk.Button(buttons,text='■ 停止并排空',command=lambda:app.pressure_action('stop'));app.test_stop.pack(side='left',fill='x',expand=True)
 app.test_progress=ttk.Progressbar(p,maximum=100);app.test_progress.pack(fill='x',pady=4)
 app.test_status=ttk.Label(p,text='尚未开始测试',wraplength=380,style='Title.TLabel');app.test_status.pack(fill='x',pady=3)
 app.test_metrics=ttk.Label(p,text='DMA / RDMA — MiB/s',wraplength=380,style='Muted.TLabel');app.test_metrics.pack(fill='x',pady=4)
 ttk.Button(p,text='保存测试报告摘要',command=app.save_test_summary).pack(fill='x',pady=3)
 app.test_report=ttk.Label(p,text='完整报告保存于接收端 results 目录',wraplength=380,style='Muted.TLabel');app.test_report.pack(fill='x',pady=2)
def processing_panel(app,p):
 from video_config import DEFAULT_CCM
 ttk.Label(p,text='FPGA 硬件图像处理',style='Title.TLabel').pack(anchor='w',pady=(5,10))
 ttk.Label(p,text='RGB 数据在 FPGA 内完成处理，再独立写入 DDR/RDMA 和 Jetson 主机直写缓冲。Bayer 原始流旁路这些处理。',style='Muted.TLabel',wraplength=300).pack(fill='x',pady=8)
 app.isp_bits=[];app.isp_widgets=[];app.ccm_vars=[]
 if not hasattr(app,'isp_var'):
  ttk.Label(p,text='由 Jetson 发送端统一设置，两端显示同一份处理结果。',style='Muted.TLabel',wraplength=300).pack(fill='x',pady=10)
 else:
  from video_isp import read_isp_selection
  mask=read_isp_selection()
  for i,label in enumerate(('黑电平扣除','白平衡增益','3×3 色彩矩阵 CCM','Gamma / 色调校正')):
   v=tk.BooleanVar(value=bool(mask&(1<<i)));app.isp_bits.append(v)
   w=ttk.Checkbutton(p,text=label,variable=v,command=lambda:app.set_isp(child=True));w.pack(anchor='w',pady=4);app.isp_widgets.append(w)
  app.field(p,'black_level','黑电平 / 0–255','0')
  for key,title,value in [('wb_r','白平衡 R','1.01'),('wb_g','白平衡 G','1.04'),('wb_b','白平衡 B','0.95')]:app.field(p,key,title,value)
  ttk.Label(p,text='CCM 系数 · 行优先 3×3',style='Muted.TLabel').pack(anchor='w',pady=(12,4))
  grid=ttk.Frame(p);grid.pack(fill='x')
  for i,value in enumerate(DEFAULT_CCM):
   v=tk.StringVar(value=str(value));app.ccm_vars.append(v);w=ttk.Entry(grid,textvariable=v,width=8);w.grid(row=i//3,column=i%3,padx=2,pady=2,sticky='ew');app.isp_widgets.append(w)
  for i in range(3):grid.columnconfigure(i,weight=1)
  app.online_note=ttk.Label(p,text='在线调参能力检测中…',style='Muted.TLabel',wraplength=300);app.online_note.pack(fill='x',pady=8)
  app.ext_bits={};app.ext_widgets=[]
  ttk.Separator(p).pack(fill='x',pady=12)
  ttk.Label(p,text='ISP 扩展 · 需要 UIP2 位流',style='Title.TLabel').pack(anchor='w')
  for bit,label in ((4,'同色邻点空间降噪'),(16,'坏点抑制'),(32,'细节锐化（RGB）'),(8,'自定义 Gamma / 对比度（替代固定 Gamma）')):
   v=tk.BooleanVar(value=False);app.ext_bits[bit]=v
   w=ttk.Checkbutton(p,text=label,variable=v,state='disabled');w.pack(anchor='w',pady=4);app.ext_widgets.append(w)
  for key,label,value in [('threshold','邻点阈值 / 0–255','16'),('defect_threshold','坏点阈值 / 0–255','32'),('sharp_strength','锐化强度 / 0–2','1.0'),('sharp_coring','锐化门槛 / 0–63','4'),('gamma','Gamma / 0.2–3','1.0'),('contrast','对比度 / 0.25–4','1.0')]:
   app.field(p,key,label,value);w=getattr(app,key+'_widget');w.configure(state='disabled');app.ext_widgets.append(w)
  app.isp_capability=ttk.Label(p,text='等待发送端硬件能力回读。',style='Muted.TLabel',wraplength=300);app.isp_capability.pack(fill='x',pady=8)
  ttk.Label(p,text='基础 ISP 沿用已有硬件。扩展默认关闭；自定义曲线替代固定 Gamma；停止后修改，开始采集时写入并核对。',style='Muted.TLabel',wraplength=300).pack(fill='x',pady=12)
 app.processing_note=ttk.Label(p,text='全部图像处理均在发送 FPGA 执行；本地与接收端显示同一处理结果。直方图独立采样，不修改像素。',style='Muted.TLabel',wraplength=300);app.processing_note.pack(fill='x',pady=10)

def install_settings_wheel(app):
 """A tag before widget/class bindings prevents wheel edits in entry/combo fields.
 Only descendants of a settings canvas are tagged; image zoom is unaffected.
 """
 app.settings_wheel_tags=[]
 for index,(canvas,page) in enumerate(app.settings_scroll_pages):
  tag='RDMASettingsWheel_'+str(id(app))+'_'+str(index)
  def wheel(event,c=canvas):
   if not c.winfo_ismapped():return
   num=getattr(event,'num',None);delta=getattr(event,'delta',0)
   steps=-3 if num==4 else 3 if num==5 else (-max(1,abs(int(delta))//120)* (1 if delta>0 else -1) if delta else 0)
   if steps and c.yview()!=(0.,1.):c.yview_scroll(steps,'units')
   return 'break'
  for sequence in ('<Button-4>','<Button-5>','<MouseWheel>'):canvas.bind_class(tag,sequence,wheel)
  def attach(widget):
   widget.bindtags((tag,)+tuple(t for t in widget.bindtags() if t!=tag))
   for child in widget.winfo_children():attach(child)
  attach(canvas);app.settings_wheel_tags.append(tag)
