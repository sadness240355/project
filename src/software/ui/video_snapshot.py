"""A frozen analysis window that never pauses the live GPU/DMA queue."""
import tkinter as tk
from tkinter import ttk
from PIL import ImageTk,Image
from video_analysis import analyze_frame
from video_pixels import BAYER
RESAMPLE=getattr(Image,'Resampling',Image)

class SnapshotWindow:
 def __init__(self,owner):
  self.owner=owner;self.closed=False;self.version=0;self.item=None;self.pending=None
  self.window=tk.Toplevel(owner.root);self.window.title(owner.root.title().split(' · ')[0]+' · 冻结快照 / 分析')
  self.window.configure(bg='#0b1220');self.window.geometry('1100x800');self.window.minsize(760,580)
  self.window.protocol('WM_DELETE_WINDOW',self.close)
  bar=ttk.Frame(self.window,padding=8);bar.pack(fill='x')
  self.info=ttk.Label(bar,text='冻结快照',justify='left');self.info.pack(side='left',fill='x',expand=True)
  ttk.Button(bar,text='保存这张图像',command=self.save).pack(side='right',padx=4)
  ttk.Button(bar,text='打开保存目录',command=owner.open_save_directory).pack(side='right',padx=4)
  self.zoom=tk.StringVar(value='适应窗口')
  zoom=ttk.Combobox(bar,textvariable=self.zoom,values=['适应窗口','100%','200%','400%'],state='readonly',width=10)
  zoom.pack(side='right',padx=4);zoom.bind('<<ComboboxSelected>>',lambda e:self.draw())
  viewport=ttk.Frame(self.window);viewport.pack(fill='both',expand=True,padx=8)
  self.canvas=tk.Canvas(viewport,bg='#070e19',highlightthickness=0)
  ys=ttk.Scrollbar(viewport,orient='vertical',command=lambda *args:self.scroll('y',*args));ys.pack(side='right',fill='y')
  xs=ttk.Scrollbar(viewport,orient='horizontal',command=lambda *args:self.scroll('x',*args));xs.pack(side='bottom',fill='x')
  self.canvas.configure(xscrollcommand=xs.set,yscrollcommand=ys.set);self.canvas.pack(fill='both',expand=True)
  self.canvas.bind('<Configure>',lambda e:self.draw());self.canvas.bind('<Button-1>',self.pixel)
  self.canvas.bind('<Button-4>',lambda e:self.scroll('y','scroll',-3,'units'))
  self.canvas.bind('<Button-5>',lambda e:self.scroll('y','scroll',3,'units'))
  self.canvas.bind('<MouseWheel>',lambda e:self.scroll('y','scroll',-3 if e.delta>0 else 3,'units'))
  self.stats=ttk.Label(self.window,text='点击快照可查看像素和行列曲线',padding=8,justify='left',wraplength=1050);self.stats.pack(fill='x')
  charts=ttk.Frame(self.window,padding=(8,0));charts.pack(fill='x')
  self.charts={}
  for col,(key,title) in enumerate([('rgb_histogram','RGB 直方图'),('horizontal','选中行灰度曲线'),('vertical','选中列灰度曲线')]):
   group=ttk.Frame(charts);group.grid(row=0,column=col,sticky='ew',padx=4);charts.columnconfigure(col,weight=1)
   ttk.Label(group,text=title).pack(anchor='w')
   cv=tk.Canvas(group,height=90,bg='#142035',highlightthickness=0);cv.pack(fill='x');self.charts[key]=cv
  self.message=tk.StringVar(value='只冻结这张快照；主视频、采集和 RDMA 继续运行。')
  ttk.Label(self.window,textvariable=self.message,padding=8,wraplength=1050,justify='left').pack(fill='x')
 def set_frame(self,item,raw=True,title='冻结快照'):
  self.item=item;self.raw=raw;self.version+=1;self.analysis=None;self.point=(item[3].width//2,item[3].height//2)
  fmt='Bayer 原始帧的去马赛克预览' if len(item)>8 and item[8]==BAYER else '无损 RGB 图像' if raw else 'JPEG 有损图像'
  self.info.configure(text=title+' · 帧 '+str(item[2])+' · %d × %d'%item[3].size+'\n'+fmt+' · 主视频继续运行')
  self.message.set('只冻结这张快照；主视频、采集和 RDMA 继续运行。')
  self.canvas.xview_moveto(0);self.canvas.yview_moveto(0);self.draw();self.analyze(*self.point)
  self.window.deiconify();self.window.lift()
 def draw(self):
  if self.closed or self.item is None:return
  im=self.item[3];cw=max(1,self.canvas.winfo_width());ch=max(1,self.canvas.winfo_height())
  self.scale=min(cw/im.width,ch/im.height) if self.zoom.get()=='适应窗口' else float(self.zoom.get().strip('%'))/100
  w=max(1,round(im.width*self.scale));h=max(1,round(im.height*self.scale))
  self.ox=max(0,(cw-w)/2);self.oy=max(0,(ch-h)/2)
  self.canvas.configure(scrollregion=(0,0,max(cw,w),max(ch,h)))
  if self.scale>1:
   x=max(0,int(self.canvas.canvasx(0)/self.scale));y=max(0,int(self.canvas.canvasy(0)/self.scale))
   right=min(im.width,x+int(cw/self.scale)+2);bottom=min(im.height,y+int(ch/self.scale)+2)
   shown=im.crop((x,y,right,bottom)).resize((max(1,round((right-x)*self.scale)),max(1,round((bottom-y)*self.scale))),RESAMPLE.NEAREST)
   px=x*self.scale;py=y*self.scale
  else:
   shown=im.resize((w,h),RESAMPLE.BILINEAR) if im.size!=(w,h) else im;px=self.ox;py=self.oy
  self.photo=ImageTk.PhotoImage(shown);self.canvas.delete('all');self.canvas.create_image(px,py,image=self.photo,anchor='nw')
  if getattr(self,'analysis',None):self.plot(self.analysis)
 def scroll(self,axis,*args):
  (self.canvas.xview if axis=='x' else self.canvas.yview)(*args);self.draw()
 def pixel(self,e):
  if not self.item:return
  x=int((self.canvas.canvasx(e.x)-self.ox)/self.scale);y=int((self.canvas.canvasy(e.y)-self.oy)/self.scale)
  if 0<=x<self.item[3].width and 0<=y<self.item[3].height:self.analyze(x,y)
 def analyze(self,x,y):
  if self.closed:return
  self.version+=1;version=self.version;item=self.item;self.point=(x,y)
  def done(d):
   if self.closed or version!=self.version:return
   self.analysis=d
   sample=(' · 原始 Bayer=%d'%item[9][d['y']*item[3].width+d['x']] if len(item)>9 and item[8]==BAYER and item[9] is not None else '')
   self.stats.configure(text='固定帧 %s · X=%d Y=%d · R=%d G=%d B=%d%s\n灰度均值 %.2f · 标准差 %.2f · 最小 %d / 最大 %d · 饱和 %.3f%%'%((str(item[2]),d['x'],d['y'],*d['rgb'],sample,d['mean'],d['stddev'],*d['extrema'],d['saturation'])))
   self.plot(d)
  self.owner.submit(lambda:analyze_frame(item[3],item[2],x,y),done)
 def plot(self,d):
  for key,cv in self.charts.items():
   cv.delete('all');w=max(2,cv.winfo_width());h=max(8,cv.winfo_height())
   channels=d[key] if key=='rgb_histogram' else [d[key]]
   peak=max(1,max(max(c) for c in channels)) if key=='rgb_histogram' else 255
   for values,color in zip(channels,('#ff6b78','#62d994','#65a9ff') if key=='rgb_histogram' else ('#59d5c6',)):
    points=[]
    for px in range(w):
     start=px*len(values)//w;end=max(start+1,(px+1)*len(values)//w)
     points.extend((px,h-4-(h-8)*max(values[start:end])/peak))
    cv.create_line(*points,fill=color,width=1)
 def save(self):
  if self.item:self.owner.save_item(self.item,self.raw,self.saved)
 def saved(self,path):
  if not self.closed:self.message.set('已保存：'+str(path))
 def close(self):
  self.closed=True;self.version+=1;self.window.destroy()
