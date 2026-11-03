"""Resize native controls without recreating the GPU drawable or restarting capture."""
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

class Responsive:
 def __init__(self,app,on_scale):
  self.app=app;self.root=app.root;self.on_scale=on_scale
  self.factor=1.;self.pending=None;self.fonts={};self.wraps=[];self.canvases=[];self.columns=[]
  self.style=ttk.Style(self.root)
  for name in ('TkDefaultFont','TkTextFont','TkFixedFont','TkMenuFont','TkHeadingFont','TkCaptionFont','TkSmallCaptionFont','TkIconFont','TkTooltipFont'):
   f=tkfont.nametofont(name);self.fonts['named:'+name]=(f,int(f.cget('size')))
  for name in ('.','Muted.TLabel','Title.TLabel','Treeview','Treeview.Heading'):
   spec=self.style.configure(name).get('font')
   if spec:self.style.configure(name,font=self.font(spec))
  self.walk(self.root)
  self.root.bind('<Configure>',self.configure,add='+')
  self.root.after(400,self.apply)
 def font(self,spec):
  key='font:'+str(spec)
  if key not in self.fonts:
   f=tkfont.Font(root=self.root,font=spec);self.fonts[key]=(f,int(f.cget('size')))
  return self.fonts[key][0]
 def walk(self,w):
  keys=w.keys()
  if 'font' in keys and w.cget('font'):
   w.configure(font=self.font(w.cget('font')))
  if 'wraplength' in keys:
   n=self.root.winfo_pixels(w.cget('wraplength') or 0)
   if n>0:self.wraps.append((w,n))
  if isinstance(w,tk.Canvas) and w is not self.app.canvas and w not in [c for c,p in self.app.settings_scroll_pages]:
   h=self.root.winfo_pixels(w.cget('height'))
   if h>0:self.canvases.append((w,h))
  if isinstance(w,ttk.Treeview):
   for c in w['columns']:self.columns.append((w,c,int(w.column(c,'width')),int(w.column(c,'minwidth'))))
  for child in w.winfo_children():self.walk(child)
 def configure(self,event):
  if event.widget is not self.root:return
  if self.pending:self.root.after_cancel(self.pending)
  self.pending=self.root.after(120,self.apply)
 def apply(self):
  self.pending=None
  if self.app.closed.is_set() or self.app.video_fullscreen:return
  w,h=self.root.winfo_width(),self.root.winfo_height()
  if w<100 or h<100:return
  # Quantization prevents font/layout feedback while the window manager resizes.
  factor=round(max(.8,min(2.25,min(w/1660,h/930)))*40)/40
  if abs(factor-self.factor)<.012:return
  self.factor=factor;self.on_scale(factor)
  for f,size in self.fonts.values():
   sign=-1 if size<0 else 1
   f.configure(size=sign*max(1,round(abs(size)*factor)))
  for widget,n in self.wraps:widget.configure(wraplength=round(n*factor))
  for widget,n in self.canvases:widget.configure(height=round(n*factor))
  for widget,column,width,minimum in self.columns:
   widget.column(column,width=round(width*factor),minwidth=round(minimum*factor))
  self.app.sidebar_box.configure(width=round(400*factor))
  self.app.notebook.configure(height=round(getattr(self.app,'statistics_panel_height',230)*factor))
  if hasattr(self.app,'layout_tools'):self.app.layout_tools()
  self.style.configure('Treeview',rowheight=round(28*factor))
  for name,padding in [('TButton',(9,7)),('TNotebook.Tab',(8,8)),('TCheckbutton',(3,3)),('TEntry',(5,5)),('TSpinbox',(4,4)),('TCombobox',(4,4))]:
   self.style.configure(name,padding=tuple(round(x*factor) for x in padding))
  self.app.dirty=True
 def status(self):
  return dict(scale=self.factor,window=[self.root.winfo_width(),self.root.winfo_height()],
              sidebar=self.app.sidebar_box.winfo_width(),
              default_font_px=abs(int(tkfont.nametofont('TkDefaultFont').cget('size'))))
