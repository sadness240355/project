#!/usr/bin/env python3
"""Compute every plot and statistic from the same immutable full-resolution frame."""
from PIL import ImageStat
def analyze_frame(im,seq,x,y):
 x=max(0,min(im.width-1,int(x)));y=max(0,min(im.height-1,int(y)))
 channels=im.histogram();rgb_histogram=[channels[i*256:(i+1)*256] for i in range(3)]
 gray=im.convert('L');hist=gray.histogram();stat=ImageStat.Stat(hist)
 return dict(rgb_histogram=rgb_histogram,rgb_mean=ImageStat.Stat(channels).mean,seq=seq,x=x,y=y,rgb=im.getpixel((x,y)),histogram=hist,
  horizontal=list(gray.crop((0,y,im.width,y+1)).getdata()),
  vertical=list(gray.crop((x,0,x+1,im.height)).getdata()),
  mean=stat.mean[0],stddev=stat.stddev[0],extrema=gray.getextrema(),
  saturation=100*hist[255]/(im.width*im.height))
