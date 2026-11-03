#!/usr/bin/env python3
import os,webbrowser
url='http://'+('10.23.68.0' if os.uname().machine=='aarch64' else '10.23.66.188')+':8788/'
if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
    webbrowser.open(url)
print('交互式图像工作站：'+url)
print('双端监看：http://10.23.66.188:8788/dual')
