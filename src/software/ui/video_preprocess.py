"""Read/apply optional UIP1 registers without resetting or rebinding the FPGA."""
import os
from contextlib import contextmanager
import mmap
import struct
import time
from uip_profile import profile

RESOURCE=os.environ.get('FPGA_RESOURCE','/sys/bus/pci/devices/0005:01:00.0/resource2')
BASE=0x14200

@contextmanager
def registers(writable=False):
    with open(RESOURCE,'r+b' if writable else 'rb',buffering=0) as f:
        with mmap.mmap(f.fileno(),0x15000,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ|(mmap.PROT_WRITE if writable else 0)) as mem:
            yield mem

def read(mem,offset):return struct.unpack_from('<I',mem,BASE+offset)[0]
def write(mem,offset,value):struct.pack_into('<I',mem,BASE+offset,value)

def status_from(mem):
    if read(mem,0)!=0x55495031 or read(mem,4) not in (0x10000,0x10001):return dict(available=False)
    result=dict(available=True,abi=read(mem,4),capabilities=read(mem,8),status=read(mem,12),generation=read(mem,0x34))
    for name,offset in [('enable',0x50),('black',0x54),('gain_r',0x58),('gain_g',0x5c),('gain_b',0x60),('threshold',0x64),('phase',0x68)]:result[name]=read(mem,offset)
    if result['abi']==0x10001:
        for name,offset in [('defect_threshold',0x6c),('sharp_strength',0x70),('sharp_coring',0x74)]:result[name]=read(mem,offset)
        result['ccm']=[struct.unpack('<i',struct.pack('<I',read(mem,0xc0+i*4)))[0] for i in range(9)]
    return result

def status():
    try:
        with registers() as mem:return status_from(mem)
    except OSError as exc:return dict(available=False,error=str(exc))

def apply(values,*,reset_stopped_capture=False):
    p=profile(values)
    with registers(True) as mem:
        before=status_from(mem)
        if not before['available']:raise ValueError('当前 FPGA 没有 UIP1 通用预处理模块，需要更新位流')
        if p['enable'] & ~before['capabilities']:raise ValueError('当前 FPGA 不支持选定 ISP 功能')
        if struct.unpack_from('<I',mem,0x14000)[0]&1 or before['status']&2:
            raise ValueError('采集未停止或上次参数提交尚未完成')
        if before['status']&1:
            if not reset_stopped_capture:
                raise ValueError('采集已禁用，但上一帧尚未排空；须由控制后台确认 DMA 全部停止后恢复')
            # The caller holds the manager lock and has verified that capture,
            # sender and external DMA workers are stopped. A camera stopped
            # mid-frame may never produce the EOF needed by uip_drained_axi.
            # Use the existing capture-only reset protocol (not ERNIC/PCIe),
            # before writing the ISP profile. Never reset a pending commit.
            struct.pack_into('<I',mem,0x14000,2)
            struct.unpack_from('<I',mem,0x14000)
            time.sleep(.001)
            struct.pack_into('<I',mem,0x14000,0)
            struct.unpack_from('<I',mem,0x14000)
            deadline=time.monotonic()+.5
            while read(mem,12)&1 and time.monotonic()<deadline:time.sleep(.001)
            before=status_from(mem)
            if before['status']&3:raise RuntimeError('采集软复位后 ISP 仍未空闲；未提交参数或启动 DMA')
        fields=[(0x10,p['enable']),(0x14,p['black']),(0x18,p['fixed_gains'][0]),(0x1c,p['fixed_gains'][1]),(0x20,p['fixed_gains'][2]),(0x24,p['threshold']),(0x28,p['phase'])]
        if p['enable']&112:
            if before['abi']!=0x10001:raise ValueError('当前位流没有坏点/细节/CCM 扩展；需要配套新位流')
            fields += [(0x2c,p['defect_threshold']),(0x38,p['fixed_sharp']),(0x3c,p['sharp_coring'])]
            fields += [(0x80+i*4,x&0xffffffff) for i,x in enumerate(p['fixed_ccm'])]
        fields += [(0x400+4*i,value) for i,value in enumerate(p['lut'])]
        for offset,value in fields:write(mem,offset,value)
        if any(read(mem,offset)!=value for offset,value in fields):raise RuntimeError('FPGA 参数回读不一致，未提交；采集保持禁用')
        write(mem,0x30,1)
        deadline=time.monotonic()+.5
        while read(mem,12)&6 and time.monotonic()<deadline:time.sleep(.001)
        current=status_from(mem)
        if current['status']&6 or current['generation']!=(before['generation']+1)&0xffffffff:raise RuntimeError('FPGA 参数提交未确认，不能开始采集')
        return current
