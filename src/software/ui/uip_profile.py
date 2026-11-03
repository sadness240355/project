"""UIP1 compatible profiles plus minor-ABI defect/detail/CCM extension."""
import math
import struct
ABI=0x10000
ABI_V2=0x10001
IDENTITY=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
DEFAULTS=dict(enable=0,black=0,gain_r=1.,gain_g=1.,gain_b=1.,threshold=16,phase=0,gamma=1.,contrast=1.,defect_threshold=32,sharp_strength=1.,sharp_coring=4,ccm=IDENTITY,saturation=1.,shadows=0.,highlights=0.)

def profile(values):
    if not isinstance(values,dict) or set(values)-set(DEFAULTS):raise ValueError('未知 FPGA 预处理参数')
    p={**DEFAULTS,**values}
    for key,maximum in [('enable',127),('black',255),('threshold',255),('phase',3),('defect_threshold',255),('sharp_coring',63)]:
        if type(p[key]) is not int or not 0<=p[key]<=maximum:raise ValueError(key+' 参数超范围')
    for key,low,high in [('gain_r',0,4095/256),('gain_g',0,4095/256),('gain_b',0,4095/256),('gamma',.2,3),('contrast',.25,4),('sharp_strength',0,2),('saturation',0,2),('shadows',-1,1),('highlights',-1,1)]:
        if isinstance(p[key],bool):raise ValueError(key+' 需要数值')
        p[key]=float(p[key])
        if not math.isfinite(p[key]) or not low<=p[key]<=high:raise ValueError(key+' 参数超范围')
    if not isinstance(p['ccm'],(list,tuple)) or len(p['ccm'])!=9:raise ValueError('CCM 需要行优先的 9 个系数')
    matrix=[]
    for x in p['ccm']:
        if isinstance(x,bool):raise ValueError('CCM 系数需要有限数值')
        x=float(x)
        if not math.isfinite(x) or not -8<=x<=32767/4096:raise ValueError('CCM 系数超范围')
        matrix.append(x)
    p['ccm']=matrix
    weights=[77/256,150/256,29/256];sat=p['saturation']
    combined=[sum(((sat if row==k else 0)+(1-sat)*weights[k])*matrix[k*3+col] for k in range(3)) for row in range(3) for col in range(3)]
    p['fixed_ccm']=[math.floor(x*4096+.5) for x in combined]
    if any(not -32768<=x<=32767 for x in p['fixed_ccm']):raise ValueError('CCM 与饱和度合并后超出 Q3.12 范围')
    p['fixed_gains']=[int(p[k]*256+.5) for k in ('gain_r','gain_g','gain_b')]
    p['fixed_sharp']=int(p['sharp_strength']*16+.5)
    lut=[]
    for x in range(256):
        # Monotone bounded shadow/highlight anchors; this is global tone, not Sony DRO.
        value=x+24*p['shadows']*max(0,1-abs(x-64)/64)+24*p['highlights']*max(0,1-abs(x-192)/(64 if x<=192 else 63))
        value=max(0,min(255,(value-128)*p['contrast']+128))/255
        lut.append(min(255,int(value**(1/p['gamma'])*255+.5)))
    p['lut']=lut
    return p

def encode(values):
    p=profile(values);extended=bool(p['enable']&112)
    data=struct.pack('<12I256B',1,ABI_V2 if extended else ABI,0,0,0,p['enable'],p['black'],*p['fixed_gains'],p['threshold'],p['phase'],*p['lut'])
    if extended:data+=struct.pack('<3I9i',p['defect_threshold'],p['fixed_sharp'],p['sharp_coring'],*p['fixed_ccm'])
    return data.hex()
