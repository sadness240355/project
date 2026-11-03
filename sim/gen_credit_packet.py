#!/usr/bin/env python3
import argparse,struct,socket,zlib,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('out',type=Path);a=p.parse_args();a.out.mkdir(exist_ok=False,parents=True)
ip=bytearray(struct.pack('!BBHHHBBH4s4s',0x45,0,68,0,0x4000,64,17,0,socket.inet_aton('192.168.1.20'),socket.inet_aton('192.168.1.10')))
s=sum(struct.unpack('!10H',ip));s=(s&65535)+(s>>16);s=(s&65535)+(s>>16);ip[10:12]=struct.pack('!H',(~s)&65535)
eth=bytes.fromhex('000a35000100b8cef68393be0800');udp=struct.pack('!HHHH',0xc000,4791,48,0)
bth=bytes.fromhex('0a00ffff0000000380000abd');reth=struct.pack('!QII',0xffffba517000,0x100,8);payload=struct.pack('<Q',1)
f=eth+ip+udp+bth+reth+payload
crcinput=bytearray(f[14:]);
for i in [15,22,24,25,40,41,46]:crcinput[i-14]=255
crc=zlib.crc32(bytes([255])*8+crcinput);f+=struct.pack('<I',crc);assert len(f)==82
(a.out/'credit_packet.bin').write_bytes(f)
(a.out/'packet.hex').write_text('\n'.join(f'{b:02x}' for b in f)+'\n')
(a.out/'reference.json').write_text(json.dumps({'scope':'new synthetic 8-byte RC WRITE_ONLY credit; not captured live mlx5 packet','QP':3,'PSN':0xabd,'virtual_base':hex(0xffffba517000),'DMA_base':hex(0xeef01000),'rkey':0x100,'payload_credit':1,'ICRC':hex(crc)},indent=2)+'\n')
