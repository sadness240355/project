#!/usr/bin/env python3
"""Read classic Ethernet pcap; report actual credit fields without changing packets."""
import argparse,hashlib,json,struct,zlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('pcap',type=Path);p.add_argument('out',type=Path);a=p.parse_args()
b=a.pcap.read_bytes();assert len(b)>=24
fmt={b'\xd4\xc3\xb2\xa1':('<',1000000),b'\xa1\xb2\xc3\xd4':('>',1000000),b'\x4d\x3c\xb2\xa1':('<',1000000000),b'\xa1\xb2\x3c\x4d':('>',1000000000)}
assert b[:4] in fmt,'classic pcap required'
e,scale=fmt[b[:4]];vmaj,vmin,zone,sig,snap,link=struct.unpack(e+'HHiIII',b[4:24]);assert link==1,('Ethernet link type required',link)
pos=24;packets=[]
while pos<len(b):
 assert pos+16<=len(b)
 sec,frac,cap,wire=struct.unpack_from(e+'4I',b,pos);pos+=16;f=b[pos:pos+cap];pos+=cap;assert len(f)==cap
 if len(f)<14:continue
 eth_type=int.from_bytes(f[12:14],'big');ipoff=14
 vlans=[]
 while eth_type in [0x8100,0x88a8] and len(f)>=ipoff+4:
  vlans.append(int.from_bytes(f[ipoff:ipoff+2],'big'));eth_type=int.from_bytes(f[ipoff+2:ipoff+4],'big');ipoff+=4
 if eth_type!=0x800 or len(f)<ipoff+20:continue
 ihl=(f[ipoff]&15)*4
 if f[ipoff+9]!=17 or len(f)<ipoff+ihl+8:continue
 u=ipoff+ihl;sp,dp,ulen,ucs=struct.unpack_from('!4H',f,u)
 if sp!=4791 and dp!=4791:continue
 row={'timestamp_seconds':sec+frac/scale,'captured_bytes':cap,'wire_bytes':wire,'destination_mac':f[:6].hex(':'),'source_mac':f[6:12].hex(':'),'VLANs':vlans,'IPv4_header_bytes':ihl,'source_IP':'.'.join(map(str,f[ipoff+12:ipoff+16])),'destination_IP':'.'.join(map(str,f[ipoff+16:ipoff+20])),'UDP_source_port':sp,'UDP_destination_port':dp,'UDP_length':ulen,'UDP_checksum':ucs,'frame_hex':f.hex()}
 start=u+8;end=u+ulen
 if len(f)>=start+12:
  h=f[start:];op=h[0];row.update(opcode=hex(op),BTH_flags=h[1],PKEY=hex(int.from_bytes(h[2:4],'big')),QPN=int.from_bytes(h[5:8],'big'),PSN=int.from_bytes(h[9:12],'big'))
  if op==0x0a and len(h)>=28:row.update(remote_address=hex(int.from_bytes(h[12:20],'big')),rkey=hex(int.from_bytes(h[20:24],'big')),write_length=int.from_bytes(h[24:28],'big'),payload_hex=f[start+28:end-4].hex())
  if ihl==20 and len(f)>=end and end>=start+16:
   data=bytearray(f[ipoff:end-4]);
   for offset in [1,8,10,11,ihl+6,ihl+7,ihl+8+4]:data[offset]=255
   crc=zlib.crc32(b'\xff'*8+data);actual=int.from_bytes(f[end-4:end],'little');row.update(ICRC_computed=f'{crc:08x}',ICRC_wire=f'{actual:08x}',ICRC_matches=crc==actual)
  if ihl==20:
   words=struct.unpack('!10H',f[ipoff:ipoff+20]);s=sum(words);s=(s&65535)+(s>>16);s=(s&65535)+(s>>16);row['IPv4_checksum_valid']=s==65535
 packets.append(row)
assert pos==len(b)
result={'scope':'observed RDMA sniffer packets; comparisons are field checks, not proof of internal FPGA acceptance','pcap_sha256':hashlib.sha256(b).hexdigest(),'packet_count':len(packets),'packets':packets}
assert not a.out.exists();a.out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'packet_count':len(packets),'opcodes':[x.get('opcode') for x in packets]}))
