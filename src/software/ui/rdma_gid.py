#!/usr/bin/env python3
"""Select this interface's IPv4 RoCE v2 GID; indices can change on link reload."""
import argparse,ipaddress,time
from pathlib import Path
def find_gid(device,port,netdev,ipv4,root=Path("/sys/class/infiniband")):
 expected=ipaddress.IPv4Address(ipv4)
 base=Path(root)/device/"ports"/str(port)
 try:entries=sorted((base/"gids").iterdir(),key=lambda p:int(p.name))
 except OSError:return None
 for p in entries:
  try:
   gid=ipaddress.IPv6Address(p.read_text().strip())
   kind=(base/"gid_attrs/types"/p.name).read_text().strip()
   dev=(base/"gid_attrs/ndevs"/p.name).read_text().strip()
  except (OSError,ValueError):continue
  if gid.ipv4_mapped==expected and kind=="RoCE v2" and dev==netdev:return int(p.name)
 return None
def wait_gid(device,port,netdev,ipv4,timeout=10,root=Path("/sys/class/infiniband")):
 deadline=time.monotonic()+timeout;previous=None
 while True:
  current=find_gid(device,port,netdev,ipv4,root)
  if current is not None and current==previous:return current
  previous=current
  if time.monotonic()>=deadline:
   raise RuntimeError("IPv4 RoCE v2 GID not ready for "+netdev+" / "+ipv4)
  time.sleep(.2)
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument("--device",default="mlx5_0");p.add_argument("--port",type=int,default=1)
 p.add_argument("--netdev",default="enp1s0f0np0");p.add_argument("--ipv4",default="192.168.1.20")
 p.add_argument("--timeout",type=float,default=10)
 a=p.parse_args()
 if "/" in a.device or not a.device or not 1<=a.port<=255 or not 0<=a.timeout<=30:p.error("invalid device, port or timeout")
 try:print(wait_gid(a.device,a.port,a.netdev,a.ipv4,a.timeout))
 except (OSError,ValueError,RuntimeError) as e:raise SystemExit("Error: "+str(e))
if __name__=="__main__":main()
