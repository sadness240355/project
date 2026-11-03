#!/usr/bin/env python3
import os
import os,mmap,struct,json,datetime
fd=os.open(os.environ.get('FPGA_RESOURCE','/sys/bus/pci/devices/0005:01:00.0/resource2'),os.O_RDONLY)
mm=mmap.mmap(fd,0x15000,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ)
values={hex(a):hex(struct.unpack_from('<I',mm,0x14000+a)[0]) for a in [0,4,0x20,0x24,0x38,0x40,0x28,0x2c,0x30,0x6c,0x70,0xc4,0xa00,0xa04,0xa0c,0xa10,0xa14,0xa1c,0xa20,0xa28]}
print(json.dumps({'scope':'READ_ONLY_CAMERA_PREFLIGHT','utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'camera_CSR_base':'0x14000','registers':values},indent=2))
mm.close();os.close(fd)
