#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
id="${1:?new ID}";case "$id" in *[!a-zA-Z0-9_-]*|'')exit 2;;esac
out="$root/board/application_build/$id";[[ ! -e "$out" ]] || exit 2;mkdir -p "$out/source"
tar -xf "$root/source/application_inputs_20261030.tar" -C "$out/source"
app="$out/source/application/software"
cp "$root/source/edev_sgdma_driver.h" "$app/capture_reference/"
cp "$root/board/runtime_build/runtime002/librn_fresh.a" "$out/"
cp "$0" "$out/build_used.sh"
python3 - "$app/src" "$out" <<'PY'
from pathlib import Path
import sys,json,hashlib,difflib
src,out=map(Path,sys.argv[1:]);changes=[]
p=src/'ernic_video_sender.c';before=p.read_text();s=before
old='video_ack_recv(sockfd,pending_ids[done%VIDEO_WINDOW])'
assert s.count(old)==2
s=s.replace(old,'(ddr?video_credit_wait(credit_buf,(uint64_t)done+1,submitted):video_ack_recv(sockfd,pending_ids[done%VIDEO_WINDOW]))')
s=s.replace('''/* The native LRDMA engine is WRITE-only. Credits return on the already
       * negotiated control socket after the receiver copies and checks each
       * frame; pixel payload and ordered commit remain RDMA WRITEs. */''','''/* Direct DDR mode receives cumulative RDMA credits after the peer
       * checks and copies each frame. Legacy host-SHM mode keeps TCP ACKs. */''')
s=s.replace('// receiver credits return over the control socket after ownership release.','// receiver credits return through the registered RDMA mailbox after ownership release.')
old='        fprintf(stderr,"FAIL: DDR capture owner stopped\\n");validation_status=1;break;'
assert s.count(old)==1
s=s.replace(old,'        /* EOF is valid only after every published DDR source has completed\n         * both local DMA and RDMA reads. Pending peer credits drain below. */\n        if(atomic_load_explicit(&ddr->published,memory_order_acquire)!=(uint64_t)ddr_index ||\n           atomic_load_explicit(&ddr->released,memory_order_acquire)!=(uint64_t)ddr_index) {\n          fprintf(stderr,"FAIL: DDR owner stopped with unreleased source frames\\n");validation_status=1;\n        } else fprintf(stderr,"DDR_SOURCE_EOF all_published_sources_released=1 draining_peer_credits=1\\n");\n        break;')
old='  free(cidb_buffer);\n  free(data_buf);'
assert s.count(old)==1
s=s.replace(old,'  /* Peer END ACKs precede disabling both QPs; coherent credit memory\n   * remains alive through QP retirement and a quiet interval. */\n  if(ddr && !validation_status) {\n    write32_data(rdma_dev->axil_ctl,0x300000,read32_data(rdma_dev->axil_ctl,0x300000)&~1u);\n    if((read32_data(rdma_dev->axil_ctl,0x300000)&1u) ||\n       rn_rdma_prepare_qp(rdma_dev,qpid) || rn_rdma_prepare_qp(rdma_dev,BOX_CHANNEL_QPID)) {\n      fprintf(stderr,"FAIL: video QP retirement; safety hold PID %ld\\n",(long)getpid());\n      for(;;)pause();\n    }\n    usleep(1000000);rn_dma_rmb();\n    fprintf(stderr,"VIDEO_QPS_RETIRED_RDMA_DISABLED_BEFORE_POOL_RELEASE\\n");\n  }\n  free(cidb_buffer);\n  free(data_buf);')
p.write_text(s);changes.append((p.name,before,s))
p=src/'video_box_sender.h';before=p.read_text();s=before
old='video_ack_recv(c->fd,sent+1)';assert s.count(old)==1
s=s.replace(old,'video_credit_wait(c->credit,sent+1,sent+1)')
s=s.replace('TCP credit waits','RDMA credit waits').replace('transport=rdma_write_tcp_credit','transport=rdma_write_credit')
p.write_text(s);changes.append((p.name,before,s))
(out/'credit_channel_change.json').write_text(json.dumps({'scope':'isolated software candidate; frozen production input unchanged; video direct-DDR and independent box frame credits now match receiver RDMA writes; session END still uses TCP; checked source EOF drains peer credits; disable and retire QPs before pool release','files':[{'name':n,'before_sha256':hashlib.sha256(a.encode()).hexdigest(),'after_sha256':hashlib.sha256(b.encode()).hexdigest()} for n,a,b in changes]},indent=2)+'\n')
(out/'change.patch').write_text(''.join(''.join(difflib.unified_diff(a.splitlines(True),b.splitlines(True),fromfile=n+'.before',tofile=n)) for n,a,b in changes))
PY
cc -O2 -Wall -Wextra -pthread -I "$app/src" -I "$app/lib" "$app/src/ernic_video_sender.c" "$app/src/video_crc.c" "$out/librn_fresh.a" -ldl -lz -lm -lrt -o "$out/sender" 2> "$out/compile.log"
cc --version > "$out/compiler.txt";find "$out/source" -type f -exec sha256sum {} + > "$out/source.sha256"
sha256sum "$out/sender" > "$out/binary.sha256"
printf 'ISOLATED_RDMA_CREDIT_MATCH_CANDIDATE_BUILD_PASS_NOT_BOARD_RESULT\n' > "$out/result.txt"
