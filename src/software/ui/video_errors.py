"""Shared error presentation for local and remote workstations."""
import re
def readable_reason(reason,c):
 if 'timed out waiting for first DMA frame' in reason:
  fill=c.get('capture',{}).get('diag',{}).get('fill')
  match=re.search(r'reason=(0x[0-9a-fA-F]+|[0-9]+)',reason)
  code=int(match.group(1),0) if match else None
  where={6:'摄像头输入/像素 FIFO 溢出',10:'帧元数据 FIFO 溢出',
         11:'FPGA AXI 写响应错误',16:'DDR 环满',13:'Camera Link 同步错误'}
  return '主机未收到第一张完整帧，采集已停止'+('；原因：'+where[code] if code in where else '')+('；DDR 环占用 %s/32 槽'%fill if fill is not None else '')
 if 'timed out waiting for DMA pump frame' in reason:
  match=re.search(r'reason=(0x[0-9a-fA-F]+|[0-9]+)',reason)
  code=int(match.group(1),0) if match else None
  where={6:'摄像头输入/像素 FIFO 溢出',10:'帧元数据 FIFO 溢出',
         11:'FPGA AXI 写响应错误',16:'DDR 环满',13:'Camera Link 同步错误'}
  return '采集未收到后续完整帧'+('；原因：'+where[code] if code in where else '')+'；具体计数见运行日志'
 if 'RDMA frame notification timeout' in reason:return '等待 RDMA 视频帧通知超时'
 if 'inbound credit DMA uncertain' in reason:return 'RDMA 接收信用写回状态未确认，保留 DMA 内存'
 return reason

def fault_report(c):
 rows=[]
 operation=c.get("operation_error") or {}
 if operation.get("message"):
  rows.append((operation.get("stage","采集操作"),operation["message"],operation.get("log_path","")))
 for key,label in (("capture","Jetson 采集 / 主机首帧"),("sender","Jetson RDMA 发送"),("receiver","Mellanox RDMA 接收")):
  task=c.get(key,{})
  if task.get("stopping") and not task.get('safety_hold'):continue
  reason=task.get("first_error") or task.get("last_error")
  if reason:rows.append((label,reason,task.get("log_path","")))
 if not rows:return None
 # A generic operation hold must not hide the earlier concrete capture fault.
 if operation.get('message') and len(rows)>1 and ('安全保持' in operation['message'] or ('停止尚未完成' in operation['message'] and any(c.get(k,{}).get('safety_hold') for k in ('capture','sender','receiver')))):
  rows=rows[1:]+rows[:1]
 stage,reason,path=rows[0]
 detail=[]
 for label,msg,log in rows:
  detail.append(label+"："+msg+("\n日志："+log if log else ""))
 hold=any(c.get(k,{}).get("safety_hold") for k in ("capture","sender","receiver")) or '安全保持' in operation.get('message','')
 if hold:detail.append("DMA 安全保持：必须先复位 FPGA，再结束保持进程；当前内存仍保留。")
 return dict(message=stage+"："+readable_reason(reason,c),detail="\n".join(detail),safety_hold=hold)

def event_text(events):
 return "\n".join(e.get("time","")+"  "+("["+e["side"]+"] " if e.get("side") else "")+e.get("message","") for e in events)
