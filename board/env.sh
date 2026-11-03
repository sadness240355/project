# Optional machine configuration: export values before invoking board scripts.
export FPGA_RESOURCE=${FPGA_RESOURCE:-/sys/bus/pci/devices/0005:01:00.0/resource2}
export DMA_DEVICE=${DMA_DEVICE:-/dev/edev0}
export FPGA_IP=${FPGA_IP:-192.168.1.10}
export PEER_IP=${PEER_IP:-192.168.1.20}
export PEER_CONTROL_IP=${PEER_CONTROL_IP:-10.23.66.188}
export PEER_NETDEV=${PEER_NETDEV:-enp1s0f0np0}
export FPGA_NETDEV=${FPGA_NETDEV:-reconic1s0f0}
export RDMA_DEVICE=${RDMA_DEVICE:-mlx5_0}
export RDMA_GID_INDEX=${RDMA_GID_INDEX:-3}
export PEER_MAC=${PEER_MAC:-b8:ce:f6:83:93:be}
export CONTROL_URL=${CONTROL_URL:-http://10.23.68.0:8788}
