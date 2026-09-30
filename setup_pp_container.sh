#!/bin/bash
# Hybrid PP(v0.3.2) 라이브러리를 컨테이너 안 /opt/pp_libs 에 설치 (한 번만).
#   사용: ./setup_pp_container.sh <hybrid_learning_path_planning-0.3.2-py3-none-any.whl 경로>
#         (PP_CONTAINER=realgazebo 가 기본. 휠이 이미 컨테이너 안에 있으면 컨테이너 안 경로를 줘도 됨)
#
# - 호스트에는 아무것도 설치하지 않는다.
# - 컨테이너 시스템 파이썬도 건드리지 않는다: pip --target 으로 /opt/pp_libs 에만 넣고,
#   run_pp.sh 가 실행할 때만 PYTHONPATH 로 붙인다 (컨테이너의 numpy 2.5 는 PP 와 안 맞아서).
# - 필요: 컨테이너에 NVIDIA GPU 가 보일 것 (nvidia-smi), python3 3.10~3.12, pip.
set -e
WHL="$1"; C=${PP_CONTAINER:-realgazebo}; T=/opt/pp_libs
[ -n "$WHL" ] || { echo "사용: $0 <휠 파일 경로>"; exit 1; }
docker ps --format '{{.Names}}' | grep -qx "$C" || { docker start "$C" >/dev/null; sleep 2; }

if [ -f "$WHL" ]; then                       # 호스트 파일이면 컨테이너로 복사
  docker cp "$WHL" "$C:/tmp/$(basename "$WHL")"; WHL="/tmp/$(basename "$WHL")"
fi
docker exec -u root "$C" bash -c "mkdir -p $T && chown 1000 $T"
docker exec -u 1000 -e HOME=/tmp "$C" pip install -q --no-cache-dir --break-system-packages --target $T \
  "$WHL" "cupy-cuda12x[ctk]>=14.2,<15" "numpy>=1.26,<2.3" "scipy>=1.11,<2" opencv-python-headless

docker exec -u 1000 -e HOME=/tmp -e PYTHONPATH=$T -e CUDA_PATH=$T/nvidia/cuda_runtime \
  -e LD_LIBRARY_PATH=$T/nvidia/cuda_nvrtc/lib:$T/nvidia/cuda_runtime/lib -e CUPY_CACHE_DIR=/tmp/cupy_cache "$C" \
  python3 -c "import hybrid_learning_path_planning as h, cupy as cp; print('Hybrid PP', h.__version__, '| GPU', cp.cuda.runtime.getDeviceCount(), '대 | 확인', int(cp.arange(4).sum()) == 6)"
