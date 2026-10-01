#! /bin/bash
# ROS2 APP (인하우스 심 PX4-SITL-Runner 용): PP 경로를 PathFollowing 으로 비행.
#   pf_test.sh 와 같은 구조 (path_following_test 가 시동·이륙·착륙 FSM, PF 가 경로 추종)에서
#   - 경유점: ${WORKSPACE_DIR}/pp_mission/pf_wp.csv  (pf/run_pf_sitl.sh 가 PP 결과로 채움)
#   - guid_type 0 (위치 유도). MPPI(guid 2)는 90° 코너에서 교착하는 문제가 있어 쓰지 않음.
#   - vehicle_type 1 (쿼드, x500 PF 설정)
# pf/run_pf_sitl.sh 가 이 파일을 러너의 scripts/ros2/apps/ 에 복사해 ROS2_APP="pp_pf" 로 실행한다.
if [ -z "${WORKSPACE_DIR}" ]; then
    WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
fi
ROS2_WS=${WORKSPACE_DIR}/ros2_ws
LOGS=${WORKSPACE_DIR}/logs
WP=${WORKSPACE_DIR}/pp_mission/pf_wp.csv

for pkg in pathfollowing path_following_test px4_msgs; do
    [ -d "${ROS2_WS}/install/${pkg}" ] || { echo "[$(basename "$0")] ERROR: ${pkg} is not built."; exit 1; }
done
[ -f "$WP" ] || { echo "[$(basename "$0")] ERROR: 경유점 파일 없음: $WP"; exit 1; }
source ${ROS2_WS}/install/setup.bash
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 NUMEXPR_NUM_THREADS=2

PF_CFG=$(ros2 pkg prefix pathfollowing 2>/dev/null)/share/pathfollowing/config
# 러너의 x500 기본 설정을 쓰되, PF 버전과 형식이 맞을 때만 (0702 이후 PF 는 sim.yaml 에 safety: 섹션 필수).
# 형식이 다르면 덮어쓰지 않고 패키지에 설치된 설정을 그대로 쓴다.
X500=${WORKSPACE_DIR}/pf_config/x500
if [ -d $X500 ] && { ! grep -q "^ *safety:" "$PF_CFG/sim.yaml" || grep -q "^ *safety:" $X500/sim.yaml; }; then
    cp $X500/sim.yaml $X500/octo.yaml "$PF_CFG"/ && echo "[$(basename "$0")] PF config: x500 → $PF_CFG"
else
    echo "[$(basename "$0")] PF config: 설치된 설정 사용 ($X500 는 이 PF 버전과 형식이 달라 건너뜀)"
fi
echo "[$(basename "$0")] PP 경유점: $WP ($(($(wc -l < $WP) - 1)) 점)"

ros2 run pathfollowing node_pathfollowing --ros-args -p vehicle_type:=1 -p guid_type:=0 \
    -r /fmu/out/vehicle_status:=/fmu/out/vehicle_status_v1 2>&1 | tee ${LOGS}/node_pathfollowing.log &
ros2 run pathfollowing node_mppi --ros-args -p vehicle_type:=1 -p guid_type:=0 2>&1 | tee ${LOGS}/node_mppi.log &

ros2 run path_following_test path_following_test --ros-args \
    -p wp_csv_path:=${WP} \
    -r /vehicle1/fmu/in/offboard_control_mode:=/fmu/in/offboard_control_mode \
    -r /vehicle1/fmu/in/trajectory_setpoint:=/fmu/in/trajectory_setpoint \
    -r /vehicle1/fmu/in/vehicle_attitude_setpoint:=/fmu/in/vehicle_attitude_setpoint \
    -r /vehicle1/fmu/in/vehicle_command:=/fmu/in/vehicle_command \
    -r /vehicle1/fmu/out/vehicle_status_v1:=/fmu/out/vehicle_status_v1 \
    -r /vehicle1/fmu/out/vehicle_local_position:=/fmu/out/vehicle_local_position \
    -r /vehicle1/fmu/out/vehicle_attitude:=/fmu/out/vehicle_attitude \
    2>&1 | tee ${LOGS}/path_following_test.log &
