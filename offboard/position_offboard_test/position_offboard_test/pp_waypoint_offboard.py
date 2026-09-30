#!/usr/bin/env python3
"""
PX4 Offboard Position Control - follow a Path-Planning (PP) waypoint list.

Mission: arm -> takeoff -> follow PP waypoints (NED, relative to the takeoff point)
         -> hover -> land.

Same defensive layers as offboard_control.py (the 1 m forward example):
  - arms only with a valid local position AND PX4 pre-flight checks passing
  - confirms ARMED + OFFBOARD, aborts on timeout
  - leaves the vehicle alone (stops commanding) on pilot take-over / failsafe
  - per-state timeouts -> abort (PX4 offboard-loss failsafe takes over)

Added for PP paths:
  - waypoints come from a CSV (x_north, y_east, z_down) in metres, relative to the
    takeoff point = PP origin (e.g. heightmap_inu/out/pp_goals_ned.csv).
    They are applied as offsets from the position captured at start, so the PX4
    local-origin (power-on point) does not have to be the takeoff point.
  - the setpoint is a "carrot" that moves along the path at `cruise_speed` and
    never runs more than `max_lead` metres ahead of the vehicle. PP waypoints can be
    20 m+ apart; sending them directly would make PX4 fly at its max speed.
  - pre-arm sanity check: every waypoint within `max_horizontal_m` of the start
    and `max_altitude_m` above it, otherwise refuse to arm.
  - yaw is held at the heading measured at start (no spin on takeoff).

IMPORTANT: secondary software safety layer only. A safety pilot with RC override,
PX4 failsafes (RC/datalink/battery/geofence) and a kill switch remain MANDATORY.
"""

import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus


ACTIVE_OFFBOARD = ('TAKEOFF', 'FOLLOW', 'HOVER')
HEARTBEAT_STATES = ('INIT', 'ARMING', 'TAKEOFF', 'FOLLOW', 'HOVER')


def load_waypoints(path):
    """CSV -> [(n, e, d), ...]. Header row optional; first 3 numeric columns are used."""
    pts = []
    with open(path, newline='') as f:
        for row in csv.reader(f):
            try:
                n, e, d = (float(v) for v in row[:3])
            except (ValueError, IndexError):
                continue          # header or blank line
            pts.append((n, e, d))
    return pts


def default_csv():
    """waypoint_csv 를 안 주면: $PP_WAYPOINT_CSV, 없으면 이 패키지 share/config/pp_goals_ned.csv.
    heightmap_inu/run_pp.sh 가 PP 실행 후 config/ 에 자동 복사한다
    (colcon build --symlink-install 이면 다시 빌드할 필요 없음)."""
    env = os.environ.get('PP_WAYPOINT_CSV', '')
    if env:
        return env
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('position_offboard_test'), 'config', 'pp_goals_ned.csv')
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    # 빌드 없이 소스에서 바로 실행한 경우: 패키지 소스의 config/
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'pp_goals_ned.csv')


class PPWaypointOffboard(Node):
    def __init__(self):
        super().__init__('pp_waypoint_offboard')

        self.declare_parameter('waypoint_csv', '')          # x_north,y_east,z_down [m]; '' = 기본 위치
        self.declare_parameter('takeoff_altitude', 3.0)     # m above start, positive = up
        self.declare_parameter('cruise_speed', 1.5)         # m/s along the path
        self.declare_parameter('max_lead', 2.0)             # m, carrot never further ahead
        self.declare_parameter('hover_seconds', 3.0)        # at the last waypoint
        self.declare_parameter('reach_tolerance', 0.3)      # m
        self.declare_parameter('state_timeout', 20.0)       # s (takeoff / hover)
        self.declare_parameter('arm_timeout', 5.0)
        self.declare_parameter('max_horizontal_m', 120.0)   # sanity limit per waypoint
        self.declare_parameter('max_altitude_m', 30.0)

        gp = lambda n: self.get_parameter(n).value
        self.csv_path = str(gp('waypoint_csv')) or default_csv()
        self.takeoff_alt = float(gp('takeoff_altitude'))
        self.speed = float(gp('cruise_speed'))
        self.max_lead = float(gp('max_lead'))
        self.hover_s = float(gp('hover_seconds'))
        self.tol = float(gp('reach_tolerance'))
        self.state_timeout = float(gp('state_timeout'))
        self.arm_timeout = float(gp('arm_timeout'))
        self.max_h = float(gp('max_horizontal_m'))
        self.max_alt = float(gp('max_altitude_m'))

        # ---- load + validate the mission before anything else ----
        self.wps_rel = []
        self.mission_ok = False
        if not self.csv_path or not os.path.isfile(self.csv_path):
            self.get_logger().error(f'waypoint_csv not found: "{self.csv_path}" - will NOT arm')
        else:
            self.wps_rel = load_waypoints(self.csv_path)
            bad = [i for i, (n, e, d) in enumerate(self.wps_rel)
                   if math.hypot(n, e) > self.max_h or -d > self.max_alt or -d < 0.5]
            if not self.wps_rel:
                self.get_logger().error('waypoint_csv has no waypoints - will NOT arm')
            elif bad:
                self.get_logger().error(
                    f'waypoints {bad} outside limits (horizontal <= {self.max_h} m, '
                    f'0.5 m <= altitude <= {self.max_alt} m) - will NOT arm')
            elif self.speed <= 0.0 or self.speed > 5.0:
                self.get_logger().error(f'cruise_speed {self.speed} out of (0, 5] m/s - will NOT arm')
            else:
                self.mission_ok = True

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.ocm_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._on_pos, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self._on_status, qos)

        self.pos = VehicleLocalPosition()
        self.status = VehicleStatus()
        self.have_pos = False
        self.have_status = False

        self.state = 'INIT'
        self.counter = 0
        self.entered_offboard = False
        self.start_set = False
        self.start = (0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.path = []            # absolute NED polyline: takeoff point, then waypoints
        self.cum = []             # cumulative length along the polyline
        self.s = 0.0              # carrot arc length
        self.target = (0.0, 0.0, 0.0)
        self.wp_reported = 0
        self.state_start = self._now()
        self.last_t = self._now()

        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz

        self.get_logger().info('PP waypoint offboard node started')
        self.get_logger().info(
            f'csv={self.csv_path} ({len(self.wps_rel)} wps) alt={self.takeoff_alt}m '
            f'speed={self.speed}m/s lead={self.max_lead}m tol={self.tol}m')
        for i, (n, e, d) in enumerate(self.wps_rel):
            self.get_logger().info(f'  WP{i + 1}: N {n:+7.2f}  E {e:+7.2f}  alt {-d:5.2f} m')

    # ---------- helpers ----------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _on_pos(self, msg):
        self.pos = msg
        self.have_pos = True

    def _on_status(self, msg):
        self.status = msg
        self.have_status = True

    def pos_valid(self):
        return self.have_pos and self.pos.xy_valid and self.pos.z_valid

    def is_armed(self):
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def in_offboard(self):
        return self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def set_state(self, s):
        self.state = s
        self.state_start = self._now()
        self.get_logger().info(f'State -> {s}')

    def elapsed(self):
        return self._now() - self.state_start

    def dist_to(self, p):
        return math.dist((self.pos.x, self.pos.y, self.pos.z), p)

    def reached(self, p):
        return self.have_pos and all(abs(a - b) < self.tol for a, b in
                                     zip((self.pos.x, self.pos.y, self.pos.z), p))

    def abort(self, reason):
        self.get_logger().error(f'ABORT: {reason}')
        self.set_state('ABORT')

    # ---------- command senders ----------
    def heartbeat(self):
        m = OffboardControlMode()
        m.position = True
        m.timestamp = self._ts()
        self.ocm_pub.publish(m)

    def setpoint(self, p):
        m = TrajectorySetpoint()
        m.position = [float(p[0]), float(p[1]), float(p[2])]
        m.yaw = float(self.yaw)
        m.timestamp = self._ts()
        self.sp_pub.publish(m)

    def send_cmd(self, command, **p):
        m = VehicleCommand()
        m.command = command
        for i in range(1, 8):
            setattr(m, f'param{i}', float(p.get(f'param{i}', 0.0)))
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = self._ts()
        self.cmd_pub.publish(m)

    def arm(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def engage_offboard(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # ---------- path (carrot) ----------
    def build_path(self):
        sx, sy, sz = self.start
        takeoff = (sx, sy, sz - self.takeoff_alt)
        self.path = [takeoff] + [(sx + n, sy + e, sz + d) for n, e, d in self.wps_rel]
        self.cum = [0.0]
        for a, b in zip(self.path, self.path[1:]):
            self.cum.append(self.cum[-1] + math.dist(a, b))
        self.get_logger().info(f'path length {self.cum[-1]:.1f} m, {len(self.path) - 1} legs')

    def point_at(self, s):
        s = min(max(s, 0.0), self.cum[-1])
        for i in range(len(self.cum) - 1):
            if s <= self.cum[i + 1] or i == len(self.cum) - 2:
                seg = self.cum[i + 1] - self.cum[i]
                t = 0.0 if seg <= 1e-9 else (s - self.cum[i]) / seg
                a, b = self.path[i], self.path[i + 1]
                return tuple(a[k] + t * (b[k] - a[k]) for k in range(3))
        return self.path[-1]

    # ---------- main loop ----------
    def loop(self):
        now = self._now()
        dt = min(max(now - self.last_t, 0.0), 0.5)
        self.last_t = now

        if self.state in HEARTBEAT_STATES:
            self.heartbeat()
        if not self.have_status:
            return

        if self.entered_offboard and self.state in ACTIVE_OFFBOARD:
            if not self.in_offboard():
                self.abort('left OFFBOARD (pilot take-over or failsafe)')
                return
            if self.status.failsafe:
                self.abort('PX4 failsafe active')
                return

        {'INIT': self._init, 'ARMING': self._arming, 'TAKEOFF': self._takeoff,
         'FOLLOW': lambda: self._follow(dt), 'HOVER': self._hover,
         'LANDING': self._landing}.get(self.state, lambda: None)()

    def _init(self):
        if not self.mission_ok:
            if self.counter == 0:
                self.get_logger().error('mission invalid - staying on the ground')
            self.counter += 1
            return
        if self.pos_valid() and not self.start_set:
            self.start = (self.pos.x, self.pos.y, self.pos.z)
            self.yaw = self.pos.heading if math.isfinite(self.pos.heading) else 0.0
            self.start_set = True
            self.build_path()
            self.get_logger().info(
                f'start: x={self.start[0]:.2f} y={self.start[1]:.2f} z={self.start[2]:.2f} '
                f'heading={math.degrees(self.yaw):.1f} deg')
        if not self.start_set:
            return

        self.setpoint(self.path[0])
        self.counter += 1
        if (self.counter >= 20 and self.pos_valid()
                and self.status.pre_flight_checks_pass and not self.status.failsafe):
            self.engage_offboard()
            self.arm()
            self.set_state('ARMING')
        elif self.counter == 60:
            self.get_logger().warn(
                f'waiting to arm: pos_valid={self.pos_valid()} '
                f'pre_flight_checks_pass={self.status.pre_flight_checks_pass}')

    def _arming(self):
        self.setpoint(self.path[0])
        if self.is_armed() and self.in_offboard():
            self.entered_offboard = True
            self.set_state('TAKEOFF')
        elif self.elapsed() > self.arm_timeout:
            self.abort('failed to ARM / enter OFFBOARD in time')

    def _takeoff(self):
        self.setpoint(self.path[0])
        if self.reached(self.path[0]):
            self.s = 0.0
            self.set_state('FOLLOW')
        elif self.elapsed() > self.state_timeout:
            self.abort('takeoff timeout')

    def _follow(self, dt):
        # advance the carrot, but never let it run more than max_lead ahead of the vehicle
        nxt = self.s + self.speed * dt
        if self.dist_to(self.point_at(nxt)) <= self.max_lead:
            self.s = nxt
        self.target = self.point_at(self.s)
        self.setpoint(self.target)

        # progress log when the carrot passes a PP waypoint
        while self.wp_reported < len(self.cum) - 1 and self.s >= self.cum[self.wp_reported + 1] - 1e-6:
            self.wp_reported += 1
            n, e, d = self.wps_rel[self.wp_reported - 1]
            self.get_logger().info(
                f'carrot at WP{self.wp_reported}/{len(self.wps_rel)} '
                f'(N {n:+.1f} E {e:+.1f} alt {-d:.1f})')

        if self.s >= self.cum[-1] and self.reached(self.path[-1]):
            self.set_state('HOVER')
        # timeout: 3x the nominal travel time + margin
        elif self.elapsed() > 3.0 * self.cum[-1] / self.speed + self.state_timeout:
            self.abort('follow timeout (vehicle not keeping up with the path)')

    def _hover(self):
        self.setpoint(self.path[-1])
        if self.elapsed() >= self.hover_s:
            self.land()
            self.set_state('LANDING')

    def _landing(self):
        if not self.is_armed():
            self.get_logger().info('landed and disarmed - mission complete')
            self.set_state('DONE')
        elif self.elapsed() > self.state_timeout * 2:
            self.get_logger().warn('still armed after landing timeout - leaving to PX4 / pilot')
            self.set_state('DONE')


def main(args=None):
    rclpy.init(args=args)
    node = PPWaypointOffboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
