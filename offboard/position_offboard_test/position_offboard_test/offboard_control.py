#!/usr/bin/env python3
"""
PX4 Offboard Position Control - defensive / real-flight oriented example.

Mission: arm -> takeoff -> move forward -> hover -> land.

Safety layers added on top of the basic example:
  - subscribes to vehicle_status_v1 (PX4 v1.16) and actually USES it
  - waits for a valid local position estimate + pre-flight checks before arming
  - confirms ARMED and OFFBOARD engaged (aborts if not within a timeout)
  - detects pilot take-over / failsafe (nav_state leaves OFFBOARD) and STOPS
    commanding, so it never fights the safety pilot and never auto-resumes
  - per-state timeouts -> abort (hands control back to PX4 failsafe)
  - tunable altitude / distance / hover / tolerance via ROS parameters

IMPORTANT: this is only a *secondary* software safety layer. A safety pilot with
RC override, PX4 failsafes (RC/datalink/battery/geofence), a geofence and a kill
switch remain MANDATORY for any real flight.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus


# states where we are actively flying in offboard and must hold OFFBOARD mode
ACTIVE_OFFBOARD = ('TAKEOFF', 'FORWARD', 'HOVER')
# states where we keep streaming the offboard heartbeat
HEARTBEAT_STATES = ('INIT', 'ARMING', 'TAKEOFF', 'FORWARD', 'HOVER')


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        # ---- tunable parameters (override at runtime / via launch) ----
        self.declare_parameter('takeoff_altitude', 2.0)   # m, positive = up
        self.declare_parameter('forward_distance', 1.0)   # m, +North
        self.declare_parameter('hover_seconds', 5.0)
        self.declare_parameter('reach_tolerance', 0.3)    # m
        self.declare_parameter('state_timeout', 20.0)     # s per move state
        self.declare_parameter('arm_timeout', 5.0)        # s to confirm ARMED+OFFBOARD

        self.takeoff_alt = float(self.get_parameter('takeoff_altitude').value)
        self.forward_dist = float(self.get_parameter('forward_distance').value)
        self.hover_s = float(self.get_parameter('hover_seconds').value)
        self.tol = float(self.get_parameter('reach_tolerance').value)
        self.state_timeout = float(self.get_parameter('state_timeout').value)
        self.arm_timeout = float(self.get_parameter('arm_timeout').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # publishers
        self.ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # subscribers
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._on_pos, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._on_status, qos)

        self.pos = VehicleLocalPosition()
        self.status = VehicleStatus()
        self.have_pos = False
        self.have_status = False

        self.state = 'INIT'
        self.counter = 0
        self.entered_offboard = False
        self.start_set = False
        self.start_x = self.start_y = self.start_z = 0.0
        self.target_x = self.target_y = self.target_z = 0.0
        self.state_start = self._now()

        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz

        self.get_logger().info('Defensive offboard node started')
        self.get_logger().info(
            f'params: alt={self.takeoff_alt}m dist={self.forward_dist}m '
            f'hover={self.hover_s}s tol={self.tol}m')

    # ---------- callbacks / helpers ----------
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

    def reached(self, x, y, z):
        if not self.have_pos:
            return False
        return (abs(self.pos.x - x) < self.tol
                and abs(self.pos.y - y) < self.tol
                and abs(self.pos.z - z) < self.tol)

    def abort(self, reason):
        # Stop commanding. With the heartbeat gone, PX4's offboard-loss failsafe
        # takes over; if a pilot took over, we simply stop fighting.
        self.get_logger().error(f'ABORT: {reason}')
        self.set_state('ABORT')

    # ---------- command senders ----------
    def heartbeat(self):
        m = OffboardControlMode()
        m.position = True
        m.timestamp = self._ts()
        self.ocm_pub.publish(m)

    def setpoint(self, x, y, z, yaw=0.0):
        m = TrajectorySetpoint()
        m.position = [float(x), float(y), float(z)]
        m.yaw = float(yaw)
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

    # ---------- main loop ----------
    def loop(self):
        # stream the offboard heartbeat only while we intend to be in offboard
        if self.state in HEARTBEAT_STATES:
            self.heartbeat()

        # all safety decisions need vehicle status
        if not self.have_status:
            return

        # pilot take-over / failsafe detection while actively flying offboard
        if self.entered_offboard and self.state in ACTIVE_OFFBOARD:
            if not self.in_offboard():
                self.abort('left OFFBOARD (pilot take-over or failsafe)')
                return
            if self.status.failsafe:
                self.abort('PX4 failsafe active')
                return

        if self.state == 'INIT':
            self._init()
        elif self.state == 'ARMING':
            self._arming()
        elif self.state == 'TAKEOFF':
            self._takeoff()
        elif self.state == 'FORWARD':
            self._forward()
        elif self.state == 'HOVER':
            self._hover()
        elif self.state == 'LANDING':
            self._landing()
        # ABORT / DONE: do nothing (no setpoints, no heartbeat)

    def _init(self):
        # capture start position once we have a valid estimate
        if self.pos_valid() and not self.start_set:
            self.start_x, self.start_y, self.start_z = self.pos.x, self.pos.y, self.pos.z
            self.start_set = True
            self.get_logger().info(
                f'start: x={self.start_x:.2f} y={self.start_y:.2f} z={self.start_z:.2f}')

        # do NOT arm without a valid position estimate
        if not self.start_set:
            return

        # stream the takeoff setpoint before switching to offboard (PX4 requirement).
        # Altitude is RELATIVE to the captured start/ground (start_z - alt), so it is
        # robust to a non-zero local-position z reference.
        self.setpoint(self.start_x, self.start_y, self.start_z - self.takeoff_alt)
        self.counter += 1

        # arm only when position is valid AND PX4 pre-arm checks pass
        if (self.counter >= 20 and self.pos_valid()
                and self.status.pre_flight_checks_pass and not self.status.failsafe):
            self.engage_offboard()
            self.arm()
            self.set_state('ARMING')
        elif self.counter == 60:
            self.get_logger().warn(
                'waiting to arm: '
                f'pos_valid={self.pos_valid()} '
                f'pre_flight_checks_pass={self.status.pre_flight_checks_pass}')

    def _arming(self):
        # keep streaming setpoint; confirm both ARMED and OFFBOARD
        self.setpoint(self.start_x, self.start_y, self.start_z - self.takeoff_alt)
        if self.is_armed() and self.in_offboard():
            self.entered_offboard = True
            self.target_x, self.target_y = self.start_x, self.start_y
            self.target_z = self.start_z - self.takeoff_alt
            self.set_state('TAKEOFF')
            self.get_logger().info(f'climbing to z={self.target_z:.2f} (start_z {self.start_z:.2f} - {self.takeoff_alt}m)')
        elif self.elapsed() > self.arm_timeout:
            self.abort('failed to ARM / enter OFFBOARD in time')

    def _takeoff(self):
        self.setpoint(self.target_x, self.target_y, self.target_z)
        if self.reached(self.target_x, self.target_y, self.target_z):
            self.target_x = self.start_x + self.forward_dist
            self.set_state('FORWARD')
        elif self.elapsed() > self.state_timeout:
            self.abort('takeoff timeout')

    def _forward(self):
        self.setpoint(self.target_x, self.target_y, self.target_z)
        if self.reached(self.target_x, self.target_y, self.target_z):
            self.set_state('HOVER')
        elif self.elapsed() > self.state_timeout:
            self.abort('forward timeout')

    def _hover(self):
        self.setpoint(self.target_x, self.target_y, self.target_z)
        if self.elapsed() >= self.hover_s:
            self.land()
            self.set_state('LANDING')

    def _landing(self):
        # PX4 AUTO_LAND now owns the descent; we stop sending setpoints/heartbeat.
        if not self.is_armed():
            self.get_logger().info('landed and disarmed - mission complete')
            self.set_state('DONE')
        elif self.elapsed() > self.state_timeout * 2:
            self.get_logger().warn('still armed after landing timeout - leaving to PX4 / pilot')
            self.set_state('DONE')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
