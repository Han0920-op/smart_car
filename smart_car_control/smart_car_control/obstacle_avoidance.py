"""Publish one velocity command stream from LaserScan and a steady-clock watchdog."""

import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from .decision import State, choose_motion, scan_clearance


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")
        descriptor = ParameterDescriptor(read_only=True)
        defaults = {
            "cruise_speed": 0.22,
            "turn_speed": 0.65,
            "front_stop_distance": 0.60,
            "front_clear_distance": 0.82,
            "side_min_distance": 0.30,
            "scan_timeout": 0.50,
        }
        config = {}
        for name, value in defaults.items():
            config[name] = self.declare_parameter(name, value, descriptor).value
        self.cruise_speed = config["cruise_speed"]
        self.turn_speed = config["turn_speed"]
        self.enter_distance = config["front_stop_distance"]
        self.exit_distance = config["front_clear_distance"]
        self.side_minimum = config["side_min_distance"]
        self.scan_timeout = config["scan_timeout"]
        if not (
            all(map(math.isfinite, config.values()))
            and 0.0 < self.cruise_speed <= 0.35
            and 0.0 < self.turn_speed <= 1.0
            and 0.0 < self.side_minimum <= self.enter_distance < self.exit_distance < 8.0
            and 0.20 <= self.scan_timeout <= 2.0
        ):
            raise ValueError("Invalid speed, distance hysteresis, or scan timeout parameters")

        self.clearance = None
        self.last_scan_at = None
        self.last_stamp_ns = None
        self.turn_direction = 0
        self.last_report = None
        self.last_log_at = -math.inf

        scan_qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 1)
        self.scan = self.create_subscription(LaserScan, "/scan", self.on_scan, scan_qos)
        # A paused Gazebo clock must not suspend the watchdog.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.on_timer, clock=self.steady_clock)

    def on_scan(self, message):
        received_at = time.monotonic()
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            if (
                stamp_ns == self.last_stamp_ns
                or received_at - self.last_scan_at < self.scan_timeout
            ):
                return
            # An older stamp after timeout permits recovery after a simulation reset.
        self.last_stamp_ns = stamp_ns
        self.last_scan_at = received_at
        self.clearance = None
        if message.header.frame_id == "laser_link":
            self.clearance = scan_clearance(
                message.ranges, message.angle_min, message.angle_max, message.angle_increment,
                message.range_min, message.range_max,
            )

    def on_timer(self):
        now = time.monotonic()
        distance = self.clearance
        if self.last_scan_at is None:
            state, turn, reason = State.STOP, 0, "waiting_for_scan"
        elif now - self.last_scan_at >= self.scan_timeout:
            state, turn, reason = State.STOP, 0, "scan_timeout"
            distance = None
        else:
            state, turn, reason = choose_motion(
                distance, self.turn_direction,
                self.enter_distance, self.exit_distance, self.side_minimum,
            )
        self.turn_direction = turn

        command = Twist()
        if state is State.CRUISE:
            command.linear.x = self.cruise_speed
        elif state is State.AVOID:
            command.angular.z = turn * self.turn_speed
        self.cmd_vel.publish(command)

        report = (state, reason)
        if report != self.last_report or now - self.last_log_at >= 1.0:
            values = "n/a" if distance is None else (
                f"{distance.front:.2f}/{distance.left:.2f}/{distance.right:.2f} m"
            )
            self.get_logger().info(
                f"state={state.value} reason={reason} front/left/right={values} "
                f"linear.x={command.linear.x:.2f} m/s angular.z={command.angular.z:.2f} rad/s"
            )
            self.last_report = report
            self.last_log_at = now


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous_term = signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = None
    try:
        node = ObstacleAvoidance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.cmd_vel.publish(Twist())
            node.cmd_vel.wait_for_all_acked(Duration(seconds=0.2))
            node.destroy_node()
        signal.signal(signal.SIGTERM, previous_term)
        rclpy.try_shutdown()
