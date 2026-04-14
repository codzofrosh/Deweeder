#!/usr/bin/env python3
"""
coverage_planner_node.py
────────────────────────
Generates a boustrophedon (lawnmower) coverage path for the weedbot field
and publishes the desired heading to the next waypoint so behavior_node
can steer toward it.

The field is divided into parallel strips along the X axis. The robot
traverses them alternately east then west (boustrophedon pattern).

Topics
──────
  Subscribes:  /odom              (nav_msgs/Odometry)
  Publishes:   /coverage/heading  (std_msgs/Float64)  — desired global yaw (radians)
               /coverage/status   (std_msgs/String)   — human-readable progress

Parameters
──────────
  field_x_min        (float, default 0.0)   — field left edge (metres)
  field_x_max        (float, default 4.0)   — field right edge
  field_y_min        (float, default -0.80) — field bottom edge
  field_y_max        (float, default  0.80) — field top edge
  strip_spacing      (float, default 0.30)  — distance between strip centres
  waypoint_threshold (float, default 0.20)  — advance when this close to wp (m)
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String


def _normalize_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _build_boustrophedon(
    x_min: float, x_max: float,
    y_min: float, y_max: float,
    spacing: float,
) -> list:
    """
    Build (x, y) waypoints in a boustrophedon pattern.

    Strips are parallel to the X axis, centred at y = y_min + spacing/2,
    y_min + 3*spacing/2, etc.  Odd strips go east (x_max), even go west (x_min).
    The robot starts near (0, 0) so the first waypoint moves it into the field.
    """
    waypoints = []
    y = y_min + spacing / 2.0
    east = True
    while y <= y_max + 1e-6:
        if east:
            waypoints.append((x_max, y))
        else:
            waypoints.append((x_min, y))
        y += spacing
        east = not east
    return waypoints


class CoveragePlannerNode(Node):
    def __init__(self):
        super().__init__('coverage_planner_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('field_x_min', 0.0)
        self.declare_parameter('field_x_max', 4.0)
        self.declare_parameter('field_y_min', -0.80)
        self.declare_parameter('field_y_max', 0.80)
        self.declare_parameter('strip_spacing', 0.30)
        self.declare_parameter('waypoint_threshold', 0.20)

        x_min = float(self.get_parameter('field_x_min').value)
        x_max = float(self.get_parameter('field_x_max').value)
        y_min = float(self.get_parameter('field_y_min').value)
        y_max = float(self.get_parameter('field_y_max').value)
        spacing = float(self.get_parameter('strip_spacing').value)
        self._threshold = float(self.get_parameter('waypoint_threshold').value)

        # ── Build waypoint list ───────────────────────────────────────────
        self._waypoints = _build_boustrophedon(x_min, x_max, y_min, y_max, spacing)
        self._wp_idx = 0
        self._complete = False

        # ── Current robot position ────────────────────────────────────────
        self._x = 0.0
        self._y = 0.0

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self._heading_pub = self.create_publisher(Float64, '/coverage/heading', 10)
        self._status_pub = self.create_publisher(String, '/coverage/status', 10)

        # ── Publish at 10 Hz ─────────────────────────────────────────────
        self.create_timer(0.1, self._publish)

        self.get_logger().info(
            f'CoveragePlanner ready — {len(self._waypoints)} waypoints, '
            f'field [{x_min:.1f},{x_max:.1f}] × [{y_min:.1f},{y_max:.1f}] m, '
            f'strip spacing {spacing:.2f} m'
        )
        for i, (wx, wy) in enumerate(self._waypoints):
            self.get_logger().info(f'  wp[{i}] = ({wx:.2f}, {wy:.2f})')

    # ────────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

        if self._complete:
            return

        # Advance to next waypoint when within threshold
        wx, wy = self._waypoints[self._wp_idx]
        dist = math.hypot(wx - self._x, wy - self._y)
        if dist < self._threshold:
            self._wp_idx += 1
            if self._wp_idx >= len(self._waypoints):
                self._complete = True
                self.get_logger().info('Coverage complete — all waypoints visited.')
            else:
                nwx, nwy = self._waypoints[self._wp_idx]
                self.get_logger().info(
                    f'wp[{self._wp_idx - 1}] reached → '
                    f'wp[{self._wp_idx}] = ({nwx:.2f}, {nwy:.2f})'
                )

    # ────────────────────────────────────────────────────────────────────
    # Publish
    # ────────────────────────────────────────────────────────────────────

    def _publish(self):
        status_msg = String()

        if self._complete:
            status_msg.data = 'COVERAGE COMPLETE'
            self._status_pub.publish(status_msg)
            return

        wx, wy = self._waypoints[self._wp_idx]
        dx = wx - self._x
        dy = wy - self._y
        heading = math.atan2(dy, dx)
        dist = math.hypot(dx, dy)

        heading_msg = Float64()
        heading_msg.data = heading
        self._heading_pub.publish(heading_msg)

        status_msg.data = (
            f'wp[{self._wp_idx}/{len(self._waypoints) - 1}] '
            f'target=({wx:.2f},{wy:.2f}) '
            f'pos=({self._x:.2f},{self._y:.2f}) '
            f'dist={dist:.2f}m '
            f'heading={math.degrees(heading):.1f}deg'
        )
        self._status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
