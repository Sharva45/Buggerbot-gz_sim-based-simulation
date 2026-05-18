#!/usr/bin/env python3
"""
ekf_odometry_corrector.py
─────────────────────────
Extended Kalman Filter (EKF) node for ROS 2 Humble.

Fuses:
  • Wheel-encoder odometry  → /bumperbot_controller/odom  (nav_msgs/Odometry)
  • LiDAR scan matching     → /scan                       (sensor_msgs/LaserScan)
    (point-to-point ICP on consecutive 2-D scans gives a Δx, Δy, Δθ update)

Publishes:
  • /odometry/filtered      (nav_msgs/Odometry)   — EKF-corrected pose
  • /ekf/covariance_marker  (visualization_msgs/Marker) — 2-σ ellipse for RViz

State vector  x = [x, y, θ]ᵀ
Motion model  f(x, u)  — same differential-drive odometry increments as the
              original script but propagated through the Jacobian Fx.
Measurement   h(x)    — identity (LiDAR ICP gives a pose observation directly).

Noise parameters are ROS parameters so you can tune them without recompiling.

Usage
─────
  ros2 run <your_pkg> ekf_odometry_corrector

Parameters (all double unless noted)
─────────────────────────────────────
  alpha1..4        motion-noise coefficients   (default 0.1 each)
  sigma_lidar_xy   LiDAR translation std-dev   (default 0.05 m)
  sigma_lidar_th   LiDAR rotation std-dev      (default 0.02 rad)
  min_scan_points  min points needed for ICP   (default 30, int)
"""

from math import atan2, sin, cos, sqrt, fabs, pi
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from tf_transformations import (quaternion_from_euler, euler_from_quaternion,
                                 quaternion_matrix)
import tf2_ros


# ─── helpers ─────────────────────────────────────────────────────────────────

def normalize(z: float) -> float:
    return atan2(sin(z), cos(z))


def angle_diff(a: float, b: float) -> float:
    a, b = normalize(a), normalize(b)
    d1 = a - b
    d2 = 2.0 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    return d1 if fabs(d1) < fabs(d2) else d2


def rotation_matrix_2d(theta: float) -> np.ndarray:
    c, s = cos(theta), sin(theta)
    return np.array([[c, -s], [s, c]])


def scan_to_cartesian(scan: LaserScan) -> np.ndarray:
    """Convert a LaserScan to an (N, 2) array of valid (x, y) points."""
    angles = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
    ranges = np.array(scan.ranges, dtype=np.float64)
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack([r * np.cos(a), r * np.sin(a)])


def icp_2d(src: np.ndarray, dst: np.ndarray, max_iter: int = 20,
           tolerance: float = 1e-4) -> tuple[float, float, float]:
    """
    Minimal point-to-point ICP for 2-D scans.
    Returns (dx, dy, dtheta) — the rigid transform from src to dst frame.
    Uses nearest-neighbour matching via a simple KD-style approach (brute-force
    for small scan sizes; replace with scipy.spatial.cKDTree for large scans).
    """
    from scipy.spatial import cKDTree  # optional fast NN; available in ROS 2

    T = np.eye(3)          # accumulated transform
    src_h = src.copy()     # working copy

    for _ in range(max_iter):
        tree = cKDTree(dst)
        distances, indices = tree.query(src_h, k=1)
        matched_dst = dst[indices]

        # Compute centroid-based SVD solution
        mu_s = src_h.mean(axis=0)
        mu_d = matched_dst.mean(axis=0)
        H = (src_h - mu_s).T @ (matched_dst - mu_d)
        U, _, Vt = np.linalg.svd(H)
        R = (Vt.T @ U.T)
        # Ensure proper rotation (det = +1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = mu_d - R @ mu_s

        # Apply incremental transform
        step = np.eye(3)
        step[:2, :2] = R
        step[:2, 2] = t
        T = step @ T
        src_h = (R @ src_h.T).T + t

        if np.linalg.norm(t) < tolerance:
            break

    dx = T[0, 2]
    dy = T[1, 2]
    dtheta = atan2(T[1, 0], T[0, 0])
    return dx, dy, dtheta


# ─── EKF node ─────────────────────────────────────────────────────────────────

class EKFOdometryCorrector(Node):

    def __init__(self):
        super().__init__('ekf_odometry_corrector')

        # ── parameters ──────────────────────────────────────────────────────
        for name, default in [('alpha1', 0.1), ('alpha2', 0.1),
                               ('alpha3', 0.1), ('alpha4', 0.1),
                               ('sigma_lidar_xy', 0.05),
                               ('sigma_lidar_th', 0.02)]:
            self.declare_parameter(name, default)
        self.declare_parameter('min_scan_points', 30)

        def gp(n): return self.get_parameter(n).get_parameter_value().double_value
        def gi(n): return self.get_parameter(n).get_parameter_value().integer_value

        self.alpha = [gp(f'alpha{i}') for i in range(1, 5)]
        self.sigma_xy = gp('sigma_lidar_xy')
        self.sigma_th = gp('sigma_lidar_th')
        self.min_pts  = gi('min_scan_points')

        # ── EKF state ────────────────────────────────────────────────────────
        # x = [x, y, theta]
        self.mu = np.zeros(3)                       # mean
        self.P  = np.diag([1e-6, 1e-6, 1e-6])      # covariance (start tight)

        # Measurement noise covariance R
        self.R_lidar = np.diag([self.sigma_xy**2,
                                self.sigma_xy**2,
                                self.sigma_th**2])

        # ── odometry bookkeeping ─────────────────────────────────────────────
        self.last_odom_x     = 0.0
        self.last_odom_y     = 0.0
        self.last_odom_theta = 0.0
        self.is_first_odom   = True

        # ── scan bookkeeping ─────────────────────────────────────────────────
        self.last_scan_pts: np.ndarray | None = None
        self.frame_id = 'odom'

        # ── pub / sub ────────────────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, 'bumperbot_controller/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        self.filtered_pub = self.create_publisher(
            Odometry, 'odometry/filtered', 10)
        self.marker_pub = self.create_publisher(
            Marker, 'ekf/covariance_marker', 10)

        self.get_logger().info('EKF Odometry Corrector started.')

    # ── odometry callback: EKF PREDICT step ──────────────────────────────────

    def odom_callback(self, msg: Odometry):
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(q)

        if self.is_first_odom:
            self.mu[0] = msg.pose.pose.position.x
            self.mu[1] = msg.pose.pose.position.y
            self.mu[2] = yaw
            self.frame_id = msg.header.frame_id
            self.last_odom_x     = self.mu[0]
            self.last_odom_y     = self.mu[1]
            self.last_odom_theta = self.mu[2]
            self.is_first_odom   = False
            return

        # Odometry increments
        dx   = msg.pose.pose.position.x - self.last_odom_x
        dy   = msg.pose.pose.position.y - self.last_odom_y
        dth  = angle_diff(yaw, self.last_odom_theta)

        trans = sqrt(dx**2 + dy**2)
        if trans < 0.001:
            delta_rot1 = 0.0
        else:
            delta_rot1 = angle_diff(atan2(dy, dx), self.last_odom_theta)
        delta_rot2 = angle_diff(dth, delta_rot1)

        # ── Predict ─────────────────────────────────────────────────────────
        th = self.mu[2]

        # Nonlinear prediction
        self.mu[0] += trans * cos(th + delta_rot1)
        self.mu[1] += trans * sin(th + delta_rot1)
        self.mu[2]  = normalize(self.mu[2] + delta_rot1 + delta_rot2)

        # Jacobian of motion model w.r.t. state (3×3)
        Fx = np.array([
            [1, 0, -trans * sin(th + delta_rot1)],
            [0, 1,  trans * cos(th + delta_rot1)],
            [0, 0,  1]
        ])

        # Process noise Q from alpha parameters (same structure as original)
        a1, a2, a3, a4 = self.alpha
        rot1_var  = a1 * delta_rot1**2 + a2 * trans**2
        trans_var = a3 * trans**2 + a4 * (delta_rot1**2 + delta_rot2**2)
        rot2_var  = a1 * delta_rot2**2 + a2 * trans**2

        # Jacobian of motion w.r.t. noise inputs (3×3)
        V = np.array([
            [-trans * sin(th + delta_rot1), cos(th + delta_rot1), 0],
            [ trans * cos(th + delta_rot1), sin(th + delta_rot1), 0],
            [1,                             0,                    1]
        ])
        M = np.diag([rot1_var, trans_var, rot2_var])

        self.P = Fx @ self.P @ Fx.T + V @ M @ V.T

        # Update bookkeeping
        self.last_odom_x     = msg.pose.pose.position.x
        self.last_odom_y     = msg.pose.pose.position.y
        self.last_odom_theta = yaw

        self._publish(msg.header.stamp)

    # ── scan callback: EKF UPDATE step (LiDAR ICP measurement) ───────────────

    def scan_callback(self, msg: LaserScan):
        pts = scan_to_cartesian(msg)
        if len(pts) < self.min_pts:
            return

        if self.last_scan_pts is None or len(self.last_scan_pts) < self.min_pts:
            self.last_scan_pts = pts
            return

        try:
            dx, dy, dth = icp_2d(self.last_scan_pts, pts)
        except Exception as e:
            self.get_logger().warn(f'ICP failed: {e}')
            self.last_scan_pts = pts
            return

        self.last_scan_pts = pts

        # ICP gives relative motion in sensor frame → map frame
        th_prev = self.mu[2] - dth
        z = np.array([
            self.mu[0] + dx * cos(th_prev) - dy * sin(th_prev),
            self.mu[1] + dx * sin(th_prev) + dy * cos(th_prev),
            normalize(self.mu[2])
        ])

        # ── Update ──────────────────────────────────────────────────────────
        H = np.eye(3)   # measurement model is identity (direct pose obs)
        S = H @ self.P @ H.T + self.R_lidar
        K = self.P @ H.T @ np.linalg.inv(S)   # Kalman gain

        innovation = z - self.mu
        innovation[2] = normalize(innovation[2])

        self.mu = self.mu + K @ innovation
        self.mu[2] = normalize(self.mu[2])
        self.P = (np.eye(3) - K @ H) @ self.P

    # ── publish filtered odometry & covariance marker ────────────────────────

    def _publish(self, stamp):
        out = Odometry()
        out.header.stamp    = stamp
        out.header.frame_id = self.frame_id
        out.child_frame_id  = 'base_footprint'

        out.pose.pose.position.x = self.mu[0]
        out.pose.pose.position.y = self.mu[1]
        q = quaternion_from_euler(0.0, 0.0, self.mu[2])
        out.pose.pose.orientation.x = q[0]
        out.pose.pose.orientation.y = q[1]
        out.pose.pose.orientation.z = q[2]
        out.pose.pose.orientation.w = q[3]

        # Flatten 3×3 pose covariance into ROS 6×6 (x,y,z,rx,ry,rz)
        cov6 = np.zeros((6, 6))
        cov6[0, 0] = self.P[0, 0]
        cov6[0, 1] = self.P[0, 1]
        cov6[1, 0] = self.P[1, 0]
        cov6[1, 1] = self.P[1, 1]
        cov6[5, 5] = self.P[2, 2]
        out.pose.covariance = cov6.flatten().tolist()

        self.filtered_pub.publish(out)
        self._publish_covariance_marker(stamp)

    def _publish_covariance_marker(self, stamp):
        """Publish a 2-σ ellipse marker representing XY uncertainty."""
        m = Marker()
        m.header.stamp    = stamp
        m.header.frame_id = self.frame_id
        m.ns   = 'ekf_covariance'
        m.id   = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position.x = self.mu[0]
        m.pose.position.y = self.mu[1]
        m.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.mu[2])
        m.pose.orientation.x = q[0]
        m.pose.orientation.y = q[1]
        m.pose.orientation.z = q[2]
        m.pose.orientation.w = q[3]

        # 2-sigma scale
        m.scale.x = 2.0 * sqrt(max(self.P[0, 0], 1e-9))
        m.scale.y = 2.0 * sqrt(max(self.P[1, 1], 1e-9))
        m.scale.z = 0.05

        m.color.r = 0.0
        m.color.g = 0.8
        m.color.b = 0.2
        m.color.a = 0.4
        m.lifetime.sec = 0
        self.marker_pub.publish(m)


# ─── entry point ──────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = EKFOdometryCorrector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()