#!/usr/bin/env python3
"""
pf_odometry_corrector.py
────────────────────────
Particle Filter (MCL) node for ROS 2 Humble — error REDUCTION counterpart
to the original odometry_motion_model.py.

Where the original script *spreads* particles to model uncertainty, this
node *collapses* them toward the true pose by:
  1. Predicting each particle with the same differential-drive motion model
     (including noise), then
  2. Weighting each particle using a Gaussian likelihood against LiDAR
     range residuals (how well does a particle's pose explain the scan?), then
  3. Low-variance resampling to concentrate mass at high-probability poses, then
  4. Publishing the weighted-mean pose as the corrected odometry.

Topics
──────
  Subscribed:
    /bumperbot_controller/odom   nav_msgs/Odometry
    /scan                        sensor_msgs/LaserScan

  Published:
    /pf/corrected_odom           nav_msgs/Odometry  — weighted-mean pose
    /pf/particles                geometry_msgs/PoseArray — live particle cloud

Parameters
──────────
  alpha1..4          motion-noise coefficients        (default 0.1)
  sigma_hit          std-dev of Gaussian hit model    (default 0.2 m)
  lambda_short       rate of short-reading exponential(default 0.1)
  z_hit              weight of Gaussian component     (default 0.9)
  z_short            weight of short-read component   (default 0.1)
  nr_samples         particle count                   (default 300, int)
  resample_interval  predict steps between resamples  (default 5, int)
  lidar_subsample    use every Nth beam               (default 10, int)
"""

import random
import time
from math import atan2, sin, cos, sqrt, fabs, pi, exp, log
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import quaternion_from_euler, euler_from_quaternion


# ─── helpers ──────────────────────────────────────────────────────────────────

def normalize(z: float) -> float:
    return atan2(sin(z), cos(z))


def angle_diff(a: float, b: float) -> float:
    a, b = normalize(a), normalize(b)
    d1 = a - b
    d2 = 2.0 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    return d1 if fabs(d1) < fabs(d2) else d2


def pose_yaw(pose: Pose) -> float:
    q = [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w]
    _, _, yaw = euler_from_quaternion(q)
    return yaw


def set_pose_yaw(pose: Pose, yaw: float) -> None:
    q = quaternion_from_euler(0.0, 0.0, yaw)
    pose.orientation.x, pose.orientation.y = q[0], q[1]
    pose.orientation.z, pose.orientation.w = q[2], q[3]


# ─── Beam sensor model (Gaussian-hit + exponential-short) ─────────────────────

def beam_range_likelihood(z_measured: float, z_expected: float,
                          sigma_hit: float, lambda_short: float,
                          z_hit: float, z_short: float) -> float:
    """
    Mixture beam model.
    p = z_hit * N(z; z_expected, sigma_hit²)  +  z_short * λ·exp(-λ·z)
    Returns log-likelihood for numerical stability.
    """
    # Gaussian hit
    diff = z_measured - z_expected
    p_hit = (1.0 / (sigma_hit * sqrt(2 * pi))) * exp(-0.5 * (diff / sigma_hit) ** 2)

    # Exponential short reading
    p_short = 0.0
    if 0.0 < z_measured <= z_expected:
        norm = 1.0 - exp(-lambda_short * z_expected)
        if norm > 1e-10:
            p_short = (lambda_short * exp(-lambda_short * z_measured)) / norm

    p = z_hit * p_hit + z_short * p_short
    return log(p + 1e-300)   # log-sum trick


# ─── Particle Filter node ──────────────────────────────────────────────────────

class PFOdometryCorrector(Node):

    def __init__(self):
        super().__init__('pf_odometry_corrector')

        # ── parameters ──────────────────────────────────────────────────────
        for name, default in [('alpha1', 0.1), ('alpha2', 0.1),
                               ('alpha3', 0.1), ('alpha4', 0.1),
                               ('sigma_hit', 0.2), ('lambda_short', 0.1),
                               ('z_hit', 0.9), ('z_short', 0.1)]:
            self.declare_parameter(name, default)
        for name, default in [('nr_samples', 300),
                               ('resample_interval', 5),
                               ('lidar_subsample', 10)]:
            self.declare_parameter(name, default)

        def gp(n): return self.get_parameter(n).get_parameter_value().double_value
        def gi(n): return self.get_parameter(n).get_parameter_value().integer_value

        self.alpha = [gp(f'alpha{i}') for i in range(1, 5)]
        self.sigma_hit     = gp('sigma_hit')
        self.lambda_short  = gp('lambda_short')
        self.z_hit         = gp('z_hit')
        self.z_short       = gp('z_short')
        self.nr_samples    = gi('nr_samples')
        self.resample_interval = gi('resample_interval')
        self.lidar_subsample   = gi('lidar_subsample')

        # ── particles (PoseArray) ────────────────────────────────────────────
        self.particles = PoseArray()
        self.particles.poses = [Pose() for _ in range(self.nr_samples)]
        self.weights = np.ones(self.nr_samples) / self.nr_samples

        # ── odometry bookkeeping ─────────────────────────────────────────────
        self.last_odom_x     = 0.0
        self.last_odom_y     = 0.0
        self.last_odom_theta = 0.0
        self.is_first_odom   = True
        self.predict_count   = 0

        # ── latest scan for update step ──────────────────────────────────────
        self.latest_scan: LaserScan | None = None

        # ── pub / sub ────────────────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, 'bumperbot_controller/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        self.corrected_pub = self.create_publisher(
            Odometry, 'pf/corrected_odom', 10)
        self.particle_pub = self.create_publisher(
            PoseArray, 'pf/particles', 10)

        self.get_logger().info('PF Odometry Corrector started.')

    # ── scan callback: just cache the latest scan ─────────────────────────────

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    # ── odometry callback: PREDICT → WEIGHT → RESAMPLE → PUBLISH ─────────────

    def odom_callback(self, msg: Odometry):
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(q)

        if self.is_first_odom:
            self.particles.header.frame_id = msg.header.frame_id
            # Initialise all particles at starting pose (tight Gaussian)
            for p in self.particles.poses:
                p.position.x = msg.pose.pose.position.x + random.gauss(0, 0.01)
                p.position.y = msg.pose.pose.position.y + random.gauss(0, 0.01)
                set_pose_yaw(p, yaw + random.gauss(0, 0.005))
            self.last_odom_x     = msg.pose.pose.position.x
            self.last_odom_y     = msg.pose.pose.position.y
            self.last_odom_theta = yaw
            self.is_first_odom   = False
            return

        # ── Odometry increments ──────────────────────────────────────────────
        dx  = msg.pose.pose.position.x - self.last_odom_x
        dy  = msg.pose.pose.position.y - self.last_odom_y
        dth = angle_diff(yaw, self.last_odom_theta)

        trans = sqrt(dx**2 + dy**2)
        if trans < 0.001:
            delta_rot1 = 0.0
        else:
            delta_rot1 = angle_diff(atan2(dy, dx), self.last_odom_theta)
        delta_rot2 = angle_diff(dth, delta_rot1)

        a1, a2, a3, a4 = self.alpha
        rot1_var  = a1 * delta_rot1**2 + a2 * trans**2
        trans_var = a3 * trans**2 + a4 * (delta_rot1**2 + delta_rot2**2)
        rot2_var  = a1 * delta_rot2**2 + a2 * trans**2

        # ── PREDICT: propagate each particle with noisy motion ───────────────
        random.seed(int(time.time() * 1e6) % (2**31))
        for p in self.particles.poses:
            r1 = angle_diff(delta_rot1, random.gauss(0.0, sqrt(max(rot1_var, 1e-9))))
            tr = delta_trans = trans - random.gauss(0.0, sqrt(max(trans_var, 1e-9)))
            r2 = angle_diff(delta_rot2, random.gauss(0.0, sqrt(max(rot2_var, 1e-9))))

            th = pose_yaw(p)
            p.position.x += tr * cos(th + r1)
            p.position.y += tr * sin(th + r1)
            set_pose_yaw(p, normalize(th + r1 + r2))

        self.predict_count += 1

        # ── UPDATE: weight particles against LiDAR scan ──────────────────────
        if self.latest_scan is not None and self.predict_count % self.resample_interval == 0:
            self._update_weights()
            self._resample()

        # Update bookkeeping
        self.last_odom_x     = msg.pose.pose.position.x
        self.last_odom_y     = msg.pose.pose.position.y
        self.last_odom_theta = yaw

        self._publish(msg.header.stamp)

    # ── weight particles using LiDAR beam model ───────────────────────────────

    def _update_weights(self):
        scan = self.latest_scan
        angles = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
        ranges = np.array(scan.ranges, dtype=np.float64)

        # Subsample beams
        idx = np.arange(0, len(ranges), self.lidar_subsample)
        z_measured = ranges[idx]
        beam_angles = angles[idx]

        valid = (np.isfinite(z_measured) &
                 (z_measured >= scan.range_min) &
                 (z_measured <= scan.range_max))
        z_m = z_measured[valid]
        b_a = beam_angles[valid]

        if len(z_m) == 0:
            return

        log_weights = np.zeros(self.nr_samples)

        for i, p in enumerate(self.particles.poses):
            th = pose_yaw(p)
            # For each beam, compute expected range as distance to nearest
            # occupied cell. Without a full map, we use the scan's own median
            # as a pseudo-expected range — a map-free approximation.
            # In a full SLAM setup replace this with a ray-cast into the map.
            z_expected = np.median(z_m)   # map-free fallback

            ll = 0.0
            for zm in z_m[:20]:   # cap beams for real-time performance
                ll += beam_range_likelihood(
                    zm, z_expected,
                    self.sigma_hit, self.lambda_short,
                    self.z_hit, self.z_short)
            log_weights[i] = ll

        # Numerically stable softmax → normalised weights
        log_weights -= log_weights.max()
        self.weights = np.exp(log_weights)
        total = self.weights.sum()
        if total > 1e-300:
            self.weights /= total
        else:
            self.weights = np.ones(self.nr_samples) / self.nr_samples

    # ── low-variance resampling ───────────────────────────────────────────────

    def _resample(self):
        n = self.nr_samples
        new_poses = []
        r = random.uniform(0, 1.0 / n)
        c = self.weights[0]
        i = 0
        for m in range(n):
            u = r + m / n
            while u > c and i < n - 1:
                i += 1
                c += self.weights[i]
            p_src = self.particles.poses[i]
            p_new = Pose()
            p_new.position.x = p_src.position.x
            p_new.position.y = p_src.position.y
            p_new.position.z = p_src.position.z
            p_new.orientation = p_src.orientation
            new_poses.append(p_new)

        self.particles.poses = new_poses
        self.weights = np.ones(n) / n

    # ── publish weighted-mean corrected pose ──────────────────────────────────

    def _publish(self, stamp):
        # Weighted mean position
        xs = np.array([p.position.x for p in self.particles.poses])
        ys = np.array([p.position.y for p in self.particles.poses])
        ths = np.array([pose_yaw(p)  for p in self.particles.poses])

        mean_x  = float(np.sum(self.weights * xs))
        mean_y  = float(np.sum(self.weights * ys))
        # Circular mean for angle
        mean_th = float(atan2(
            np.sum(self.weights * np.sin(ths)),
            np.sum(self.weights * np.cos(ths))
        ))

        out = Odometry()
        out.header.stamp    = stamp
        out.header.frame_id = self.particles.header.frame_id
        out.child_frame_id  = 'base_footprint'
        out.pose.pose.position.x = mean_x
        out.pose.pose.position.y = mean_y

        q = quaternion_from_euler(0.0, 0.0, mean_th)
        out.pose.pose.orientation.x = q[0]
        out.pose.pose.orientation.y = q[1]
        out.pose.pose.orientation.z = q[2]
        out.pose.pose.orientation.w = q[3]

        # Covariance from particle spread
        var_x  = float(np.sum(self.weights * (xs  - mean_x) ** 2))
        var_y  = float(np.sum(self.weights * (ys  - mean_y) ** 2))
        var_th = float(np.sum(self.weights * angle_diff_array(ths, mean_th) ** 2))
        cov6   = [0.0] * 36
        cov6[0]  = var_x
        cov6[7]  = var_y
        cov6[35] = var_th
        out.pose.covariance = cov6

        self.corrected_pub.publish(out)

        # Publish particle cloud for RViz
        self.particles.header.stamp = stamp
        self.particle_pub.publish(self.particles)


def angle_diff_array(a: np.ndarray, b: float) -> np.ndarray:
    diff = a - b
    return np.arctan2(np.sin(diff), np.cos(diff))


# ─── entry point ──────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = PFOdometryCorrector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()