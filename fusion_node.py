#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import PointCloud2
from px4_msgs.msg import VehicleOdometry
import numpy as np

CHI2_THRESH = 7.815  # chi-squared 95% confidence, 3 DOF


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        qos_be = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        qos_rel = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        self.lio_pos  = None
        self.vio_pos  = None
        self.gps_pos  = None
        self.lio_prev = None

        self.P_lio = 10.0
        self.P_vio = 10.0
        self.P_gps = 0.02

        self.lio_feats = 0
        self.vio_pts   = 0

        self.create_subscription(Odometry,
            '/lio_sam/mapping/odometry', self.cb_lio, qos_be)
        self.create_subscription(Odometry,
            '/odometry', self.cb_vio, qos_be)
        self.create_subscription(VehicleOdometry,
            '/fmu/out/vehicle_odometry', self.cb_gps, qos_be)
        self.create_subscription(PointCloud2,
            '/lio_sam/feature/cloud_surface', self.cb_lio_feat, qos_be)
        self.create_subscription(PointCloud2,
            '/vins_estimator/point_cloud', self.cb_vio_pts, qos_be)

        self.pub_odom    = self.create_publisher(Odometry, '/fusion/odometry', qos_rel)
        self.pub_state   = self.create_publisher(String, '/fusion/state', qos_rel)
        self.pub_weights = self.create_publisher(Float32MultiArray, '/fusion/weights', qos_rel)

        self.create_timer(0.1, self.fuse)
        self.get_logger().info('Covariance-Gated Fusion Node starting...')
        self.get_logger().info(f'CHI2 thresholds: 95%={CHI2_THRESH}, 99%=11.345')

    def cb_lio(self, msg):
        p = msg.pose.pose.position
        pos = np.array([p.x, p.y, p.z])

        if self.lio_prev is not None:
            jump = np.linalg.norm(pos - self.lio_prev)
            if jump > 2.0:
                self.P_lio = 20.0
            elif jump > 0.5:
                self.P_lio = 2.0

        self.lio_prev = pos
        self.lio_pos  = pos

    def cb_vio(self, msg):
        p = msg.pose.pose.position
        self.vio_pos = np.array([p.x, p.y, p.z])

    def cb_gps(self, msg):
        # NED to ENU
        self.gps_pos = np.array([
            float(msg.position[1]),
            float(msg.position[0]),
            -float(msg.position[2])
        ])
        v = msg.position_variance
        self.P_gps = float(np.mean([float(v[0]), float(v[1]), float(v[2])]))

    def cb_lio_feat(self, msg):
        self.lio_feats = msg.width * msg.height
        if self.lio_feats > 200:   self.P_lio = 0.05
        elif self.lio_feats > 100: self.P_lio = 0.2
        elif self.lio_feats > 50:  self.P_lio = 1.0
        else:                      self.P_lio = 10.0

    def cb_vio_pts(self, msg):
        self.vio_pts = msg.width * msg.height
        if self.vio_pts > 80:   self.P_vio = 0.1
        elif self.vio_pts > 40: self.P_vio = 0.5
        elif self.vio_pts > 20: self.P_vio = 5.0
        else:                   self.P_vio = 50.0

    def fuse(self):
        if self.gps_pos is None:
            self._pub_state("INITIALIZING", {}, 1.0)
            return

        accepted = {}
        weights  = {}

        if self.P_gps < 1.0:
            accepted['gps'] = self.gps_pos
            weights['gps']  = 1.0 / self.P_gps

        if self.lio_pos is not None:
            r    = self.lio_pos - self.gps_pos
            chi2 = float(np.sum(r**2) / self.P_lio)
            if chi2 < CHI2_THRESH:
                accepted['lio'] = self.lio_pos
                weights['lio']  = 1.0 / self.P_lio

        if self.vio_pos is not None:
            r    = self.vio_pos - self.gps_pos
            chi2 = float(np.sum(r**2) / self.P_vio)
            if chi2 < CHI2_THRESH:
                accepted['vio'] = self.vio_pos
                weights['vio']  = 1.0 / self.P_vio

        if not accepted:
            self._pub_state("ALL_FAILED", {}, 1.0)
            return

        w_total = sum(weights.values())
        fused = np.zeros(3)
        for s in accepted:
            w = weights[s] / w_total
            fused = fused + w * accepted[s]

        l = 'lio' in accepted
        v = 'vio' in accepted
        g = 'gps' in accepted

        if l and v and g:
            state = "FULL"
        elif not l and v and g:
            state = "LIO_DEGRADED"
        elif l and not v and g:
            state = "VIO_DEGRADED"
        elif l and v and not g:
            state = "GPS_DEGRADED"
        elif not l and not v:
            state = "LIO_VIO_FAILED"
        else:
            state = "ALL_FAILED"

        self._pub_odom(fused)
        self._pub_state(state, weights, w_total)

        self.get_logger().info(
            f'Fusion state: {state} | '
            f'P_lio={self.P_lio:.3f} '
            f'P_vio={self.P_vio:.3f} '
            f'P_gps={self.P_gps:.4f}',
            throttle_duration_sec=2.0)

    def _pub_odom(self, pos):
        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id  = 'base_link'
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        self.pub_odom.publish(msg)

    def _pub_state(self, state, weights, w_total):
        s      = String()
        s.data = state
        self.pub_state.publish(s)

        w      = Float32MultiArray()
        w.data = [
            float(weights.get('lio', 0.0) / w_total),
            float(weights.get('vio', 0.0) / w_total),
            float(weights.get('gps', 0.0) / w_total)
        ]
        self.pub_weights.publish(w)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FusionNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
