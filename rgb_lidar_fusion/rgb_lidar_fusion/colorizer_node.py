"""Live colorization of lidar scans from an RGB camera.

Each lidar scan is coloured against the camera image nearest in time to the
middle of the sweep:

  1. look up T_cam_lidar from TF (lidar frame -> camera optical frame),
  2. transform the scan into the camera frame and project it into the image,
  3. reject points the camera cannot see (behind a nearer surface),
  4. sample each surviving point's colour.

Outputs:
  * `colored_scan` -- every coloured scan, in the lidar frame, as XYZRGB.
  * `colored_map`  -- (if `accumulate`) a voxel grid that keeps the best view of
                      each voxel across scans. Without odometry this assumes the
                      rig is STATIONARY; moving it smears the map.

Timing: livox_ros_driver2 stamps a scan with the time of its FIRST point and the
sweep lasts ~0.1 s, so the scan's content is centred on stamp + 0.05 s
(`scan_mid_offset`). Both drivers stamp with the host clock. A scan reaches us
~0.10 s after its first point but the image taken mid-sweep only ~0.13 s after
(camera latency), so each scan is held until an image from after its midpoint
has arrived (or `max_wait` passes) before the nearest image is chosen.
Choosing immediately would pick the previous frame and drop half the scans.
"""
import os
import signal
import time
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from rgb_lidar_fusion.fusion import Accumulator, view_quality
from rgb_lidar_fusion.io_cloud import write_both
from rgb_lidar_fusion.projection import CameraModel, occlusion_mask, project
from rgb_lidar_fusion.ros_conv import (
    image_to_rgb,
    pointcloud2_to_xyz,
    stamp_to_sec,
    transform_to_matrix,
    xyzrgb_to_msg,
)


class ColorizerNode(Node):

    def __init__(self):
        super().__init__('colorizer')

        p = self.declare_parameter
        p('lidar_topic', '/livox/lidar')
        p('image_topic', '/camera/camera/color/image_raw')
        p('camera_info_topic', '/camera/camera/color/camera_info')
        p('camera_frame', '')            # '' -> use camera_info's frame_id
        p('scan_mid_offset', 0.05)       # s, scan stamp -> sweep midpoint
        p('time_offset', 0.0)            # s, added to image stamps
        p('max_dt', 0.05)                # s, max scan-midpoint-to-image gap
        p('min_range', 0.1)              # m, lidar blind zone
        p('max_range', 40.0)             # m, camera depth
        p('occl_cell', 4)                # px
        p('occl_tol', 0.10)              # m
        p('max_buffered_images', 30)
        p('max_wait', 0.3)               # s, how long a scan may wait for its image
        p('accumulate', False)
        p('voxel', 0.03)                 # m
        p('publish_period', 1.0)         # s, colored_map republish
        p('output_dir', os.path.join(os.getcwd(), 'output'))
        p('output_stem', 'colored_cloud')
        p('save_on_shutdown', True)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.camera_frame = str(g('camera_frame'))
        self.scan_mid_offset = float(g('scan_mid_offset'))
        self.time_offset = float(g('time_offset'))
        self.max_dt = float(g('max_dt'))
        self.max_wait = float(g('max_wait'))
        self.min_range = float(g('min_range'))
        self.max_range = float(g('max_range'))
        self.occl_cell = int(g('occl_cell'))
        self.occl_tol = float(g('occl_tol'))
        self.accumulate = bool(g('accumulate'))
        self.publish_period = float(g('publish_period'))
        self.output_dir = str(g('output_dir'))
        self.output_stem = str(g('output_stem'))

        self.cam: Optional[CameraModel] = None
        # (t, Image msg); decoded only when chosen, so buffering is cheap.
        self._images = deque(maxlen=int(g('max_buffered_images')))
        self._pending = deque(maxlen=20)   # (t_scan_mid, arrival monotonic, msg)
        self.acc = Accumulator(voxel=float(g('voxel')))
        self._last_map_pub = 0.0
        self._gaps = deque(maxlen=2000)
        self.stats = dict(scans=0, coloured=0, no_camera_info=0, no_tf=0,
                          no_image=0, dt_too_big=0, empty=0, images_rx=0,
                          points_in=0, points_coloured=0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, g('camera_info_topic'),
                                 self._on_camera_info, 5)
        # RELIABLE, not best-effort: a 720p frame (2.7 MB) is larger than the
        # DDS shared-memory segment, so it travels as many UDP fragments, and
        # best-effort drops the whole frame if any fragment is lost. Measured on
        # a D435i at 15 fps: best-effort lost about half the frames (gaps up to
        # 0.87 s); reliable lost none.
        image_qos = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, g('image_topic'), self._on_image, image_qos)
        self.create_subscription(PointCloud2, g('lidar_topic'), self._on_cloud, 5)
        self.scan_pub = self.create_publisher(PointCloud2, 'colored_scan', 5)
        self.map_pub = self.create_publisher(PointCloud2, 'colored_map', 1)
        self.create_service(Trigger, '~/save_cloud', self._on_save_request)
        self.create_timer(5.0, self._log_progress)

        self.get_logger().info(
            'colorizer up: max_dt={:.3f}s scan_mid_offset={:+.3f}s '
            'time_offset={:+.3f}s range=[{:.2f},{:.1f}]m occl(cell={},tol={:.2f}m) '
            'accumulate={} voxel={:.3f}m'.format(
                self.max_dt, self.scan_mid_offset, self.time_offset,
                self.min_range, self.max_range, self.occl_cell, self.occl_tol,
                self.accumulate, self.acc.voxel))

    # --- inputs ---------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo):
        if self.cam is None:
            self.cam = CameraModel.from_camera_info(msg)
            if not self.camera_frame:
                self.camera_frame = msg.header.frame_id
            self.get_logger().info('camera {} in frame {!r}'.format(
                self.cam, self.camera_frame))

    def _on_image(self, msg: Image):
        self.stats['images_rx'] += 1
        self._images.append((stamp_to_sec(msg.header.stamp) + self.time_offset, msg))
        self._process_ready()

    def _on_cloud(self, msg: PointCloud2):
        self.stats['scans'] += 1
        if self.cam is None:
            self.stats['no_camera_info'] += 1
            return
        t_scan = stamp_to_sec(msg.header.stamp) + self.scan_mid_offset
        self._pending.append((t_scan, time.monotonic(), msg))
        self._process_ready()

    def _process_ready(self):
        """Colorize every pending scan whose image bracket is complete: an image
        stamped after the scan midpoint has arrived, or the scan waited too long."""
        newest = self._images[-1][0] if self._images else float('-inf')
        now = time.monotonic()
        while self._pending:
            t_scan, arrived, msg = self._pending[0]
            if newest < t_scan and now - arrived < self.max_wait:
                return
            self._pending.popleft()
            self._colorize(t_scan, msg)

    def _lookup_T_cam_lidar(self, lidar_frame: str) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(self.camera_frame, lidar_frame, Time())
        except TransformException as exc:
            self.stats['no_tf'] += 1
            self.get_logger().warn('no TF {} -> {}: {}'.format(
                lidar_frame, self.camera_frame, exc), throttle_duration_sec=5.0)
            return None
        return transform_to_matrix(tf.transform)

    # --- the pipeline ---------------------------------------------------

    def _colorize(self, t_scan: float, msg: PointCloud2):
        T_cam_lidar = self._lookup_T_cam_lidar(msg.header.frame_id)
        if T_cam_lidar is None:
            return
        if not self._images:
            self.stats['no_image'] += 1
            return

        t_img, img_msg = min(self._images, key=lambda e: abs(e[0] - t_scan))
        self._gaps.append(t_img - t_scan)
        if abs(t_img - t_scan) > self.max_dt:
            self.stats['dt_too_big'] += 1
            return

        pts_lidar = pointcloud2_to_xyz(msg)
        self.stats['points_in'] += pts_lidar.shape[0]
        # The Mid-360 reports no-return points as (0, 0, 0); drop them and
        # anything inside the blind zone before they reach the camera maths.
        pts_lidar = pts_lidar[np.linalg.norm(pts_lidar, axis=1) > self.min_range]

        pts_cam = pts_lidar @ T_cam_lidar[:3, :3].T + T_cam_lidar[:3, 3]
        depth = pts_cam[:, 2]
        near = (depth > self.min_range) & (depth < self.max_range)
        pts_cam, pts_lidar = pts_cam[near], pts_lidar[near]

        uv, valid = project(self.cam, pts_cam)
        uv, pts_cam, pts_lidar = uv[valid], pts_cam[valid], pts_lidar[valid]
        keep = occlusion_mask(uv, pts_cam[:, 2], (self.cam.height, self.cam.width),
                              cell=self.occl_cell, tol=self.occl_tol)
        uv, pts_cam, pts_lidar = uv[keep], pts_cam[keep], pts_lidar[keep]
        if pts_lidar.shape[0] == 0:
            self.stats['empty'] += 1
            return

        try:
            img = image_to_rgb(img_msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        cols = np.clip(np.rint(uv[:, 0]), 0, self.cam.width - 1).astype(np.int32)
        rows = np.clip(np.rint(uv[:, 1]), 0, self.cam.height - 1).astype(np.int32)
        rgb = img[rows, cols]

        self.stats['coloured'] += 1
        self.stats['points_coloured'] += pts_lidar.shape[0]
        self.scan_pub.publish(xyzrgb_to_msg(pts_lidar, rgb, msg.header.frame_id,
                                            msg.header.stamp))

        if self.accumulate:
            quality = view_quality(pts_cam, uv, self.cam, max_range=self.max_range)
            self.acc.add(pts_lidar, rgb, quality)
            self._maybe_publish_map(msg.header.frame_id)

    def _maybe_publish_map(self, frame_id: str):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_map_pub < self.publish_period:
            return
        self._last_map_pub = now
        xyz, rgb = self.acc.export()
        if xyz.shape[0]:
            self.map_pub.publish(xyzrgb_to_msg(
                xyz, rgb, frame_id, self.get_clock().now().to_msg()))

    # --- saving and reporting -------------------------------------------

    def save(self):
        os.makedirs(self.output_dir, exist_ok=True)
        xyz, rgb = self.acc.export()
        paths = write_both(os.path.join(self.output_dir, self.output_stem), xyz, rgb)
        self.get_logger().info('wrote {} voxels to {} and {}'.format(
            xyz.shape[0], *paths))
        return paths

    def _on_save_request(self, request, response):
        if not self.accumulate:
            response.success = False
            response.message = 'accumulate is off; nothing to save'
            return response
        try:
            response.message = 'wrote {} and {}'.format(*self.save())
            response.success = True
        except Exception as exc:                       # noqa: BLE001
            response.success = False
            response.message = 'save failed: {}'.format(exc)
        return response

    def _log_progress(self):
        s = self.stats
        gap = ''
        if self._gaps:
            g = np.asarray(self._gaps)
            gap = ' | gap(img-scan) median {:+.3f}s p95|.| {:.3f}s'.format(
                float(np.median(g)), float(np.percentile(np.abs(g), 95)))
        self.get_logger().info(
            'scans={} coloured={} img_rx={} pts/scan={:.0f} | drops: no_info={} '
            'no_tf={} no_img={} dt={} empty={}{}{}'.format(
                s['scans'], s['coloured'], s['images_rx'],
                s['points_coloured'] / max(s['coloured'], 1),
                s['no_camera_info'], s['no_tf'], s['no_image'], s['dt_too_big'],
                s['empty'], gap,
                ' | voxels={}'.format(self.acc.count) if self.accumulate else ''))


def main(args=None):
    rclpy.init(args=args)
    node = ColorizerNode()

    # ros2 launch stops nodes with SIGTERM, which does not raise
    # KeyboardInterrupt; without this the shutdown save never runs.
    def _on_term(signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_term)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.accumulate and node.get_parameter('save_on_shutdown').value \
                and node.acc.count:
            try:
                node.save()
            except Exception as exc:                   # noqa: BLE001
                node.get_logger().error('save on shutdown failed: {}'.format(exc))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
