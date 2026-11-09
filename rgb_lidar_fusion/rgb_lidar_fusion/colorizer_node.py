"""Live colorization of lidar scans from an RGB camera, and map recording.

Each lidar scan is coloured against the camera image nearest in time to the
middle of the sweep:

  1. look up T_cam_lidar from TF (lidar frame -> camera optical frame),
  2. transform the scan into the camera frame and project it into the image,
  3. reject points the camera cannot see (behind a nearer surface),
  4. sample each surviving point's colour.

While a map is RECORDING, two voxel maps grow side by side:
  * colored -- the best-viewed colour per voxel, only where the camera sees;
  * full    -- every lidar point (one per voxel, with intensity), because the
               camera covers only part of the lidar's field of view.
Without odometry both assume the rig is STATIONARY; moving it smears the map.

Topics:   colored_scan (each coloured scan), colored_map, lidar_map, ~/status
Services: ~/start_map, ~/stop_map (saves), ~/cancel_map, ~/save_cloud
          (std_srvs/Trigger; the map name comes from the `map_name` parameter)
Saved:    <output_dir>/<name>/colored.{pcd,ply}, full.{pcd,ply}, info.json

Timing: livox_ros_driver2 stamps a scan with the time of its FIRST point and the
sweep lasts ~0.1 s, so the scan's content is centred on stamp + 0.05 s
(`scan_mid_offset`). Both drivers stamp with the host clock. A scan reaches us
~0.10 s after its first point but the image taken mid-sweep only ~0.13 s after
(camera latency), so each scan is held until an image from after its midpoint
has arrived (or `max_wait` passes) before the nearest image is chosen.
Choosing immediately would pick the previous frame and drop half the scans.
"""
import json
import os
import re
import shutil
import signal
import time
from collections import deque
from datetime import datetime
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from rgb_lidar_fusion.fusion import Accumulator, VoxelGrid, view_quality
from rgb_lidar_fusion.io_cloud import write_both, write_pcd_xyzi, write_ply_xyzi
from rgb_lidar_fusion.projection import CameraModel, occlusion_mask, project
from rgb_lidar_fusion.ros_conv import (
    image_to_rgb,
    pointcloud2_field,
    pointcloud2_to_xyz,
    stamp_to_sec,
    transform_to_matrix,
    xyzi_to_msg,
    xyzrgb_to_msg,
)

MAP_NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,64}$')


def valid_map_name(name: str) -> bool:
    return bool(MAP_NAME_RE.match(name or ''))


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
        p('accumulate', False)           # start recording `map_name` at launch
        p('map_name', '')                # name for the next ~/start_map
        p('voxel', 0.03)                 # m
        p('publish_period', 1.0)         # s, colored_map republish
        p('full_publish_period', 2.0)    # s, lidar_map republish (it is ~40x larger)
        p('image_queue', 5)              # images DDS may hold while we are busy
        p('output_dir', os.path.join(os.getcwd(), 'output'))
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
        self.voxel = float(g('voxel'))
        self.publish_period = float(g('publish_period'))
        self.full_publish_period = float(g('full_publish_period'))
        self.output_dir = str(g('output_dir'))

        self.cam: Optional[CameraModel] = None
        # (t, Image msg); decoded only when chosen, so buffering is cheap.
        self._images = deque(maxlen=int(g('max_buffered_images')))
        # (t_scan_mid, arrival monotonic, frame_id, stamp, xyz) awaiting an image
        self._pending = deque(maxlen=20)
        self._gaps = deque(maxlen=2000)
        self._frame_id = 'livox_frame'
        self._last_map_pub = 0.0
        self._last_full_pub = 0.0
        self._counts = (0, 0)              # (colored, full), refreshed when maps are published
        self.stats = dict(scans=0, coloured=0, no_camera_info=0, no_tf=0,
                          no_image=0, dt_too_big=0, empty=0, images_rx=0,
                          points_in=0, points_coloured=0)

        # recording state
        self.recording = False
        self.map_name: Optional[str] = None
        self.rec_started: Optional[float] = None
        self.rec_scans = 0
        self.last_saved: Optional[str] = None
        self.acc = Accumulator(voxel=self.voxel)
        self.full = VoxelGrid(voxel=self.voxel)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, g('camera_info_topic'),
                                 self._on_camera_info, 5)
        # RELIABLE, not best-effort: a full-resolution frame is larger than the
        # DDS shared-memory segment, so it travels as many UDP fragments, and
        # best-effort drops the whole frame if any fragment is lost. Measured on
        # a D435i at 15 fps: best-effort lost about half the frames (gaps up to
        # 0.87 s); reliable lost none.
        # Depth 5 (~0.33 s at 15 fps) rides out a busy moment (map publishing)
        # without losing frames; 1080p RGB is ~6 MB a frame, so ~30 MB at most.
        image_qos = QoSProfile(depth=int(g('image_queue')), history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, g('image_topic'), self._on_image, image_qos)
        self.create_subscription(PointCloud2, g('lidar_topic'), self._on_cloud, 5)
        self.scan_pub = self.create_publisher(PointCloud2, 'colored_scan', 5)
        self.map_pub = self.create_publisher(PointCloud2, 'colored_map', 1)
        self.full_pub = self.create_publisher(PointCloud2, 'lidar_map', 1)
        self.status_pub = self.create_publisher(String, '~/status', 1)
        self.create_service(Trigger, '~/start_map', self._srv_start)
        self.create_service(Trigger, '~/stop_map', self._srv_stop)
        self.create_service(Trigger, '~/cancel_map', self._srv_cancel)
        self.create_service(Trigger, '~/save_cloud', self._srv_save)
        self.create_timer(1.0, self._publish_status)
        self.create_timer(5.0, self._log_progress)

        self.get_logger().info(
            'colorizer up: max_dt={:.3f}s scan_mid_offset={:+.3f}s '
            'time_offset={:+.3f}s range=[{:.2f},{:.1f}]m occl(cell={},tol={:.2f}m) '
            'voxel={:.3f}m output_dir={}'.format(
                self.max_dt, self.scan_mid_offset, self.time_offset,
                self.min_range, self.max_range, self.occl_cell, self.occl_tol,
                self.voxel, self.output_dir))

        if bool(g('accumulate')):
            ok, msg = self.start_map(str(g('map_name')) or
                                     datetime.now().strftime('map_%Y%m%d_%H%M%S'))
            (self.get_logger().info if ok else self.get_logger().error)(msg)

    # --- recording ------------------------------------------------------

    def start_map(self, name: str):
        if self.recording:
            return False, 'already recording {!r}'.format(self.map_name)
        if not valid_map_name(name):
            return False, 'invalid map name {!r}: use 1-64 of A-Z a-z 0-9 _ -'.format(name)
        if os.path.exists(os.path.join(self.output_dir, name)):
            return False, 'a map named {!r} already exists'.format(name)
        self.acc = Accumulator(voxel=self.voxel)
        self.full = VoxelGrid(voxel=self.voxel)
        self.recording, self.map_name = True, name
        self._counts = (0, 0)
        self.rec_started, self.rec_scans = time.time(), 0
        return True, 'recording {!r}'.format(name)

    def save_map(self) -> str:
        """Write both clouds and info.json for the current map; returns the folder."""
        folder = os.path.join(self.output_dir, self.map_name)
        os.makedirs(folder, exist_ok=True)
        xyz_c, rgb = self.acc.export()
        xyz_f, inten = self.full.export()
        write_both(os.path.join(folder, 'colored'), xyz_c, rgb)
        write_pcd_xyzi(os.path.join(folder, 'full.pcd'), xyz_f, inten)
        write_ply_xyzi(os.path.join(folder, 'full.ply'), xyz_f, inten)
        info = dict(name=self.map_name,
                    started=datetime.fromtimestamp(self.rec_started).astimezone().isoformat(
                        timespec='seconds'),
                    duration_s=round(time.time() - self.rec_started, 1),
                    scans=self.rec_scans, colored_points=int(xyz_c.shape[0]),
                    full_points=int(xyz_f.shape[0]), voxel_m=self.voxel,
                    frame_id=self._frame_id)
        with open(os.path.join(folder, 'info.json'), 'w') as fh:
            json.dump(info, fh, indent=2)
        self.last_saved = self.map_name
        self.get_logger().info('saved map {!r}: {} colored + {} full points in {}'.format(
            self.map_name, info['colored_points'], info['full_points'], folder))
        return folder

    def stop_map(self, save: bool = True):
        if not self.recording:
            return False, 'not recording'
        name = self.map_name
        try:
            msg = 'saved {}'.format(self.save_map()) if save else 'cancelled {!r}'.format(name)
        finally:
            self.recording = False
        if not save:
            shutil.rmtree(os.path.join(self.output_dir, name), ignore_errors=True)
        return True, msg

    def _srv_start(self, request, response):
        name = str(self.get_parameter('map_name').value) or \
            datetime.now().strftime('map_%Y%m%d_%H%M%S')
        response.success, response.message = self.start_map(name)
        return response

    def _srv_stop(self, request, response):
        try:
            response.success, response.message = self.stop_map(save=True)
        except Exception as exc:                       # noqa: BLE001
            response.success, response.message = False, 'save failed: {}'.format(exc)
        return response

    def _srv_cancel(self, request, response):
        response.success, response.message = self.stop_map(save=False)
        return response

    def _srv_save(self, request, response):
        if not self.recording:
            response.success, response.message = False, 'not recording'
            return response
        try:
            response.success, response.message = True, 'saved {}'.format(self.save_map())
        except Exception as exc:                       # noqa: BLE001
            response.success, response.message = False, 'save failed: {}'.format(exc)
        return response

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
        self._frame_id = msg.header.frame_id
        xyz = pointcloud2_to_xyz(msg)
        self.stats['points_in'] += xyz.shape[0]
        # The Mid-360 reports no-return points as (0, 0, 0); drop them and
        # anything inside the blind zone.
        keep = np.linalg.norm(xyz, axis=1) > self.min_range
        xyz = xyz[keep]

        if self.recording:   # the full map needs no camera
            self.rec_scans += 1
            self.full.add(xyz, pointcloud2_field(msg, 'intensity')[keep])
            self._maybe_publish_maps()

        if self.cam is None:
            self.stats['no_camera_info'] += 1
            return
        t_scan = stamp_to_sec(msg.header.stamp) + self.scan_mid_offset
        self._pending.append((t_scan, time.monotonic(), msg.header.frame_id,
                              msg.header.stamp, xyz))
        self._process_ready()

    def _process_ready(self):
        """Colorize every pending scan whose image bracket is complete: an image
        stamped after the scan midpoint has arrived, or the scan waited too long."""
        newest = self._images[-1][0] if self._images else float('-inf')
        now = time.monotonic()
        while self._pending:
            t_scan, arrived = self._pending[0][:2]
            if newest < t_scan and now - arrived < self.max_wait:
                return
            _, _, frame_id, stamp, xyz = self._pending.popleft()
            self._colorize(t_scan, frame_id, stamp, xyz)

    def _lookup_T_cam_lidar(self, lidar_frame: str) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(self.camera_frame, lidar_frame, Time())
        except TransformException as exc:
            self.stats['no_tf'] += 1
            self.get_logger().warn('no TF {} -> {}: {}'.format(
                lidar_frame, self.camera_frame, exc), throttle_duration_sec=5.0)
            return None
        return transform_to_matrix(tf.transform)

    # --- the colour pipeline --------------------------------------------

    def _colorize(self, t_scan: float, frame_id: str, stamp, pts_lidar: np.ndarray):
        T_cam_lidar = self._lookup_T_cam_lidar(frame_id)
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
        self.scan_pub.publish(xyzrgb_to_msg(pts_lidar, rgb, frame_id, stamp))

        if self.recording:
            quality = view_quality(pts_cam, uv, self.cam, max_range=self.max_range)
            self.acc.add(pts_lidar, rgb, quality)

    def _maybe_publish_maps(self):
        """Republish the maps at their own rates. This is the one piece of work
        that grows with the map, so it runs only as often as needed, and the
        (big) messages are built only if someone is subscribed."""
        now = time.monotonic()
        stamp = None
        if now - self._last_map_pub >= self.publish_period:
            self._last_map_pub = now
            stamp = self.get_clock().now().to_msg()
            xyz, rgb = self.acc.export()
            self._counts = (xyz.shape[0], self._counts[1])
            if self.map_pub.get_subscription_count():
                self.map_pub.publish(xyzrgb_to_msg(xyz, rgb, self._frame_id, stamp))
        if now - self._last_full_pub >= self.full_publish_period:
            self._last_full_pub = now
            stamp = stamp or self.get_clock().now().to_msg()
            xyz, inten = self.full.export()
            self._counts = (self._counts[0], xyz.shape[0])
            if self.full_pub.get_subscription_count():
                self.full_pub.publish(xyzi_to_msg(xyz, inten, self._frame_id, stamp))

    # --- reporting ------------------------------------------------------

    def status(self) -> dict:
        s = self.stats
        return dict(
            recording=self.recording, map_name=self.map_name,
            elapsed_s=round(time.time() - self.rec_started, 1) if self.recording else 0.0,
            colored_points=self._counts[0], full_points=self._counts[1],
            rec_scans=self.rec_scans, scans=s['scans'], coloured=s['coloured'],
            camera_ok=self.cam is not None, last_saved=self.last_saved,
            output_dir=self.output_dir)

    def _publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.status())))

    def _log_progress(self):
        s = self.stats
        gap = ''
        if self._gaps:
            g = np.asarray(self._gaps)
            gap = ' | gap(img-scan) median {:+.3f}s p95|.| {:.3f}s'.format(
                float(np.median(g)), float(np.percentile(np.abs(g), 95)))
        rec = ' | REC {!r} colored={} full={}'.format(
            self.map_name, *self._counts) if self.recording else ''
        self.get_logger().info(
            'scans={} coloured={} img_rx={} pts/scan={:.0f} | drops: no_info={} '
            'no_tf={} no_img={} dt={} empty={}{}{}'.format(
                s['scans'], s['coloured'], s['images_rx'],
                s['points_coloured'] / max(s['coloured'], 1),
                s['no_camera_info'], s['no_tf'], s['no_image'], s['dt_too_big'],
                s['empty'], gap, rec))


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
        if node.recording and node.get_parameter('save_on_shutdown').value:
            try:
                node.stop_map(save=True)
            except Exception as exc:                   # noqa: BLE001
                node.get_logger().error('save on shutdown failed: {}'.format(exc))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
