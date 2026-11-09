"""Live colorization of lidar scans from an RGB camera, and map recording.

Works on registered scans from lidar odometry (FAST-LIO's /cloud_registered,
in its `camera_init` world frame), so the rig can be carried while mapping. It
also works on raw scans in the lidar frame for a stationary rig. Every pose
comes from TF, looked up at each image's own timestamp:

    world (camera_init) -> body (IMU) -> livox_frame -> camera_link -> camera optical

Per scan: pick the camera image nearest the middle of the sweep, transform the
points into the camera at that instant, project, reject points hidden behind
nearer ones (the two sensors see from different places), and sample colours.
Scans taken while the rig turns faster than `max_ang_rate` are not coloured
(motion would smear colour across edges), but still go into the full map.

Maps are kept in the `map` frame (see mapframe.py): gravity-levelled, floor at
z = 0, origin on the floor below the lidar's start, +X forward. Before a
recording that frame is estimated live from the IMU (the preview); pressing
Start freezes it, which needs the rig held still for a moment.

While RECORDING two voxel maps grow side by side:
  * colored -- best-viewed colour per voxel, where the camera sees;
  * full    -- every lidar point (one per voxel, with intensity).

Topics:   colored_scan, lidar_scan (map frame, each scan), colored_map,
          lidar_map (display copy, coarser), ~/status (JSON)
Services: ~/start_map, ~/stop_map (saves), ~/cancel_map, ~/save_cloud
          (std_srvs/Trigger; name and height from the `map_name` and
          `lidar_height` parameters)
Saved:    <output_dir>/<name>/  colored.{pcd,ply} full.{pcd,ply}   (map frame)
                                <robot>/colored.{pcd,ply} full.{pcd,ply}
                                trajectory.csv  info.json

Timing: FAST-LIO stamps a scan at the END of its 0.1 s sweep (raw livox scans:
at the start), so the content is centred on stamp + `scan_mid_offset`. A scan
can arrive before the image taken mid-sweep, so each scan is held until an
image from after its midpoint has arrived (or `max_wait` passes).
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
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from rgb_lidar_fusion.extrinsics import invert, matrix_to_quat
from rgb_lidar_fusion.fusion import Accumulator, VoxelGrid, view_quality
from rgb_lidar_fusion.io_cloud import write_both, write_pcd_xyzi, write_ply_xyzi
from rgb_lidar_fusion.mapframe import (apply, gravity_up, level_frame, path_length,
                                       robot_frame, rotation_angle, thin_path)
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


def _sec_to_time(t: float) -> Time:
    return Time(nanoseconds=int(round(t * 1e9)))


def _matrix_to_tf(T: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    """4x4 (parent <- child) -> TransformStamped."""
    x, y, z, w = matrix_to_quat(T[:3, :3])
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id, msg.header.stamp = parent, child, stamp
    t = msg.transform.translation
    t.x, t.y, t.z = (float(v) for v in T[:3, 3])
    r = msg.transform.rotation
    r.x, r.y, r.z, r.w = float(x), float(y), float(z), float(w)
    return msg


class ColorizerNode(Node):

    def __init__(self):
        super().__init__('colorizer')

        p = self.declare_parameter
        p('cloud_topic', '/cloud_registered')
        p('imu_topic', '/livox/imu')
        p('image_topic', '/camera/camera/color/image_raw')
        p('camera_info_topic', '/camera/camera/color/camera_info')
        p('camera_frame', '')            # '' -> use camera_info's frame_id
        p('lidar_frame', 'livox_frame')  # the lidar itself (forward axis, IMU axes)
        p('map_frame', 'map')
        p('scan_mid_offset', -0.05)      # s, scan stamp -> sweep midpoint
        p('time_offset', 0.0)            # s, added to image stamps
        p('max_dt', 0.05)                # s, max scan-midpoint-to-image gap
        p('max_ang_rate', 30.0)          # deg/s, faster turns are not coloured
        p('min_range', 0.1)              # m
        p('max_range', 40.0)             # m, camera depth
        p('occl_cell', 4)                # px
        p('occl_tol', 0.10)              # m
        p('max_buffered_images', 30)
        p('image_queue', 5)
        p('max_wait', 0.3)               # s, how long a scan may wait for its image
        p('lidar_height', 0.715)         # m, lidar origin above the floor at Start
        p('static_gyro', 0.05)           # rad/s, "held still" threshold for Start
        p('level_window', 1.0)           # s of IMU averaged for gravity
        p('accumulate', False)           # start recording `map_name` at launch
        p('map_name', '')
        p('overwrite_existing', False)   # one-shot: next ~/start_map may replace a map
        p('voxel', 0.03)                 # m, saved maps
        p('display_voxel', 0.10)         # m, live full map sent to viewers
        p('publish_period', 1.0)         # s, colored_map
        p('full_publish_period', 2.0)    # s, lidar_map
        p('output_dir', os.path.join(os.getcwd(), 'output'))
        p('save_on_shutdown', True)
        p('export_names', ['go2', 'g1'])
        p('export_heights', [0.45, 1.25])
        p('export_flip', [False, False])

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.camera_frame = str(g('camera_frame'))
        self.lidar_frame = str(g('lidar_frame'))
        self.map_frame = str(g('map_frame'))
        self.scan_mid_offset = float(g('scan_mid_offset'))
        self.time_offset = float(g('time_offset'))
        self.max_dt = float(g('max_dt'))
        self.max_ang_rate = np.radians(float(g('max_ang_rate')))
        self.max_wait = float(g('max_wait'))
        self.min_range = float(g('min_range'))
        self.max_range = float(g('max_range'))
        self.occl_cell = int(g('occl_cell'))
        self.occl_tol = float(g('occl_tol'))
        self.static_gyro = float(g('static_gyro'))
        self.level_window = float(g('level_window'))
        self.voxel = float(g('voxel'))
        self.default_height = float(g('lidar_height'))   # the UI's default
        self.display_voxel = float(g('display_voxel'))
        self.publish_period = float(g('publish_period'))
        self.full_publish_period = float(g('full_publish_period'))
        self.output_dir = str(g('output_dir'))
        names, heights, flips = list(g('export_names')), list(g('export_heights')), \
            list(g('export_flip'))
        if not (len(names) == len(heights) == len(flips)):
            raise ValueError('export_names, export_heights and export_flip differ in length')
        self.exports = [dict(name=str(n), height=float(h), flip=bool(f))
                        for n, h, f in zip(names, heights, flips) if valid_map_name(str(n))]

        self.cam: Optional[CameraModel] = None
        self._images = deque(maxlen=int(g('max_buffered_images')))
        self._pending = deque(maxlen=20)     # (t_mid, arrival, frame, stamp, xyz)
        self._imu = deque(maxlen=2000)       # (t, acc(3), gyro(3)), lidar axes
        self._gaps = deque(maxlen=2000)
        self._world = None                   # frame the clouds arrive in
        self._T_map_world = None             # live estimate (preview)
        self._last_map_pub = self._last_full_pub = 0.0
        self._counts = (0, 0)
        self.stats = dict(scans=0, coloured=0, no_camera_info=0, no_tf=0, no_image=0,
                          dt_too_big=0, too_fast=0, empty=0, images_rx=0, imu_rx=0)

        # recording state
        self.recording = False
        self.map_name: Optional[str] = None
        self.rec_started: Optional[float] = None
        self.rec_scans = 0
        self.rec_height = None
        self.rec_T = None                    # frozen T_map_world
        self.rec_replaces = False            # recording will replace an existing map
        self.path = []                       # (t, x, y, z) of the lidar in the map frame
        self.last_saved: Optional[str] = None
        self.acc = Accumulator(voxel=self.voxel)
        self.full = VoxelGrid(voxel=self.voxel)
        self.full_display = VoxelGrid(voxel=self.display_voxel)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        # own thread: lookups at an image's timestamp may need TF that is
        # still arriving, which a single-threaded spin could never deliver
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_pub = TransformBroadcaster(self)

        self.create_subscription(CameraInfo, g('camera_info_topic'), self._on_camera_info, 5)
        # RELIABLE: a full-resolution frame is larger than the DDS shared-memory
        # segment and travels as UDP fragments; best-effort lost about half the
        # frames in testing. Depth 5 rides out busy moments (~0.33 s at 15 fps).
        image_qos = QoSProfile(depth=int(g('image_queue')), history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, g('image_topic'), self._on_image, image_qos)
        self.create_subscription(PointCloud2, g('cloud_topic'), self._on_cloud, 5)
        self.create_subscription(Imu, g('imu_topic'), self._on_imu, 200)
        self.scan_pub = self.create_publisher(PointCloud2, 'colored_scan', 5)
        self.lidar_scan_pub = self.create_publisher(PointCloud2, 'lidar_scan', 2)
        self.map_pub = self.create_publisher(PointCloud2, 'colored_map', 1)
        self.full_pub = self.create_publisher(PointCloud2, 'lidar_map', 1)
        self.status_pub = self.create_publisher(String, '~/status', 1)
        self.create_service(Trigger, '~/start_map', self._srv_start)
        self.create_service(Trigger, '~/stop_map', self._srv_stop)
        self.create_service(Trigger, '~/cancel_map', self._srv_cancel)
        self.create_service(Trigger, '~/save_cloud', self._srv_save)
        self.create_timer(0.5, self._publish_status)    # 2 Hz: the turn warning reacts fast
        self.create_timer(5.0, self._log_progress)

        self.get_logger().info(
            'colorizer up: cloud={} max_dt={:.3f}s scan_mid_offset={:+.3f}s '
            'max_ang_rate={:.0f}deg/s voxel={:.3f}m exports={} output_dir={}'.format(
                g('cloud_topic'), self.max_dt, self.scan_mid_offset,
                np.degrees(self.max_ang_rate), self.voxel,
                ', '.join('{}@{:.2f}m{}'.format(e['name'], e['height'],
                                                ' flipped' if e['flip'] else '')
                          for e in self.exports), self.output_dir))

        if bool(g('accumulate')):
            self._autostart = str(g('map_name')) or datetime.now().strftime('map_%Y%m%d_%H%M%S')
            self._autostart_timer = self.create_timer(1.0, self._try_autostart)

    def _try_autostart(self):
        ok, msg = self.start_map(self._autostart)
        if ok or 'exists' in msg or 'invalid' in msg:
            (self.get_logger().info if ok else self.get_logger().error)(msg)
            self._autostart_timer.cancel()

    # --- TF helpers -----------------------------------------------------

    def _lookup(self, target: str, source: str, t: Optional[float] = None,
                fallback_latest: bool = True) -> Optional[np.ndarray]:
        """4x4 target <- source at time t (None: latest). Never waits: a blocking
        lookup stalls this node's only thread, and with it the IMU and images."""
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, Time() if t is None else _sec_to_time(t))
        except TransformException:
            if t is not None and fallback_latest:
                return self._lookup(target, source, None)
            return None
        return transform_to_matrix(tf.transform)

    # --- the map frame --------------------------------------------------

    def _imu_window(self):
        if not self._imu:
            return None, None
        t_end = self._imu[-1][0]
        rows = [e for e in self._imu if e[0] >= t_end - self.level_window]
        return np.array([e[1] for e in rows]), np.array([e[2] for e in rows])

    def _estimate_map_frame(self, height: float):
        """(T_map_world, still) from the latest pose and the recent IMU, or (None, why)."""
        if self._world is None:
            return None, 'no lidar scans yet'
        T_world_lidar = self._lookup(self._world, self.lidar_frame)
        if T_world_lidar is None:
            return None, 'no odometry (TF {} -> {})'.format(self._world, self.lidar_frame)
        acc, gyro = self._imu_window()
        if acc is None or len(acc) < 20:
            return None, 'no IMU data yet (just started or busy saving); try again in a second'
        R = T_world_lidar[:3, :3]
        T = level_frame(up=R @ gravity_up(acc), forward_hint=R[:, 0],
                        origin=T_world_lidar[:3, 3], height=height)
        still = float(np.linalg.norm(gyro, axis=1).mean()) < self.static_gyro
        return T, still

    def _current_T_map_world(self):
        return self.rec_T if self.recording else self._T_map_world

    # --- recording ------------------------------------------------------

    def start_map(self, name: str, overwrite: bool = False):
        if self.recording:
            return False, 'already recording {!r}'.format(self.map_name)
        if not valid_map_name(name):
            return False, 'invalid map name {!r}: use 1-64 of A-Z a-z 0-9 _ -'.format(name)
        exists = os.path.exists(os.path.join(self.output_dir, name))
        if exists and not overwrite:
            return False, 'a map named {!r} already exists'.format(name)
        height = float(self.get_parameter('lidar_height').value)
        if not 0.05 <= height <= 5.0:
            return False, 'lidar height {:.2f} m is out of range (0.05-5 m)'.format(height)
        try:
            T, still = self._estimate_map_frame(height)
        except ValueError as exc:
            return False, str(exc)
        if T is None:
            return False, 'cannot start: {}'.format(still)
        if not still:
            return False, 'hold the rig still for a second, then press Start again'
        self.acc = Accumulator(voxel=self.voxel)
        self.full = VoxelGrid(voxel=self.voxel)
        self.full_display = VoxelGrid(voxel=self.display_voxel)
        self.recording, self.map_name, self.rec_T, self.rec_height = True, name, T, height
        self.rec_replaces = exists       # the old map stays until this one is saved
        self.rec_started, self.rec_scans, self.path = time.time(), 0, []
        self._counts = (0, 0)
        self._too_fast_at_start = self.stats['too_fast']
        return True, 'recording {!r} (lidar at {:.3f} m){}'.format(
            name, height, ', will replace the existing map when saved' if exists else '')

    def save_map(self) -> str:
        """Write the map (map frame), each robot export, trajectory and info.

        Everything is written to a hidden temporary folder first and swapped in
        at the end, so an existing map is only replaced by a complete new one
        (a failed save leaves the old map as it was)."""
        final = os.path.join(self.output_dir, self.map_name)
        folder = os.path.join(self.output_dir, '.{}.saving'.format(self.map_name))
        shutil.rmtree(folder, ignore_errors=True)
        os.makedirs(folder)
        xyz_c, rgb = self.acc.export()
        xyz_f, inten = self.full.export()

        def write_set(where, T):
            os.makedirs(where, exist_ok=True)
            write_both(os.path.join(where, 'colored'), apply(T, xyz_c), rgb)
            pts = apply(T, xyz_f)
            write_pcd_xyzi(os.path.join(where, 'full.pcd'), pts, inten)
            write_ply_xyzi(os.path.join(where, 'full.ply'), pts, inten)

        write_set(folder, np.eye(4))
        for e in self.exports:
            write_set(os.path.join(folder, e['name']), robot_frame(e['height'], e['flip']))
        with open(os.path.join(folder, 'trajectory.csv'), 'w') as fh:
            fh.write('t,x,y,z\n')
            for row in self.path:
                fh.write('{:.3f},{:.4f},{:.4f},{:.4f}\n'.format(*row))
        path = np.array([r[1:] for r in self.path]) if self.path else np.zeros((0, 3))
        length = path_length(path)
        info = dict(
            name=self.map_name,
            started=datetime.fromtimestamp(self.rec_started).astimezone().isoformat(
                timespec='seconds'),
            duration_s=round(time.time() - self.rec_started, 1),
            scans=self.rec_scans, colored_points=int(xyz_c.shape[0]),
            full_points=int(xyz_f.shape[0]), voxel_m=self.voxel,
            lidar_height_m=self.rec_height, path_length_m=round(length, 2),
            frame='map: gravity-levelled, floor z=0, origin below the start lidar, +X forward',
            exports=[dict(e, frame='lidar at {:.2f} m{}'.format(
                e['height'], ', upside down' if e['flip'] else '')) for e in self.exports])
        with open(os.path.join(folder, 'info.json'), 'w') as fh:
            json.dump(info, fh, indent=2)
        if os.path.exists(final):
            shutil.rmtree(final)
        os.rename(folder, final)
        folder = final
        self.last_saved = self.map_name
        self.get_logger().info(
            'saved map {!r}: {} colored + {} full points, {} m walked, exports: {}'.format(
                self.map_name, info['colored_points'], info['full_points'],
                info['path_length_m'], ', '.join(e['name'] for e in self.exports)))
        return folder

    def stop_map(self, save: bool = True):
        if not self.recording:
            return False, 'not recording'
        name = self.map_name
        try:
            msg = 'saved {}'.format(self.save_map()) if save else 'cancelled {!r}'.format(name)
        finally:
            self.recording = False
        if not save and not self.rec_replaces:
            # a cancelled NEW map leaves nothing behind (e.g. from ~/save_cloud);
            # a cancelled overwrite leaves the old map untouched
            shutil.rmtree(os.path.join(self.output_dir, name), ignore_errors=True)
        return True, msg

    def _srv_start(self, request, response):
        name = str(self.get_parameter('map_name').value) or \
            datetime.now().strftime('map_%Y%m%d_%H%M%S')
        overwrite = bool(self.get_parameter('overwrite_existing').value)
        if overwrite:                                   # one-shot
            self.set_parameters([Parameter('overwrite_existing', Parameter.Type.BOOL, False)])
        response.success, response.message = self.start_map(name, overwrite)
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
            self.get_logger().info('camera {} in frame {!r}'.format(self.cam, self.camera_frame))

    def _on_imu(self, msg: Imu):
        self.stats['imu_rx'] += 1
        a, w = msg.linear_acceleration, msg.angular_velocity
        self._imu.append((stamp_to_sec(msg.header.stamp), (a.x, a.y, a.z), (w.x, w.y, w.z)))

    def _on_image(self, msg: Image):
        self.stats['images_rx'] += 1
        self._images.append((stamp_to_sec(msg.header.stamp) + self.time_offset, msg))
        self._process_ready()

    def _on_cloud(self, msg: PointCloud2):
        self.stats['scans'] += 1
        self._world = msg.header.frame_id
        xyz = pointcloud2_to_xyz(msg)
        inten = pointcloud2_field(msg, 'intensity')
        if self._world == self.lidar_frame:          # raw scans: drop no-returns
            keep = np.linalg.norm(xyz, axis=1) > self.min_range
            xyz, inten = xyz[keep], inten[keep]

        if not self.recording:                       # live estimate for the preview
            try:
                T, _ = self._estimate_map_frame(float(self.get_parameter('lidar_height').value))
                if T is not None:
                    self._T_map_world = T
            except ValueError:
                pass
        T_map_world = self._current_T_map_world()
        if T_map_world is not None:
            self.tf_pub.sendTransform(_matrix_to_tf(
                invert(T_map_world), self.map_frame, self._world, msg.header.stamp))
            pts_map = apply(T_map_world, xyz)
            if self.lidar_scan_pub.get_subscription_count():
                self.lidar_scan_pub.publish(xyzi_to_msg(pts_map, inten, self.map_frame,
                                                        msg.header.stamp))
            if self.recording:
                self.rec_scans += 1
                self.full.add(pts_map, inten)
                self.full_display.add(pts_map, inten)
                T_wl = self._lookup(self._world, self.lidar_frame)
                if T_wl is not None:
                    pos = apply(T_map_world, T_wl[:3, 3])[0]
                    self.path.append((stamp_to_sec(msg.header.stamp), *pos))
                self._maybe_publish_maps()

        if self.cam is None:
            self.stats['no_camera_info'] += 1
            return
        t_scan = stamp_to_sec(msg.header.stamp) + self.scan_mid_offset
        self._pending.append((t_scan, time.monotonic(), self._world, msg.header.stamp, xyz))
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
            _, _, frame, stamp, xyz = self._pending.popleft()
            self._colorize(t_scan, frame, stamp, xyz)

    # --- the colour pipeline --------------------------------------------

    def _colorize(self, t_scan: float, frame: str, stamp, pts_world: np.ndarray):
        if not self._images:
            self.stats['no_image'] += 1
            return
        t_img, img_msg = min(self._images, key=lambda e: abs(e[0] - t_scan))
        self._gaps.append(t_img - t_scan)
        if abs(t_img - t_scan) > self.max_dt:
            self.stats['dt_too_big'] += 1
            return

        T_cam_world = self._lookup(self.camera_frame, frame, t_img)
        if T_cam_world is None:
            self.stats['no_tf'] += 1
            self.get_logger().warn('no TF {} -> {}'.format(frame, self.camera_frame),
                                   throttle_duration_sec=5.0)
            return
        if frame != self.lidar_frame:                 # moving rig: skip fast turns
            # over the 0.1 s up to the image: poses that already exist
            R0 = self._lookup(frame, self.lidar_frame, t_img - 0.1, fallback_latest=False)
            R1 = self._lookup(frame, self.lidar_frame, t_img, fallback_latest=False)
            if R0 is not None and R1 is not None and \
                    rotation_angle(R0[:3, :3], R1[:3, :3]) / 0.1 > self.max_ang_rate:
                self.stats['too_fast'] += 1
                return

        pts_cam = apply(T_cam_world, pts_world)
        depth = pts_cam[:, 2]
        near = (depth > self.min_range) & (depth < self.max_range)
        pts_cam, pts_world = pts_cam[near], pts_world[near]
        uv, valid = project(self.cam, pts_cam)
        uv, pts_cam, pts_world = uv[valid], pts_cam[valid], pts_world[valid]
        keep = occlusion_mask(uv, pts_cam[:, 2], (self.cam.height, self.cam.width),
                              cell=self.occl_cell, tol=self.occl_tol)
        uv, pts_cam, pts_world = uv[keep], pts_cam[keep], pts_world[keep]
        if pts_world.shape[0] == 0:
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

        T_map_world = self._current_T_map_world()
        if T_map_world is None:
            return
        pts_map = apply(T_map_world, pts_world)
        self.scan_pub.publish(xyzrgb_to_msg(pts_map, rgb, self.map_frame, stamp))
        if self.recording:
            quality = view_quality(pts_cam, uv, self.cam, max_range=self.max_range)
            self.acc.add(pts_map, rgb, quality)

    def _maybe_publish_maps(self):
        """Republish the maps at their own rates; built only if someone listens."""
        now = time.monotonic()
        stamp = None
        if now - self._last_map_pub >= self.publish_period:
            self._last_map_pub = now
            stamp = self.get_clock().now().to_msg()
            xyz, rgb = self.acc.export()
            self._counts = (xyz.shape[0], self._counts[1])
            if self.map_pub.get_subscription_count():
                self.map_pub.publish(xyzrgb_to_msg(xyz, rgb, self.map_frame, stamp))
        if now - self._last_full_pub >= self.full_publish_period:
            self._last_full_pub = now
            stamp = stamp or self.get_clock().now().to_msg()
            self._counts = (self._counts[0], self.full.count)
            if self.full_pub.get_subscription_count():
                xyz, inten = self.full_display.export()
                self.full_pub.publish(xyzi_to_msg(xyz, inten, self.map_frame, stamp))

    # --- reporting ------------------------------------------------------

    def _pose(self):
        """Lidar x, y, z, yaw in the map frame (for the viewer), or None."""
        T = self._current_T_map_world()
        if T is None or self._world is None:
            return None
        T_wl = self._lookup(self._world, self.lidar_frame)
        if T_wl is None:
            return None
        M = T @ T_wl
        return dict(x=round(float(M[0, 3]), 3), y=round(float(M[1, 3]), 3),
                    z=round(float(M[2, 3]), 3),
                    yaw=round(float(np.arctan2(M[1, 0], M[0, 0])), 3))

    def turn_rate(self, window: float = 0.25) -> float:
        """Current rotation rate in deg/s, from the IMU gyro (all axes)."""
        if not self._imu:
            return 0.0
        t_end = self._imu[-1][0]
        w = [e[2] for e in self._imu if e[0] >= t_end - window]
        return float(np.degrees(np.linalg.norm(np.mean(w, axis=0)))) if w else 0.0

    def status(self) -> dict:
        acc, gyro = self._imu_window()
        still = gyro is not None and len(gyro) > 0 and \
            float(np.linalg.norm(gyro, axis=1).mean()) < self.static_gyro
        path = thin_path(np.array([r[1:] for r in self.path])) \
            if self.recording and self.path else np.zeros((0, 3))
        path = path[::max(1, len(path) // 500)]
        return dict(
            recording=self.recording, map_name=self.map_name,
            elapsed_s=round(time.time() - self.rec_started, 1) if self.recording else 0.0,
            colored_points=self._counts[0], full_points=self._counts[1],
            rec_scans=self.rec_scans, scans=self.stats['scans'],
            coloured=self.stats['coloured'], too_fast=self.stats['too_fast'],
            rec_too_fast=self.stats['too_fast'] - getattr(self, '_too_fast_at_start', 0)
            if self.recording else 0,
            turn_rate_dps=round(self.turn_rate(), 1),
            max_turn_rate_dps=round(float(np.degrees(self.max_ang_rate)), 1),
            camera_ok=self.cam is not None,
            odometry_ok=self._world is not None and
            self._lookup(self._world, self.lidar_frame) is not None,
            imu_ok=acc is not None and len(acc) > 20, still=bool(still),
            lidar_height=float(self.get_parameter('lidar_height').value),
            default_lidar_height=self.default_height,
            rec_height=self.rec_height if self.recording else None,
            pose=self._pose(), path=[[round(float(x), 3), round(float(y), 3)] for x, y, _ in path],
            exports=self.exports, last_saved=self.last_saved, output_dir=self.output_dir)

    def _publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.status())))

    def _log_progress(self):
        s = self.stats
        gap = ''
        if self._gaps:
            g = np.asarray(self._gaps)
            gap = ' | gap(img-scan) median {:+.3f}s p95|.| {:.3f}s'.format(
                float(np.median(g)), float(np.percentile(np.abs(g), 95)))
        rec = ' | REC {!r} colored={} full={} path={}'.format(
            self.map_name, *self._counts, len(self.path)) if self.recording else ''
        self.get_logger().info(
            'scans={} coloured={} img_rx={} imu_rx={} | drops: no_info={} no_tf={} '
            'no_img={} dt={} fast={} empty={}{}{}'.format(
                s['scans'], s['coloured'], s['images_rx'], s['imu_rx'], s['no_camera_info'],
                s['no_tf'], s['no_image'], s['dt_too_big'], s['too_fast'], s['empty'], gap, rec))


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
