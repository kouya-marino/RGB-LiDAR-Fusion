"""Calibrate the lidar -> camera rotation on the running rig (edge alignment).

    ros2 run rgb_lidar_fusion calibrate                 # capture 20 s, solve, report
    ros2 run rgb_lidar_fusion calibrate --write         # ... and update the config
    ros2 run rgb_lidar_fusion calibrate --from FILE.npz  # re-solve a saved capture

With the stack running (sensors.launch.py or colorize.launch.py), point the rig
at a scene with strong DEPTH edges 2-4 m away (a doorway, furniture in front
of a wall, a pillar) and keep it STILL. The tool:

  1. accumulates lidar sweeps for `--duration` s into one dense cloud in the
     lidar frame (refuses if the IMU says the rig moved), keeping only points
     present in every quarter of the capture: people walking through drop out,
  2. takes `--frames` camera images over the same time and uses their
     per-pixel median, which also removes anything moving through the view,
  3. refines the rotation (calibration.py) starting from the current TF,
  4. saves the capture, before/after overlay images and a report,
  5. with --write: rewrites config/mount_extrinsics.yaml (livox_frame ->
     camera_link) in the source tree; rebuild and restart to use it.

Only the rotation is refined; the translation stays at its CAD value.
"""
import argparse
import json
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np

from rgb_lidar_fusion.calibration import (
    AXES, alignment_score, auto_refine, camera_delta, coverage_mask, default_cell, drop_floor,
    image_edges, median_image, prefilter, render_overlay, static_points, voxel_downsample)
from rgb_lidar_fusion.extrinsics import invert, matrix_to_quat, to_rpy_xyz
from rgb_lidar_fusion.projection import CameraModel

CHUNKS = 4          # time slices for the static-scene filter
SRC_CONFIG = '/ws/src/rgb_lidar_fusion/rgb_lidar_fusion/config/mount_extrinsics.yaml'
DEFAULT_OUT = '/ws/src/rgb_lidar_fusion/output/calibration'
DESIGN = dict(xyz=[0.0842, 0.0175, -0.0669], quat_xyzw=[0.0, -0.2164, 0.0, 0.9763])


# --- capture (ROS) ---------------------------------------------------------

def capture(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
    from tf2_ros import Buffer, TransformException, TransformListener

    from rgb_lidar_fusion.ros_conv import (
        image_to_rgb, pointcloud2_to_xyz, transform_to_matrix)

    rclpy.init()
    node = Node('calibrate_capture')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node, spin_thread=True)
    st = dict(info=None, clouds=[], chunk=[], images=[], gyro=[], acc=[], poses=[],
              frame_cam=None, used=0,
              skipped=0, t0=0.0)

    def lookup(target, source, stamp=None):
        try:
            tf = tf_buffer.lookup_transform(target, source, stamp or Time())
        except TransformException:
            return None
        return transform_to_matrix(tf.transform)

    def on_info(msg):
        st['info'] = msg

    def on_cloud(msg):
        if not st.get('recording'):
            return
        # the pose AT this sweep's stamp; no fallback to the latest pose: a
        # sweep placed with the wrong pose corrupts the whole capture
        T = lookup(args.lidar_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if T is None:
            st['skipped'] += 1
            return
        st['poses'].append(T)
        pts = pointcloud2_to_xyz(msg)
        pts = pts @ T[:3, :3].T + T[:3, 3]
        pts = pts[np.linalg.norm(pts, axis=1) > 0.3]       # no-returns, the rig itself
        st['clouds'].append(pts.astype(np.float32))
        st['chunk'].append(np.full(len(pts), min(CHUNKS - 1, int(
            (time.time() - st['t0']) / args.duration * CHUNKS)), np.uint8))
        st['used'] += 1

    def on_image(msg):
        if st.get('recording'):
            st['last_img'] = msg

    def on_imu(msg):
        if st.get('recording'):
            w, a = msg.angular_velocity, msg.linear_acceleration
            st['gyro'].append(np.linalg.norm([w.x, w.y, w.z]))
            st['acc'].append((a.x, a.y, a.z))

    node.create_subscription(CameraInfo, args.camera_info_topic, on_info, 5)
    node.create_subscription(PointCloud2, args.cloud_topic, on_cloud, 10)
    node.create_subscription(Image, args.image_topic, on_image, QoSProfile(
        depth=2, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(Imu, args.imu_topic, on_imu, 200)

    def spin_for(seconds, until=None):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if until and until():
                return True
        return False

    print('waiting for camera_info, TF and data ...')
    ok = spin_for(15, lambda: st['info'] is not None)
    if not ok:
        sys.exit('no camera_info on {} -- is the stack running?'.format(args.camera_info_topic))
    cam_frame = st['info'].header.frame_id
    T_link_opt, T_lidar_link = None, None
    for _ in range(100):
        T_link_opt = lookup('camera_link', cam_frame)
        T_lidar_link = lookup(args.lidar_frame, 'camera_link')
        if T_link_opt is not None and T_lidar_link is not None:
            break
        spin_for(0.1)
    if T_link_opt is None or T_lidar_link is None:
        sys.exit('no TF {} -> camera_link -> {}'.format(args.lidar_frame, cam_frame))

    print('capturing {:.0f} s: keep the rig STILL, facing depth edges 2-4 m away'.format(
        args.duration))
    st['recording'] = True
    t0 = st['t0'] = time.time()
    next_img = t0 + args.duration / (2 * args.frames)
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() >= next_img and st.get('last_img') is not None:
            st['images'].append(image_to_rgb(st['last_img']).copy())
            next_img += args.duration / args.frames
    st['recording'] = False
    # stop the listener's thread while the context is still valid; left to the
    # garbage collector it shuts down after rclpy and prints a traceback
    tf_listener.executor.shutdown()
    del tf_listener
    node.destroy_node()
    rclpy.shutdown()

    gyro = float(np.mean(st['gyro'])) if st['gyro'] else float('nan')
    print('captured {} sweeps ({} skipped), {} images, mean gyro {:.3f} rad/s'.format(
        st['used'], st['skipped'], len(st['images']), gyro))
    if not st['clouds'] or not st['images']:
        sys.exit('nothing captured -- check {} and {}'.format(args.cloud_topic, args.image_topic))
    if np.isfinite(gyro) and gyro > args.max_gyro:
        sys.exit('the rig moved during capture (mean gyro {:.3f} > {:.3f} rad/s); '
                 'keep it still and try again'.format(gyro, args.max_gyro))
    if st['skipped'] > 0.1 * (st['used'] + st['skipped']):
        sys.exit('{} of {} sweeps had no pose (odometry lost track?); try again'.format(
            st['skipped'], st['used'] + st['skipped']))
    # the rig is still, so odometry must report the same pose throughout; if it
    # does not, it glitched and the sweeps do not line up
    P = np.array(st['poses'])
    drift = float(np.max(np.linalg.norm(P[:, :3, 3] - P[0, :3, 3], axis=1)))
    turn = float(max(np.degrees(np.arccos(np.clip((np.trace(P[0, :3, :3].T @ R) - 1) / 2,
                                                  -1, 1))) for R in P[:, :3, :3]))
    print('odometry while still: drift {:.1f} mm, rotation {:.2f} deg'.format(drift * 1e3, turn))
    if drift > 0.02 or turn > 0.3:
        sys.exit('odometry moved while the rig was still ({:.0f} mm, {:.2f} deg): the sweeps '
                 'do not line up; try again'.format(drift * 1e3, turn))
    pts_all, chunk = np.vstack(st['clouds']), np.concatenate(st['chunk'])
    pts = static_points(pts_all, chunk)
    print('static filter: kept {:.0f}% of points (the rest moved during the capture)'.format(
        100.0 * len(pts) / max(1, len(pts_all))))
    pts = voxel_downsample(pts, args.voxel)
    info = st['info']
    return dict(pts=pts, images=np.stack(st['images']), K=np.array(info.k).reshape(3, 3),
                D=np.array(info.d), width=info.width, height=info.height,
                T_link_opt=T_link_opt, T_lidar_link=T_lidar_link,
                up=np.mean(st['acc'], axis=0) if st['acc'] else np.zeros(3),
                meta=json.dumps(dict(when=datetime.now().astimezone().isoformat(
                    timespec='seconds'), sweeps=st['used'], gyro=gyro,
                    cloud_topic=args.cloud_topic, cam_frame=cam_frame)))


# --- solve (no ROS) --------------------------------------------------------

def solve(cap, args, out_dir):
    cam = CameraModel(cap['K'], cap['D'], int(cap['width']), int(cap['height']))
    T_lidar_link0 = np.asarray(cap['T_lidar_link'])
    T_link_opt = np.asarray(cap['T_link_opt'])
    T_seed = T_lidar_link0 @ T_link_opt                    # optical frame in lidar
    cell = default_cell(cam.width)
    images = [median_image(cap['images'])]          # moving things removed
    edges = [image_edges(img, cam, cell) for img in images]
    pts = np.asarray(cap['pts'])
    if 'up' in cap and np.linalg.norm(cap['up']) > 0 and not args.keep_floor:
        n0 = len(pts)
        pts = drop_floor(pts, cap['up'])                # IMU 'up' is in the lidar frame
        print('floor removed: {:.0f}% of points'.format(100.0 * (1 - len(pts) / max(1, n0))))
    pts = prefilter(pts, cam, invert(T_seed), max_range=args.max_range)
    print('solving: {} points in view, median of {} images, raster cell {} px'.format(
        len(pts), len(cap['images']), cell))

    # compare the same cells for every candidate (see alignment_score)
    mask = coverage_mask(pts, cam, invert(T_seed), cell, args.min_range, args.max_range)

    def score(delta):
        return alignment_score(pts, edges, cam, invert(camera_delta(T_seed, delta)), cell,
                               min_range=args.min_range, max_range=args.max_range,
                               mask=mask)[0]

    delta, s0, s1, hits = auto_refine(score, log=print)
    T_new = camera_delta(T_seed, delta)
    T_lidar_link = T_new @ invert(T_link_opt)
    rot_change = float(np.degrees(np.arccos(np.clip(
        (np.trace(T_lidar_link0[:3, :3].T @ T_lidar_link[:3, :3]) - 1) / 2, -1, 1))))

    os.makedirs(out_dir, exist_ok=True)
    for name, T in (('before', T_seed), ('after', T_new)):
        img = render_overlay(images[0], pts, cam, invert(T), args.min_range, args.max_range,
                             label='{}  score {:.3f}'.format(name, score(
                                 [0, 0, 0]) if name == 'before' else s1))
        cv2.imwrite(os.path.join(out_dir, 'overlay_{}.jpg'.format(name)), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
    rpy0, _ = to_rpy_xyz(T_lidar_link0)
    rpy1, xyz1 = to_rpy_xyz(T_lidar_link)
    report = dict(
        score_before=round(s0, 4), score_after=round(s1, 4),
        camera_axes_correction_deg=dict(zip(AXES, [round(v, 2) for v in delta])),
        rotation_change_deg=round(rot_change, 2), edge_of_search=hits,
        lidar_to_camera_link=dict(xyz=[round(v, 5) for v in xyz1],
                                  quat_xyzw=[round(v, 5) for v in matrix_to_quat(
                                      T_lidar_link[:3, :3])],
                                  rpy_deg=[round(v, 3) for v in rpy1]),
        previous_rpy_deg=[round(v, 3) for v in rpy0], points=len(pts),
        images=len(cap['images']))
    with open(os.path.join(out_dir, 'report.json'), 'w') as fh:
        json.dump(report, fh, indent=2)

    gain = 100.0 * (s1 - s0) / abs(s0) if s0 else float('nan')
    print('\nscore {:.4f} -> {:.4f} ({:+.0f}%)'.format(s0, s1, gain))
    print('correction (camera axes): ' + ', '.join(
        '{} {:+.2f} deg'.format(a, v) for a, v in zip(AXES, delta)))
    print('rotation change: {:.2f} deg'.format(rot_change))
    if hits:
        print('WARNING: optimum at the edge of the search ({}): the true value may be '
              'further out -- check the overlays, re-run'.format(', '.join(hits)))
    print('overlays + report: {}'.format(out_dir))
    return T_lidar_link, report, bool(hits) or not np.isfinite(s1) or s1 <= s0


def write_config(path, T_lidar_link, report):
    q = matrix_to_quat(T_lidar_link[:3, :3])
    xyz = T_lidar_link[:3, 3]
    when = datetime.now().astimezone().isoformat(timespec='seconds')
    text = '''# livox_frame -> camera_link for the reference mount (hardware/mount/).
# CALIBRATED {when} by `ros2 run rgb_lidar_fusion calibrate` (edge alignment,
# rotation only): score {s0} -> {s1}, rotation change {rc} deg.
# Design values (CAD): xyz {dx}, quat_xyzw {dq}
# The RealSense driver adds camera_link -> camera_color_optical_frame itself
# (factory calibrated), so only this one transform is published here.
parent_frame: livox_frame
child_frame: camera_link
xyz: [{x:.5f}, {y:.5f}, {z:.5f}]
quat_xyzw: [{qx:.5f}, {qy:.5f}, {qz:.5f}, {qw:.5f}]
'''.format(when=when, s0=report['score_before'], s1=report['score_after'],
           rc=report['rotation_change_deg'], dx=DESIGN['xyz'], dq=DESIGN['quat_xyzw'],
           x=xyz[0], y=xyz[1], z=xyz[2], qx=q[0], qy=q[1], qz=q[2], qw=q[3])
    with open(path, 'w') as fh:
        fh.write(text)
    print('wrote {} -- rebuild (cd /ws && colcon build) and restart the stack to use it'.format(
        path))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--duration', type=float, default=20.0, help='capture seconds')
    ap.add_argument('--frames', type=int, default=9,
                    help='camera images; their per-pixel median is used')
    ap.add_argument('--cloud-topic', default='/cloud_registered',
                    help='/cloud_registered (odometry on) or /livox/lidar (odometry:=false)')
    ap.add_argument('--image-topic', default='/camera/camera/color/image_raw')
    ap.add_argument('--camera-info-topic', default='/camera/camera/color/camera_info')
    ap.add_argument('--imu-topic', default='/livox/imu')
    ap.add_argument('--lidar-frame', default='livox_frame')
    ap.add_argument('--voxel', type=float, default=0.01, help='m, capture downsampling')
    ap.add_argument('--min-range', type=float, default=0.5)
    ap.add_argument('--max-range', type=float, default=10.0)
    ap.add_argument('--max-gyro', type=float, default=0.03, help='rad/s, "still" limit')
    ap.add_argument('--out', default=DEFAULT_OUT, help='where captures and overlays go')
    ap.add_argument('--from', dest='from_file', help='re-solve a saved capture (.npz)')
    ap.add_argument('--keep-floor', action='store_true',
                    help='score the floor too (by default it is left out)')
    ap.add_argument('--write', action='store_true', help='update mount_extrinsics.yaml')
    ap.add_argument('--force', action='store_true', help='write even if the result looks doubtful')
    ap.add_argument('--config', default=SRC_CONFIG, help='mount_extrinsics.yaml to update')
    args = ap.parse_args(argv)

    if args.from_file:
        cap = dict(np.load(args.from_file, allow_pickle=False))
        out_dir = os.path.splitext(args.from_file)[0]
    else:
        cap = capture(args)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_dir = os.path.join(args.out, stamp)
        os.makedirs(out_dir, exist_ok=True)
        np.savez_compressed(out_dir + '.npz', **cap)
        print('capture saved: {}.npz'.format(out_dir))

    T_lidar_link, report, doubtful = solve(cap, args, out_dir)
    if args.write:
        if doubtful and not args.force:
            sys.exit('not writing: the result looks doubtful (see above); check the '
                     'overlays, then re-run with --force if it is right')
        write_config(args.config, T_lidar_link, report)


if __name__ == '__main__':
    main()
