"""Browser UI for live colored mapping: a tiny HTTP server inside a ROS node.

Serves one static page (Three.js, vendored) plus a small JSON/binary API:

  GET  /api/status                     colorizer status (recording, counts, ...)
  GET  /api/live/{scan,lidar,colored,full}.bin
                                       latest coloured scan / full lidar scan /
                                       growing coloured map / growing full map
                                       (all in the levelled map frame)
  GET  /api/maps                       saved maps (from each folder's info.json)
  GET  /api/maps/<name>/{colored,full}.bin
  GET  /api/maps/<name>/files/[<robot>/]<file>
                                       download colored|full .pcd|.ply (map frame
                                       or a robot export), info.json, trajectory.csv
  POST /api/start  {"name": "...", "lidar_height": 0.715, "overwrite": false}
                                       start recording a map (overwrite: replace
                                       an existing map of that name when saved)
  POST /api/stop                       stop and save
  POST /api/cancel                     stop and discard

Binary clouds: uint32 N, float32 xyz[3N], then uint8 rgb[3N] (coloured) or
float32 intensity[N] (lidar). Live endpoints send an ETag so an unchanged
cloud costs a 304, not a download. Messages are kept as they arrive and
encoded only when a browser asks, so an idle page costs nothing.

Only the standard library's http.server is used: no web framework to install.
"""
import json
import os
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rgb_lidar_fusion.colorizer_node import valid_map_name
from rgb_lidar_fusion.io_cloud import encode_xyzi, encode_xyzrgb, read_pcd, unpack_rgb
from rgb_lidar_fusion.ros_conv import pointcloud2_field, pointcloud2_to_array, pointcloud2_to_xyz

CONTENT_TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
                 '.css': 'text/css; charset=utf-8', '.json': 'application/json',
                 '.pcd': 'application/octet-stream', '.ply': 'application/octet-stream',
                 '.png': 'image/png', '.svg': 'image/svg+xml'}
CLOUD_FILES = ('colored.pcd', 'colored.ply', 'full.pcd', 'full.ply')
MAP_FILES = CLOUD_FILES + ('info.json', 'trajectory.csv')


def encode_colored_msg(msg: PointCloud2) -> bytes:
    """Our XYZRGB PointCloud2 (colored_scan / colored_map) -> web binary."""
    arr = pointcloud2_to_array(msg)
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1)
    return encode_xyzrgb(xyz, unpack_rgb(arr['rgb']))


def encode_lidar_msg(msg: PointCloud2, min_range: float = 0.0) -> bytes:
    """Any lidar PointCloud2 -> web binary with intensity. `min_range` > 0 drops
    points near the frame origin (raw scans: the sensor's no-returns); the
    map-frame clouds the colorizer publishes need no filtering."""
    xyz = pointcloud2_to_xyz(msg)
    inten = pointcloud2_field(msg, 'intensity')
    if min_range > 0:
        keep = np.linalg.norm(xyz, axis=1) > min_range
        xyz, inten = xyz[keep], inten[keep]
    return encode_xyzi(xyz, inten)


def encode_saved(path: str) -> bytes:
    """A saved colored.pcd (x y z rgb) or full.pcd (x y z intensity) -> web binary."""
    rec = read_pcd(path)
    xyz = np.stack([rec['x'], rec['y'], rec['z']], axis=1)
    if 'rgb' in rec.dtype.names:
        return encode_xyzrgb(xyz, unpack_rgb(rec['rgb']))
    return encode_xyzi(xyz, rec['intensity'])


class _Latest:
    """Newest message on a topic, encoded lazily and at most once."""

    def __init__(self, encoder):
        self.encoder = encoder
        self.lock = threading.Lock()
        self.msg, self.seq, self._enc_seq, self._enc = None, 0, -1, b''

    def put(self, msg):
        with self.lock:
            self.msg, self.seq = msg, self.seq + 1

    def get(self) -> Tuple[int, bytes]:
        with self.lock:
            msg, seq = self.msg, self.seq
            if msg is None:
                return 0, np.uint32(0).tobytes()
            if seq != self._enc_seq:
                self._enc, self._enc_seq = self.encoder(msg), seq
            return seq, self._enc


class WebUiNode(Node):

    def __init__(self):
        super().__init__('web_ui')
        share = get_package_share_directory('rgb_lidar_fusion')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('output_dir', os.path.join(os.getcwd(), 'output'))
        self.declare_parameter('colorizer', '/colorizer')
        self.web_root = os.path.join(share, 'web')
        self.vendor_root = os.path.join(share, 'vendor')
        self.output_dir = str(self.get_parameter('output_dir').value)
        colorizer = str(self.get_parameter('colorizer').value).rstrip('/')

        self.live: Dict[str, _Latest] = {
            'scan': _Latest(encode_colored_msg),
            'lidar': _Latest(encode_lidar_msg),
            'colored': _Latest(encode_colored_msg),
            'full': _Latest(encode_lidar_msg),
        }
        sub = self.create_subscription
        sub(PointCloud2, '/colored_scan', self.live['scan'].put, 1)
        sub(PointCloud2, '/lidar_scan', self.live['lidar'].put, 1)
        sub(PointCloud2, '/colored_map', self.live['colored'].put, 1)
        sub(PointCloud2, '/lidar_map', self.live['full'].put, 1)
        self._status: Optional[dict] = None
        self._status_t = 0.0
        sub(String, colorizer + '/status', self._on_status, 1)

        self.cli = {k: self.create_client(Trigger, '{}/{}_map'.format(colorizer, k))
                    for k in ('start', 'stop', 'cancel')}
        self.param_cli = self.create_client(SetParameters, colorizer + '/set_parameters')
        self._saved_cache: Dict[str, Tuple[float, bytes]] = {}

        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        handler = type('Handler', (_Handler,), {'node': self})
        self.httpd = ThreadingHTTPServer((host, port), handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info('web UI on http://{}:{}/  (maps in {})'.format(
            'localhost' if host in ('0.0.0.0', '') else host, port, self.output_dir))

    def _on_status(self, msg: String):
        self._status = json.loads(msg.data)
        self._status_t = self.get_clock().now().nanoseconds * 1e-9

    def status(self) -> dict:
        age = self.get_clock().now().nanoseconds * 1e-9 - self._status_t
        st = dict(self._status or {})
        st['connected'] = self._status is not None and age < 3.0
        return st

    # --- calls into the colorizer (from HTTP threads) ---------------------

    def _call(self, client, request, timeout=15.0):
        if not client.wait_for_service(timeout_sec=2.0):
            return False, 'colorizer is not running'
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return False, 'colorizer did not answer'
        return True, future.result()

    def start(self, name: str, lidar_height: float, overwrite: bool = False):
        if not valid_map_name(name):
            return False, 'name must be 1-64 of A-Z a-z 0-9 _ -'
        if not 0.05 <= lidar_height <= 5.0:
            return False, 'lidar height must be 5-500 cm'
        req = SetParameters.Request(parameters=[
            Parameter(name='map_name', value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=name)),
            Parameter(name='lidar_height', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(lidar_height))),
            Parameter(name='overwrite_existing', value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=bool(overwrite)))])
        ok, res = self._call(self.param_cli, req)
        if not ok:
            return False, res
        bad = [r.reason for r in res.results if not r.successful]
        if bad:
            return False, 'colorizer refused the settings: {}'.format('; '.join(bad))
        return self.trigger('start')

    def trigger(self, which: str):
        ok, res = self._call(self.cli[which], Trigger.Request())
        return (res.success, res.message) if ok else (False, res)

    # --- saved maps -----------------------------------------------------

    def list_maps(self):
        maps = []
        if os.path.isdir(self.output_dir):
            for name in os.listdir(self.output_dir):
                info_path = os.path.join(self.output_dir, name, 'info.json')
                if valid_map_name(name) and os.path.isfile(info_path):
                    try:
                        with open(info_path) as fh:
                            info = json.load(fh)
                    except (OSError, ValueError):
                        continue
                    folder = os.path.join(self.output_dir, name)
                    files = [f for f in MAP_FILES if os.path.isfile(os.path.join(folder, f))]
                    for e in info.get('exports', []):
                        files += ['{}/{}'.format(e['name'], f) for f in CLOUD_FILES
                                  if valid_map_name(e.get('name', '')) and
                                  os.path.isfile(os.path.join(folder, e['name'], f))]
                    info['files'] = files
                    maps.append(info)
        return sorted(maps, key=lambda m: m.get('started', ''), reverse=True)

    def saved_bin(self, name: str, which: str) -> Optional[bytes]:
        path = os.path.join(self.output_dir, name, which + '.pcd')
        if not os.path.isfile(path):
            return None
        mtime = os.path.getmtime(path)
        hit = self._saved_cache.get(path)
        if hit is None or hit[0] != mtime:
            hit = (mtime, encode_saved(path))
            self._saved_cache = {path: hit}          # keep one: maps can be large
        return hit[1]


class _Handler(BaseHTTPRequestHandler):
    node: WebUiNode = None
    server_version = 'rgb-lidar-fusion'

    def log_message(self, fmt, *args):            # quiet: no line per request
        pass

    def _send(self, code, body: bytes, ctype='application/octet-stream', headers=None):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-cache')
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(body)

    def _json(self, obj, code=HTTPStatus.OK):
        self._send(code, json.dumps(obj).encode(), 'application/json')

    def _file(self, root: str, rel: str, download: bool = False):
        path = os.path.realpath(os.path.join(root, rel))
        if not path.startswith(os.path.realpath(root) + os.sep) or not os.path.isfile(path):
            return self._send(HTTPStatus.NOT_FOUND, b'not found', 'text/plain')
        with open(path, 'rb') as fh:
            body = fh.read()
        ext = os.path.splitext(path)[1]
        headers = {'Content-Disposition': 'attachment; filename="{}"'.format(
            os.path.basename(path))} if download else None
        self._send(HTTPStatus.OK, body, CONTENT_TYPES.get(ext, 'application/octet-stream'),
                   headers)

    def do_GET(self):  # noqa: N802
        n, path = self.node, self.path.split('?', 1)[0]
        parts = [p for p in path.split('/') if p]
        try:
            if path in ('/', '/index.html'):
                return self._file(n.web_root, 'index.html')
            if parts and parts[0] == 'vendor':
                return self._file(n.vendor_root, '/'.join(parts[1:]))
            if parts[:1] != ['api']:
                return self._file(n.web_root, '/'.join(parts))
            if parts == ['api', 'status']:
                return self._json(n.status())
            if len(parts) == 3 and parts[1] == 'live' and parts[2].endswith('.bin'):
                latest = n.live.get(parts[2][:-4])
                if latest is None:
                    return self._send(HTTPStatus.NOT_FOUND, b'unknown stream', 'text/plain')
                seq, body = latest.get()
                etag = '"{}"'.format(seq)
                if self.headers.get('If-None-Match') == etag:
                    return self._send(HTTPStatus.NOT_MODIFIED, b'', headers={'ETag': etag})
                return self._send(HTTPStatus.OK, body, headers={'ETag': etag})
            if parts == ['api', 'maps']:
                return self._json(n.list_maps())
            if len(parts) >= 4 and parts[1] == 'maps' and valid_map_name(parts[2]):
                name = parts[2]
                if len(parts) == 4 and parts[3] in ('colored.bin', 'full.bin'):
                    body = n.saved_bin(name, parts[3][:-4])
                    if body is None:
                        return self._send(HTTPStatus.NOT_FOUND, b'no such map', 'text/plain')
                    return self._send(HTTPStatus.OK, body)
                if len(parts) == 5 and parts[3] == 'files' and parts[4] in MAP_FILES:
                    return self._file(os.path.join(n.output_dir, name), parts[4], download=True)
                if len(parts) == 6 and parts[3] == 'files' and valid_map_name(parts[4]) \
                        and parts[5] in CLOUD_FILES:     # a robot export
                    return self._file(os.path.join(n.output_dir, name, parts[4]), parts[5],
                                      download=True)
            return self._send(HTTPStatus.NOT_FOUND, b'not found', 'text/plain')
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):  # noqa: N802
        n, path = self.node, self.path.split('?', 1)[0]
        length = int(self.headers.get('Content-Length') or 0)
        try:
            body = json.loads(self.rfile.read(length) or b'{}') if length else {}
        except ValueError:
            return self._json({'ok': False, 'message': 'bad JSON'}, HTTPStatus.BAD_REQUEST)
        if path == '/api/start':
            try:
                height = float(body.get('lidar_height', 0.715))
            except (TypeError, ValueError):
                return self._json({'ok': False, 'message': 'lidar_height must be a number'},
                                  HTTPStatus.BAD_REQUEST)
            ok, msg = n.start(str(body.get('name', '')).strip(), height,
                              body.get('overwrite') is True)
        elif path in ('/api/stop', '/api/cancel'):
            ok, msg = n.trigger(path.rsplit('/', 1)[1])
        else:
            return self._json({'ok': False, 'message': 'not found'}, HTTPStatus.NOT_FOUND)
        self._json({'ok': ok, 'message': msg}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)


def main(args=None):
    rclpy.init(args=args)
    node = WebUiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
