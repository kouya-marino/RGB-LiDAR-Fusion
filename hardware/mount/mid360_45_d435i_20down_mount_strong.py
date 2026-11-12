"""Parametric mount: Livox Mid-360S (scan axis 45 deg up) + RealSense D435i (20 deg down).
Frame: X = forward (away from wall), Y = sideways, Z = up. Units: mm.
Run (from this folder): python3 mid360_45_d435i_20down_mount_strong.py > extrinsics.txt
  -> writes mid360_45_d435i_20down_mount_strong.stl, prints the design extrinsics,
     and prints clearance checks to stderr.
Deps: pip install manifold3d trimesh numpy scipy

v3: closed side walls, a window in the back plate with 45 deg channels so the two
uphill lidar screws go in from behind, and a 1/4"-20 socket on the bottom for a
camera stick / tripod. Sensor positions are unchanged from v2, so the extrinsics
are too.
"""
import math
import sys
import numpy as np
from manifold3d import Manifold, CrossSection
import trimesh

# ---------------- parameters ----------------
LIDAR_AXIS_DEG = 45          # lidar axis angle above horizontal
PLATE_TILT = 90 - LIDAR_AXIS_DEG   # plate angle from horizontal (45); lidar-to-camera pitch = PLATE_TILT - CAM_TILT = 25
T = 7.0                      # general wall thickness
RIB = 6.0                    # rib thickness
WALL = 7.0                   # side wall thickness
LIDAR_ORIGIN_H = 0.047       # Mid-360S point-cloud origin O above its base [m] (manual: 47.0 mm)
W = 100.0                    # back plate width (Y)
H = 120.0                    # back plate height (Z)
PLATE = 66.0                 # lidar plate length along the slope (Mid-360S base is 65 x 65)
PLATE_W = W                  # lidar plate width: full width, so the side walls fuse to it
M3 = 3.4                     # M3 clearance
# Mid-360S bottom (user manual v1.0 2026.04, Dimensions; matches Livox's 3D model):
# 4x M3 (5 mm deep) on a 48 x 36 mm rectangle centred on the body; 48 mm along
# the lidar X axis.
HOLES_XY = (48.0, 36.0)
# Locating holes (dia 3.0, 1.8 deep) on the X centreline: a round one 16 mm and a
# slot 23 mm from centre, on opposite sides along Y. Pin holes go at both +-16 and
# +-23, so pins fit either way round. Sized for 3 mm dowel pins.
PIN_D = 3.0
PIN_Y = (16.0, 23.0)
CAM_TILT = 20               # camera pitch down (deg); 0 = straight
SHELF_LEN = 75.0             # camera shelf length (X)
CAM_X = 57.5                 # camera 1/4"-20 screw position (X)
QUARTER = 6.6                # 1/4" clearance
M5 = 5.5                     # wall screws
SEG = 48
# Back window: opens the cavity under the lidar plate. The uphill M3 screw axes
# leave the back plate at z ~ 100 mm, so the window reaches up to there and a
# channel along each axis runs from the window to the hole.
WIN_Y = 25.0                 # window half width (the uphill M3 holes are at y = +-18)
WIN_Z = (62.0, 104.0)        # window bottom / top
CHANNEL_D = 7.0              # screw head (5.5) + hex key clearance
SIDE_WALL_FOOT = 40.0        # walls are cut back to here near the shelf: camera + USB-C plug clearance
# Camera-stick / tripod / monopod socket: a 1/4"-20 hex nut (7/16" = 11.1 mm
# across flats, 5.6 mm thick) dropped into a pocket from above, in a boss; the
# stick's screw comes up through the bottom. It sits behind the camera screw so
# a stick's head plate (up to ~40 mm) leaves that screw free.
STICK_X = 28.0
STICK_BOSS_D, STICK_BOSS_TOP = 22.0, 16.0
NUT_AF, NUT_POCKET = 11.4, 6.2


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl(d, h, axis="z"):
    c = Manifold.cylinder(h, d / 2, d / 2, SEG).translate([0, 0, -h / 2])
    if axis == "x":
        c = c.rotate([0, 90, 0])
    elif axis == "y":
        c = c.rotate([90, 0, 0])
    return c


def xz_prism(pts, y0, y1):
    """Extrude a polygon given in (x, z) along Y from y0 to y1."""
    cs = CrossSection([[(x, z) for x, z in pts]])
    m = Manifold.extrude(cs, y1 - y0)          # polygon in XY, extruded along Z
    return m.rotate([90, 0, 0]).translate([0, y1, 0])  # XY->XZ, extrusion -> -Y


ta = math.radians(PLATE_TILT)
s_, c_ = math.sin(ta), math.cos(ta)


def plate_pt(px, py, pz):
    """Plate-local point (x downhill, y, z = plate normal; origin at the top rear
    edge) -> mount coordinates."""
    return np.array([T + px * c_ + pz * s_, py, H - px * s_ + pz * c_])


# ---------------- back plate with window ----------------
back = box(0, T, -W / 2, W / 2, 0, H)
holes = [cyl(M5, 20, "x").translate([T / 2, y, z])
         for y, z in [(-22, 50), (22, 50), (0, 20)]]
slots = [box(-1, T + 1, y - 2, y + 2, zc - 6, zc + 6)   # hose-clamp slots for pole mounting
         for zc in (39, 53) for y in (-41, 41)]
window = box(-1, T + 1, -WIN_Y, WIN_Y, WIN_Z[0], WIN_Z[1])
for h in holes + slots + [window]:
    back = back - h

# ---------------- lidar plate (tilted 45 deg) ----------------
plate = box(0, PLATE, -PLATE_W / 2, PLATE_W / 2, -T, 0)
c = PLATE / 2
# plate-local +x runs downhill = lidar +X (connector on -X faces uphill)
for sx in (-1, 1):
    for sy in (-1, 1):
        plate = plate - cyl(M3, 20).translate([c + sx * HOLES_XY[0] / 2, sy * HOLES_XY[1] / 2, 0])
for y in PIN_Y:
    for sy in (-1, 1):
        plate = plate - cyl(PIN_D, 20).translate([c, sy * y, 0])
plate = plate.rotate([0, PLATE_TILT, 0]).translate([T, 0, H])

# ---------------- ribs under the plate ----------------
# rib top edge runs along the plate's mid-thickness so rib and plate fuse solidly
Px, Pz = T - (T / 2) * s_, H - (T / 2) * c_          # mid-plane at plate rear
zmid = lambda x: Pz - (x - Px) * math.tan(ta)
fx = Px + PLATE * c_ - 3                              # stop 3 mm before plate front
rib_pts = [(0, 60), (fx, zmid(fx)), (T, zmid(T)), (0, H)]   # counter-clockwise
ribs = [xz_prism(rib_pts, y, y + RIB) for y in (-33.0, 33.0 - RIB)]

# ---------------- side walls ----------------
# Full height at both outer edges, from the back plate to the plate's front edge.
# The top edge runs inside the plate (mid-thickness) so the two fuse without
# coincident faces; near the shelf the wall is cut back to SIDE_WALL_FOOT.
mid_front = plate_pt(PLATE - 1, 0, -T / 2)[[0, 2]]
mid_back = plate_pt(0.5, 0, -T / 2)[[0, 2]]
wall_pts = [(0, 0), (SIDE_WALL_FOOT, 0), (SIDE_WALL_FOOT, 30),
            tuple(mid_front), tuple(mid_back), (0, mid_back[1])]
walls = [xz_prism(wall_pts, y, y + WALL) for y in (-W / 2, W / 2 - WALL)]

# ---------------- camera shelf ----------------
shelf = box(0, SHELF_LEN, -W / 2, W / 2, 0, T)
if CAM_TILT:
    # wedge whose top face tilts CAM_TILT deg nose-down; camera bolts on it
    tt = math.tan(math.radians(CAM_TILT))
    x0, x1, hf = CAM_X - 17.5, CAM_X + 17.5, T + 3.0
    hb = hf + (x1 - x0) * tt
    shelf = shelf + xz_prism([(x0, T - 0.5), (x1, T - 0.5), (x1, hf), (x0, hb)], -30, 30)
    ztop = hb - (CAM_X - x0) * tt
    shelf = shelf - cyl(QUARTER, 60).rotate([0, CAM_TILT, 0]).translate([CAM_X, 0, ztop])
else:
    ztop = T
    shelf = shelf - cyl(QUARTER, 20).translate([CAM_X, 0, 0])

# ---------------- camera-stick socket ----------------
shelf = shelf + Manifold.cylinder(STICK_BOSS_TOP, STICK_BOSS_D / 2, STICK_BOSS_D / 2, SEG).translate([STICK_X, 0, 0])
nut = Manifold.cylinder(NUT_POCKET + 1, NUT_AF / math.sqrt(3), NUT_AF / math.sqrt(3), 6).rotate([0, 0, 30])
shelf = shelf - nut.translate([STICK_X, 0, STICK_BOSS_TOP - NUT_POCKET])
shelf = shelf - Manifold.cylinder(STICK_BOSS_TOP + 2, QUARTER / 2, QUARTER / 2, SEG).translate([STICK_X, 0, -1])

mount = back + plate + shelf
for r in ribs + walls:
    mount = mount + r

# ---------------- channels to the uphill M3 holes ----------------
# Along each screw axis (45 deg down-back) from the plate underside out through
# the back plate. They start 0.3 mm into the underside (a shallow spot face).
for sy in (-1, 1):
    start = plate_pt(c - HOLES_XY[0] / 2, sy * HOLES_XY[1] / 2, -T + 0.3)
    ch = Manifold.cylinder(60, CHANNEL_D / 2, CHANNEL_D / 2, SEG).rotate([0, 180 + PLATE_TILT, 0])
    mount = mount - ch.translate(start.tolist())


def to_trimesh(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts))


tm = to_trimesh(mount)
tm.export("mid360_45_d435i_20down_mount_strong.stl")
print("watertight:", tm.is_watertight, "bounds:", tm.bounds.round(1).tolist(),
      "volume cm3:", round(tm.volume / 1000, 1))

# ---------------- clearance checks (stderr) ----------------
# Simple stand-ins: lidar body 65 x 65 x 65, camera 90 x 25 x 25 on its screw,
# USB-C plug + strain relief 30 mm out of either camera end, M3 socket heads
# under all four holes, and a 3 mm hex key along the uphill screw axes.
lidar_box = box(c - 32.5, c + 32.5, -32.5, 32.5, 0, 65).rotate([0, PLATE_TILT, 0]).translate([T, 0, H])
cam_box = box(-10.1, 14.9, -45, 45, 0, 25).rotate([0, CAM_TILT, 0]).translate([CAM_X, 0, ztop])
plugs = (box(-6, 6, 45, 75, 6, 18) + box(-6, 6, -75, -45, 6, 18)).rotate([0, CAM_TILT, 0]).translate([CAM_X, 0, ztop])
checks = [("lidar body", lidar_box), ("camera body", cam_box), ("camera USB plugs", plugs)]
for sx in (-1, 1):
    for sy in (-1, 1):
        p = plate_pt(c + sx * HOLES_XY[0] / 2, sy * HOLES_XY[1] / 2, -T)
        head = Manifold.cylinder(3, 2.75, 2.75, SEG).rotate([0, 180 + PLATE_TILT, 0]).translate(p.tolist())
        checks.append((f"M3 head {'uphill' if sx < 0 else 'downhill'} y={sy * 18:+.0f}", head))
        if sx < 0:
            q = p + 3 * np.array([-s_, 0, -c_])
            key = Manifold.cylinder(60, 1.5, 1.5, SEG).rotate([0, 180 + PLATE_TILT, 0]).translate(q.tolist())
            checks.append((f"hex key to y={sy * 18:+.0f}", key))
for name, part in checks:
    v = (mount ^ part).volume()
    print(f"clearance {name:22s}: {'ok' if v < 0.01 else f'CLASH {v:.1f} mm3'}", file=sys.stderr)


# ---------------- extrinsics (from CAD geometry) ----------------
from scipy.spatial.transform import Rotation as Rot


def rot_y(deg):
    return Rot.from_euler("y", deg, degrees=True).as_matrix()


def make_T(R, p):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = p; return M


# Lidar: X = downhill along plate (away from wall), Y = left, Z = scan axis
R_ML = rot_y(PLATE_TILT)
base_c = np.array([T, 0, H]) + R_ML @ np.array([PLATE / 2, 0, 0])
p_ML = base_c / 1000 + R_ML @ np.array([0, 0, LIDAR_ORIGIN_H])
# Camera camera_link (realsense-ros): X fwd, Y left, Z up, origin at left IR imager
R_MC = rot_y(CAM_TILT)
screw = np.array([CAM_X, 0, ztop]) / 1000
p_MC = screw + R_MC @ np.array([0.0149, 0.0175, 0.0125])   # D435i URDF offsets

T_ML, T_MC = make_T(R_ML, p_ML), make_T(R_MC, p_MC)
T_LC = np.linalg.inv(T_ML) @ T_MC            # camera_link expressed in lidar frame
R_link_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])  # optical: Z fwd, X right, Y down
T_LCo = T_LC @ make_T(R_link_opt, np.zeros(3))


def report(name, M):
    q = Rot.from_matrix(M[:3, :3]).as_quat()
    rpy = Rot.from_matrix(M[:3, :3]).as_euler("xyz", degrees=True)
    print(f"{name}\n  t [m]   = {np.round(M[:3, 3], 4).tolist()}\n"
          f"  q xyzw  = {np.round(q, 4).tolist()}\n  rpy deg = {np.round(rpy, 2).tolist()}\n"
          f"  matrix  =\n{np.round(M, 4)}\n")


report("T_lidar_cameralink (camera_link in lidar frame)", T_LC)
report("T_cameralink_lidar (lidar in camera_link frame)", np.linalg.inv(T_LC))
report("T_lidar_cameraoptical (optical frame in lidar frame)", T_LCo)
report("T_cameraoptical_lidar (lidar in optical frame)", np.linalg.inv(T_LCo))
