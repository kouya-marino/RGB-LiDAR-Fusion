"""Parametric mount: Livox Mid-360S (scan axis 45 deg up) + RealSense D435i (20 deg down).
Frame: X = forward (away from wall), Y = sideways, Z = up. Units: mm.
Run (from this folder): python3 mid360_45_d435i_20down_mount_strong.py > extrinsics.txt
  -> writes mid360_45_d435i_20down_mount_strong.stl and prints the design extrinsics.
Deps: pip install manifold3d trimesh numpy scipy
"""
import math
import numpy as np
from manifold3d import Manifold, CrossSection
import trimesh

# ---------------- parameters ----------------
LIDAR_AXIS_DEG = 45          # lidar axis angle above horizontal
PLATE_TILT = 90 - LIDAR_AXIS_DEG   # plate angle from horizontal (45); lidar-to-camera pitch = PLATE_TILT - CAM_TILT = 25
T = 7.0                      # general wall thickness (was 5)
RIB = 6.0                    # rib / gusset thickness (was 4)
LIDAR_ORIGIN_H = 0.047       # Mid-360S point-cloud origin O above its base [m] (manual: 47.0 mm)
W = 100.0                    # back plate width (Y)
H = 120.0                    # back plate height (Z)
PLATE = 66.0                 # lidar plate size (Mid-360S base is 65 x 65)
M3 = 3.4                     # M3 clearance
# Mid-360S bottom (user manual v1.0 2026.04, Dimensions): 4x M3 (5 mm deep) on a
# 48 x 36 mm rectangle centred on the body; 48 mm along the lidar X axis.
HOLES_XY = (48.0, 36.0)
# Locating holes (dia 3.0, 1.8 deep) on the X centreline: a round one 16 mm and a
# slot 23 mm from centre, on opposite sides along Y. The drawing leaves the side
# ambiguous, so pin holes go at both +-16 and +-23: pins fit whichever pair
# matches. Sized for 3 mm dowel pins (press fit; printers shrink holes slightly).
PIN_D = 3.0
PIN_Y = (16.0, 23.0)
CAM_TILT = 20               # camera pitch down (deg); 0 = straight
SHELF_LEN = 75.0             # camera shelf length (X)
CAM_X = 57.5                 # camera 1/4"-20 screw position (X)
QUARTER = 6.6                # 1/4" clearance
M5 = 5.5                     # wall screws
SEG = 48


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


# ---------------- back plate ----------------
back = box(0, T, -W / 2, W / 2, 0, H)
holes = [cyl(M5, 20, "x").translate([T / 2, y, z])
         for y, z in [(-22, 66), (22, 66), (0, 20)]]
slots = []
for zc in (39, 53):                     # hose-clamp slots for pole mounting
    for y in (-42, 42):
        slots.append(box(-1, T + 1, y - 2, y + 2, zc - 6, zc + 6))
for h in holes + slots:
    back = back - h

# ---------------- lidar plate (tilted 45 deg) ----------------
plate = box(0, PLATE, -PLATE / 2, PLATE / 2, -T, 0)
c = PLATE / 2
# plate-local +x runs downhill = lidar +X (connector on -X faces uphill)
for sx in (-1, 1):
    for sy in (-1, 1):
        plate = plate - cyl(M3, 20).translate([c + sx * HOLES_XY[0] / 2, sy * HOLES_XY[1] / 2, 0])
for y in PIN_Y:
    for sy in (-1, 1):
        plate = plate - cyl(PIN_D, 20).translate([c, sy * y, 0])
plate = plate.rotate([0, PLATE_TILT, 0]).translate([T, 0, H])

# ---------------- side ribs under plate ----------------
ta = math.radians(PLATE_TILT)
# rib top edge runs along the plate's mid-thickness so rib and plate fuse solidly
s_, c_ = math.sin(ta), math.cos(ta)
Px, Pz = T - (T / 2) * s_, H - (T / 2) * c_          # mid-plane at plate rear
zmid = lambda x: Pz - (x - Px) * math.tan(ta)
fx = Px + PLATE * c_ - 3                              # stop 3 mm before plate front
rib_pts = [(0, 60), (fx, zmid(fx)), (T, zmid(T)), (0, H)]   # counter-clockwise
ribs = [xz_prism(rib_pts, y, y + RIB) for y in (-PLATE / 2, PLATE / 2 - RIB)]

# ---------------- camera shelf ----------------
shelf = box(0, SHELF_LEN, -W / 2, W / 2, 0, T)
if CAM_TILT:
    # wedge whose top face tilts CAM_TILT deg nose-down; camera bolts on it
    tt = math.tan(math.radians(CAM_TILT))
    x0, x1, hf = CAM_X - 17.5, CAM_X + 17.5, T + 3.0
    hb = hf + (x1 - x0) * tt
    shelf = shelf + xz_prism([(x0, T - 0.5), (x1, T - 0.5), (x1, hf), (x0, hb)], -30, 30)
    ztop = hb - (CAM_X - x0) * tt
    hole = cyl(QUARTER, 60).rotate([0, CAM_TILT, 0]).translate([CAM_X, 0, ztop])
    shelf = shelf - hole
else:
    shelf = shelf - cyl(QUARTER, 20).translate([CAM_X, 0, 0])
gus_pts = [(0, T), (40, T), (0, 40)]
gussets = [xz_prism(gus_pts, y, y + RIB) for y in (-W / 2, W / 2 - RIB)]

mount = back + plate + shelf
for r in ribs + gussets:
    mount = mount + r

mesh = mount.to_mesh()
tm = trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                     faces=np.asarray(mesh.tri_verts))
tm.export("mid360_45_d435i_20down_mount_strong.stl")
print("watertight:", tm.is_watertight, "bounds:", tm.bounds.round(1).tolist(),
      "volume cm3:", round(tm.volume / 1000, 1))


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
screw = np.array([CAM_X, 0, ztop if CAM_TILT else T]) / 1000
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
