#!/usr/bin/env python3
"""Find the robot on test_map by matching one live /scan, pick 4 patrol points, draw a PNG.
Publishes nothing. Writes /tmp/plan.json and /tmp/plan.png."""
import json, math, struct, sys, time, zlib
import numpy as np
import rclpy, yaml
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros

MAP = '/home/london/wheelbot/test_map'
meta = yaml.safe_load(open(MAP + '.yaml'))
res = meta['resolution']; ox, oy = meta['origin'][:2]
raw = open(MAP + '.pgm', 'rb').read()
parts, i = [], 0
while len(parts) < 4:  # P5 header (skip comments)
    while raw[i:i+1].isspace(): i += 1
    if raw[i:i+1] == b'#':
        while raw[i:i+1] != b'\n': i += 1
        continue
    j = i
    while not raw[j:j+1].isspace(): j += 1
    parts.append(raw[i:j]); i = j
W, H = int(parts[1]), int(parts[2])
img = np.frombuffer(raw[i+1:i+1+W*H], np.uint8).reshape(H, W)
grid = img[::-1]                    # row 0 = bottom = origin y
occ = grid < 50
free = grid > 250
unknown = ~occ & ~free

def dilate(m):
    d = m.copy()
    d[1:] |= m[:-1]; d[:-1] |= m[1:]; d[:, 1:] |= m[:, :-1]; d[:, :-1] |= m[:, 1:]
    return d

# clearance (cells) from obstacles/unknown, capped
blocked = occ | unknown
clear = np.full(grid.shape, 99, np.int16); cur = blocked.copy(); clear[cur] = 0
for k in range(1, 16):
    nxt = dilate(cur) if k % 2 else (dilate(cur) | np.roll(np.roll(cur, 1, 0), 1, 1) | np.roll(np.roll(cur, -1, 0), -1, 1)
                                     | np.roll(np.roll(cur, 1, 0), -1, 1) | np.roll(np.roll(cur, -1, 0), 1, 1))
    clear[nxt & ~cur] = k; cur = nxt
occ_d = dilate(occ)

# ---- one scan + laser offset
rclpy.init(); node = rclpy.create_node('plan_patrol')
scans = []
node.create_subscription(LaserScan, 'scan', scans.append, qos_profile_sensor_data)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, node)
t0 = time.time(); tfm = None
while time.time() - t0 < 20 and (len(scans) < 3 or tfm is None):
    rclpy.spin_once(node, timeout_sec=0.2)
    if scans and tfm is None:
        try: tfm = buf.lookup_transform('base_footprint', scans[-1].header.frame_id, rclpy.time.Time())
        except Exception: pass
if not scans or tfm is None:
    sys.exit('no scan / tf')
s = scans[-1]
lx, ly = tfm.transform.translation.x, tfm.transform.translation.y
q = tfm.transform.rotation; lyaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
r = np.array(s.ranges); a = s.angle_min + np.arange(len(r)) * s.angle_increment
ok = np.isfinite(r) & (r > max(s.range_min, 0.15)) & (r < min(s.range_max, 11.0))
r, a = r[ok], a[ok]
sub = slice(None, None, max(1, len(r) // 180)); rs, as_ = r[sub], a[sub]

def score(px, py, th, rr, aa, exact=False):
    """px,py: (P,) positions; returns hit fraction per position."""
    ex = lx*math.cos(th) - ly*math.sin(th) + rr*np.cos(th + lyaw + aa)
    ey = lx*math.sin(th) + ly*math.cos(th) + rr*np.sin(th + lyaw + aa)
    cx = ((px[:, None] + ex[None]) - ox) / res; cy = ((py[:, None] + ey[None]) - oy) / res
    cx = cx.astype(np.int32); cy = cy.astype(np.int32)
    inb = (cx >= 0) & (cx < W) & (cy >= 0) & (cy < H)
    cx = np.clip(cx, 0, W-1); cy = np.clip(cy, 0, H-1)
    hd = occ_d[cy, cx] & inb
    if not exact: return hd.mean(1)
    return 0.5 * hd.mean(1) + 0.5 * (occ[cy, cx] & inb).mean(1)

# coarse: every 4th cell with clearance >= 3 cells, 5 deg steps
ys, xs = np.nonzero((clear >= 3))
keep = (ys % 4 == 0) & (xs % 4 == 0); ys, xs = ys[keep], xs[keep]
PX = ox + (xs + 0.5) * res; PY = oy + (ys + 0.5) * res
if len(sys.argv) > 4:  # prior: only search near the last known position (corridors look alike)
    px0, py0, rad = map(float, sys.argv[2:5])
    near = np.hypot(PX - px0, PY - py0) <= rad
    PX, PY = PX[near], PY[near]
cands = []
for th in np.radians(np.arange(-180, 180, 5)):
    sc = score(PX, PY, th, rs, as_)
    for k in np.argsort(sc)[-8:]: cands.append((sc[k], PX[k], PY[k], th))
cands.sort(reverse=True)
# refine top distinct candidates with all beams
best = []
for sc0, x0, y0, th0 in cands[:40]:
    if any(math.hypot(x0-b[1], y0-b[2]) < 0.3 and abs(math.remainder(th0-b[3], 2*math.pi)) < 0.2 for b in best): continue
    gx, gy = np.meshgrid(np.arange(-0.2, 0.21, 0.05), np.arange(-0.2, 0.21, 0.05))
    gx, gy = (x0 + gx).ravel(), (y0 + gy).ravel()
    top = (0, 0, 0, 0)
    for dth in np.radians(np.arange(-6, 6.1, 1.5)):
        sc = score(gx, gy, th0 + dth, r, a, exact=True); k = int(np.argmax(sc))
        if sc[k] > top[0]: top = (float(sc[k]), float(gx[k]), float(gy[k]), float(th0 + dth))
    best.append(top)
    if len(best) >= 6: break
best.sort(reverse=True)
bx, by, bth = best[0][1:]
runner = next((b for b in best[1:] if math.hypot(b[1]-bx, b[2]-by) > 1.0), None)

# ---- patrol points: reachable free space, clearance >= 0.5 m, spread out
rc, rr_ = int((bx-ox)/res), int((by-oy)/res)
ok_drive = clear >= 4
comp = np.zeros_like(ok_drive); comp[rr_, rc] = True
for _ in range(3000):
    n = dilate(comp) & ok_drive
    if n.sum() == comp.sum(): break
    comp = n
pts = []
for need in (10, 9, 8, 7):
    cy_, cx_ = np.nonzero(comp & (clear >= need))
    if len(cx_) > 50: break
P = np.stack([ox + (cx_ + 0.5) * res, oy + (cy_ + 0.5) * res], 1)
d = np.hypot(P[:, 0] - bx, P[:, 1] - by)
MAXD = float(sys.argv[1]) if len(sys.argv) > 1 else 1e9  # keep the test loop close to the robot
if (d <= MAXD).sum() > 20:
    P = P[d <= MAXD]; d = d[d <= MAXD]
chosen = [int(np.argmax(d))]
for _ in range(3):
    dm = np.min(np.stack([np.hypot(P[:, 0]-P[c, 0], P[:, 1]-P[c, 1]) for c in chosen]), 0)
    chosen.append(int(np.argmax(dm)))
pts = P[chosen]; c = pts.mean(0)
pts = pts[np.argsort(np.arctan2(pts[:, 1]-c[1], pts[:, 0]-c[0]))]
pts = [[round(float(x), 3), round(float(y), 3)] for x, y in pts]

out = dict(pose=dict(x=round(bx, 3), y=round(by, 3), yaw_deg=round(math.degrees(bth), 1), score=round(best[0][0], 3)),
           runner_up=None if runner is None else dict(x=round(runner[1], 3), y=round(runner[2], 3),
                                                      yaw_deg=round(math.degrees(runner[3]), 1), score=round(runner[0], 3)),
           points=pts, clearance_m=need * res, map=dict(w=W, h=H, res=res, origin=[ox, oy]))
json.dump(out, open('/tmp/plan.json', 'w')); print(json.dumps(out, indent=1))

# ---- PNG (cropped to known area, 3x)
known = ~unknown; ky, kx = np.nonzero(known)
y0, y1, x0, x1 = max(ky.min()-10, 0), min(ky.max()+10, H-1), max(kx.min()-10, 0), min(kx.max()+10, W-1)
rgb = np.stack([grid]*3, -1).astype(np.uint8)
rgb[comp & (clear >= need)] = (200, 235, 200)
def dot(x, y, col, rad=3):
    cx_, cy_ = int((x-ox)/res), int((y-oy)/res)
    rgb[max(cy_-rad, 0):cy_+rad+1, max(cx_-rad, 0):cx_+rad+1] = col
for k in range(12):  # robot heading line
    dot(bx + k*0.03*math.cos(bth), by + k*0.03*math.sin(bth), (255, 0, 0), 1)
dot(bx, by, (255, 0, 0), 3)
ex = bx + lx*math.cos(bth) - ly*math.sin(bth) + r*np.cos(bth + lyaw + a)
ey = by + lx*math.sin(bth) + ly*math.cos(bth) + r*np.sin(bth + lyaw + a)
for x, y in zip(ex, ey): dot(x, y, (255, 120, 0), 0)
cols = [(0, 0, 255), (0, 160, 255), (160, 0, 255), (255, 0, 200)]
for (x, y), col in zip(pts, cols): dot(x, y, col, 4)
crop = rgb[y0:y1+1, x0:x1+1][::-1]
crop = crop.repeat(3, 0).repeat(3, 1)
h, w = crop.shape[:2]
rawp = b''.join(b'\x00' + crop[i].tobytes() for i in range(h))
def chunk(t, d): return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
open('/tmp/plan.png', 'wb').write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                                  + chunk(b'IDAT', zlib.compress(rawp, 9)) + chunk(b'IEND', b''))
print('png', w, h)
