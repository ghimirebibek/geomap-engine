"""Converts a TUM RGB-D ground-truth trajectory into a geomap-engine fixture.

Camera pose (position + orientation) is real, recorded data from the TUM
RGB-D SLAM Dataset and Benchmark (Sturm et al.), Freiburg1/xyz sequence:
  https://cvg.cit.tum.de/data/datasets/rgbd-dataset
  Licensed CC BY 4.0 (Computer Vision Group, TU Munich).

TUM RGB-D is a SLAM benchmark and has no object-detection ground truth, so
the detections in the output are synthetic: this script places a couple of
fixed points on the assumed floor plane (z=0) and forward-projects them
into each real camera pose to compute a plausible bbox, keeping only the
frames where the point actually falls in view. Confidence and the
projected pixel both get small deterministic jitter (sine-based, not a
stray RNG dependency) so this isn't a noiseless round-trip of the same
pinhole model: the +/-8px jitter approximates a moderate real-time object
detector's localization error, giving EngineConfig's default association
radius (see src/engine.rs) something realistic to be tested against.

Regenerate with:
    curl -o groundtruth.txt \\
      https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz-groundtruth.txt
    python3 scripts/gen_tum_fixture.py groundtruth.txt \\
      tests/fixtures/tum_freiburg1_xyz.json

Caveat: TUM's world frame is whatever the motion-capture rig was
calibrated to, not a verified gravity-aligned floor frame. Its z stays in
a plausible ~1.3-1.8m handheld-camera-height band throughout this
sequence, which is why it's usable here, but that's an empirical
observation about this one sequence, not a guarantee of the dataset's
coordinate convention in general.
"""

import json, math, sys

TF = sys.argv[1]
OUT = sys.argv[2]

FX, FY, CX, CY = 517.3, 516.5, 318.6, 255.3
WIDTH, HEIGHT = 2 * CX, 2 * CY

def parse():
    rows = []
    for line in open(TF):
        if line.startswith('#') or not line.strip():
            continue
        t, x, y, z, qx, qy, qz, qw = map(float, line.split())
        rows.append((t, x, y, z, qx, qy, qz, qw))
    return rows

def conj(q):
    qx, qy, qz, qw = q
    return (-qx, -qy, -qz, qw)

def rotate(v, q):
    qx, qy, qz, qw = q
    qv = (qx, qy, qz)
    def cross(a, b):
        return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    c1 = cross(qv, v)
    c2 = cross(qv, c1)
    return (
        v[0] + 2*qw*c1[0] + 2*c2[0],
        v[1] + 2*qw*c1[1] + 2*c2[1],
        v[2] + 2*qw*c1[2] + 2*c2[2],
    )

def project(cam_pos, q, obj_pos):
    """Forward pinhole projection: world point -> (px, py, depth) in camera frame, or None if behind camera."""
    d_world = tuple(obj_pos[i] - cam_pos[i] for i in range(3))
    d_cam = rotate(d_world, conj(q))
    if d_cam[2] <= 0.05:
        return None
    px = FX * d_cam[0] / d_cam[2] + CX
    py = FY * d_cam[1] / d_cam[2] + CY
    if not (0 <= px <= WIDTH and 0 <= py <= HEIGHT):
        return None
    return px, py, d_cam[2]

rows = parse()
print(f"parsed {len(rows)} rows", file=sys.stderr)

# Downsample to ~1 frame per 0.5s of real time.
step = max(1, len(rows) // 60)
sampled = rows[::step]
print(f"sampled {len(sampled)} frames (step={step})", file=sys.stderr)

# Grid-search candidate floor-plane (z=0) object positions near the
# trajectory's xy footprint, keep the ones visible in the most frames.
xs = [r[1] for r in rows]
ys = [r[2] for r in rows]
cx_mid, cy_mid = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2

candidates = []
for dx in [i * 0.5 for i in range(-6, 7)]:
    for dy in [i * 0.5 for i in range(-6, 7)]:
        ox, oy = cx_mid + dx, cy_mid + dy
        visible = 0
        for (t, x, y, z, qx, qy, qz, qw) in sampled:
            if project((x, y, z), (qx, qy, qz, qw), (ox, oy, 0.0)):
                visible += 1
        candidates.append((visible, ox, oy))

candidates.sort(reverse=True)
print("top candidates (visible_count, x, y):", candidates[:5], file=sys.stderr)

chair_pos = (candidates[0][1], candidates[0][2], 0.0)
# Second object: best candidate at least 1m away from the first, so it's spatially distinct.
door_pos = None
for visible, ox, oy in candidates:
    if math.hypot(ox - chair_pos[0], oy - chair_pos[1]) >= 1.0:
        door_pos = (ox, oy, 0.0)
        door_visible = visible
        break

print(f"chair at {chair_pos}, visible in {candidates[0][0]}/{len(sampled)} sampled frames", file=sys.stderr)
if door_pos:
    print(f"door at {door_pos}, visible in {door_visible}/{len(sampled)} sampled frames", file=sys.stderr)
else:
    print("no second object found >=1m from the first", file=sys.stderr)

# Approximate localization error (pixels) for a moderate real-time object
# detector; see the docstring above and src/engine.rs's EngineConfig for
# how this feeds into choosing the association radius.
DETECTOR_NOISE_PX = 8.0

def label_seed(label):
    # A stable, non-randomized substitute for Python's hash(), which is
    # salted per-process and would make "deterministic" jitter not actually
    # reproducible across runs.
    return sum(ord(c) for c in label)

def make_detection(label, px, py, depth, base_confidence, jitter):
    obj_width_m = 0.4
    bbox_w = min(0.6, max(0.02, (FX * obj_width_m / depth) / WIDTH))
    bbox_h = bbox_w
    return {
        "label": label,
        "confidence": round(min(0.95, max(0.15, base_confidence + jitter)), 3),
        "bbox_x": round(max(0.0, px / WIDTH - bbox_w / 2), 4),
        "bbox_y": round(max(0.0, py / HEIGHT - bbox_h / 2), 4),
        "bbox_w": round(bbox_w, 4),
        "bbox_h": round(bbox_h, 4),
    }

frames = []
t0 = sampled[0][0]
for i, (t, x, y, z, qx, qy, qz, qw) in enumerate(sampled):
    detections = []
    for label, pos in (("chair", chair_pos), ("door", door_pos)):
        if pos is None:
            continue
        hit = project((x, y, z), (qx, qy, qz, qw), pos)
        if hit is None:
            continue
        px, py, depth = hit
        seed = label_seed(label)
        px += DETECTOR_NOISE_PX * math.sin(i * 1.3 + seed)
        py += DETECTOR_NOISE_PX * math.cos(i * 1.7 + seed * 2)
        # Deterministic small confidence jitter so fused confidence isn't
        # perfectly uniform across observations, without a stray RNG dependency.
        jitter = 0.1 * math.sin(i * 0.7 + seed % 7)
        detections.append(make_detection(label, px, py, depth, 0.6, jitter))

    frames.append({
        "pose": {
            "timestamp": round(t - t0, 4),
            "x": round(x, 4), "y": round(y, 4), "z": round(z, 4),
            "qx": round(qx, 4), "qy": round(qy, 4), "qz": round(qz, 4), "qw": round(qw, 4),
        },
        "intrinsics": {"fx": FX, "fy": FY, "cx": CX, "cy": CY},
        "detections": detections,
    })

with open(OUT, "w") as f:
    json.dump(frames, f, indent=2)

total_dets = sum(len(fr["detections"]) for fr in frames)
print(f"wrote {len(frames)} frames, {total_dets} detections to {OUT}", file=sys.stderr)
