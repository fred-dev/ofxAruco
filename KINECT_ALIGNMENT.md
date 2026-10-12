# Aligning two Azure Kinects with a marker (point based)

How to put two (or more) Azure Kinects into one shared, gravity-aligned world, using
**ofxk4a** (cameras, factory calibration, point clouds, IMU) and **ofxAruco** (marker
detection), so that a combined point cloud can be saved as one mesh.

This is a plan and a reference: what to build, in what order, with the maths and the
checks. Nothing here is implemented yet as a ready example.

```
camera A (reference)                         camera B
 IMU gravity, captured once ('g')            marker points matched with A
        |                                            |
   T_world_A  (gravity aligned)               T_A_B  (rigid, from point pairs)
        \______________________ world ___________________/
   p_world = T_world_A * p_A          p_world = T_world_A * T_A_B * p_B
```

---

## 1. Why points, not board poses

At 4 m an A4 board's *pose* (its rotation especially) is too uncertain, and rotation
errors grow with distance: 0.5 degrees is 3.5 cm at 4 m. The Kinects measure depth, so we
don't need the marker's pose at all:

* one marker, detected in both colour images, gives **one 3D point in each camera's frame**
  (marker centre pixel + depth);
* moving the marker through the shared space gives 50 - 200 **matched point pairs**;
* the rigid transform between the point sets is solved directly. The rotation is then set
  by how far apart the points are (metres), not by the size of a board (centimetres).

With ~2 mm depth noise and 100 points spread over 3 m the rotation is good to ~0.01
degrees (< 1 mm at 4 m). The same solver works later for any other point source (VR
trackers, motion capture markers).

## 2. What you need

| | |
|---|---|
| Cameras | 2 Azure Kinects, **sync cable** (master / subordinate) strongly recommended, rigid mounts |
| Software | ofxk4a, ofxAruco, ofxOpenCv, ofxGui |
| Factory calibration | exported per serial with `ofxk4a/example-factory-calibration` (colour + depth intrinsics, depth <-> colour extrinsics) |
| Marker | `DICT_4X4_50` **id 45** (white centre), 170 mm, A4, printed with `ofxAruco/example-print-boards`, **glued flat on foam board**, measured |
| Colour mode | 2160p (4K): ~80 px per marker side at 4 m |
| Depth mode | the one you will use for the point clouds (NFOV unbinned or WFOV); calibrate in the same mode |

Why id 45: the middle of the marker is white paper. Depth is read there, and many black
inks are invisible to (or absorb) the Kinect's infrared, which gives wrong depth on black.

## 3. Frames and conventions

* **Camera frame** (per Kinect): the **depth camera** frame, OpenCV convention: x right,
  y down, z forward, **metres** (ofxk4a's point clouds are in this frame unless
  `bigPointCloud` is on, which puts them in the colour camera frame: pick one and use it
  everywhere; this document uses the depth frame).
* **Colour -> depth**: `p_depth = R_cd * p_colour + t_cd`, from the factory calibration
  (`AzureKinect_<serial>_extrinsics.yml`: `color_to_depth_rotation` / `_translation`, metres).
* **IMU -> depth**: from the device calibration (`k4a_calibration_t::extrinsics
  [K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_DEPTH]`). *Not in the exported files
  yet: add it to ofxk4a's FactoryCalibration (see section 9).*
* **World**: gravity aligned, **Y up**, right handed, metres (openFrameworks / OpenGL
  convention), origin at camera A's depth camera (optionally moved to the floor, 4.4).
* All transforms are 4x4 rigid matrices (rotation + translation, no scale), stored
  row-major in YAML, used as `glm::mat4` (column vectors: `p' = M * p`).

## 4. Step 1: camera A to world, from its IMU (key `g`)

The accelerometer at rest measures the reaction to gravity: a vector pointing **up**.

1. Keep camera A still. On `g`, average the accelerometer for ~2 s (`ofxK4aInterface::getIMU().acc`,
   ~200 samples). Reject the capture if the samples vary too much (std > 0.05 m/s^2:
   something moved) or if the magnitude isn't ~9.81 m/s^2.
2. Rotate it into the depth camera frame: `up_A = normalize(R_depth_accel * acc_mean)`.
3. Build the world axes, expressed in camera A's frame:
   ```
   Yw = up_A                                   // world up
   f  = (0, 0, 1)                              // camera A looks along +z
   Zw = -normalize(f - dot(f, Yw) * Yw)        // world "back": camera A looks along -Zw
   Xw = cross(Yw, Zw)
   R_A_world = [Xw Yw Zw]   (columns)          // world -> A
   R_world_A = transpose(R_A_world)            // A -> world
   T_world_A = [R_world_A | 0]                 // origin at camera A
   ```
   Check: a level camera (`up_A = (0,-1,0)`) gives `Xw = (1,0,0)`, `Yw = (0,-1,0)`,
   `Zw = (0,0,-1)`: world x = camera x, world y = up, camera looking along world -z.
   Yaw (heading) can't come from gravity: it is defined by camera A's view direction.
4. Optional, floor as origin: fit a plane (RANSAC) to camera A's points whose normal is
   within ~10 degrees of `up_A`, take the lowest large plane, and shift the world origin
   down to it (`T_world_A.translation.y = distance from the camera to the floor`). The
   floor normal can also refine `up_A` (it is usually more accurate than the IMU: < 0.1 deg).

Save `T_world_A` with camera A's serial, the raw `acc_mean` and the date.

## 5. Step 2: camera B to camera A, from marker point pairs

### 5.1 Setup

* Both Kinects open by **serial number** (not index: the index order can change).
* Sync cable: A master, B subordinate with `depth_delay_off_color_usec = 160` so the two
  depth cameras don't fire at the same time (two Kinects interfere otherwise and depth gets
  noisy or holes appear). Without a cable: use different `depth_delay_off_color_usec`
  per camera anyway and rely on the "still" check (5.3) for timing.
* Colour 2160p, the final depth mode, `registerDepthToColour` on (depth per colour pixel).
* **Warm up**: Azure Kinect depth drifts a few millimetres for the first 20 - 40 minutes.
  Run both cameras for 30 minutes before calibrating.
* ofxAruco detector: `DICT_4X4_50`, accept **only id 45**, corner refinement subpixel.

### 5.2 One marker point, in one camera

For each colour frame in which id 45 is found:

1. **Centre pixel**: intersection of the two diagonals of the four refined corners
   (not the mean of the corners: the diagonal intersection is the true projected centre
   under perspective).
2. **Depth patch**: depth-in-colour pixels within a radius of ~0.2 x the marker side (in
   pixels) around the centre: this stays inside the white centre of id 45.
3. **Plane fit**: turn the valid patch pixels into 3D (colour intrinsics x depth), fit a
   plane (least squares, reject outliers > 5 mm), intersect the **centre ray** with it.
   This handles a tilted marker and averages ~hundreds of depth samples.
   (Simpler alternative: median depth of the patch x centre ray; fine when the marker is
   roughly facing the camera.)
4. **Reject** if: fewer than 60% valid depth pixels, plane residual > 3 mm, the marker is
   seen at more than ~60 degrees from face-on, or the centre is within 5% of the image edge.
5. **To the depth frame**: `p = R_cd * p_colour + t_cd`.

### 5.3 A point pair

Keep a pair `(a_i in A, b_i in B)` only when:

* both cameras found id 45 in frames captured together (same capture with the sync cable;
  device timestamps within 1 ms), and
* the marker is **still**: its centre moved less than ~1 px over the last 5 frames in both
  cameras (also protects against timing differences without a cable), and
* it is new: at least ~10 cm from every pair already kept (so the points spread out
  instead of piling up where you stand).

Average the 5 still frames into one pair.

### 5.4 Coverage

Show the kept points in 3D (and as a top view), and aim for:

* 60 - 150 pairs;
* the whole shared volume: near, far (to 4 m), left, right, **high and low** (points all at
  one height leave the roll badly constrained);
* the far region well filled: that's where errors show.

### 5.5 Solve: rigid transform with outlier rejection

Find `R, t` minimising `sum |a_i - (R b_i + t)|^2` (Kabsch / Umeyama, no scale):

```
a_mean = mean(a_i), b_mean = mean(b_i)
H = sum (b_i - b_mean)(a_i - a_mean)^T        // 3x3
H = U S V^T                                   // cv::SVD
D = diag(1, 1, det(V U^T))                    // no reflection
R = V D U^T
t = a_mean - R b_mean
T_A_B = [R | t]
```

Outliers (a bad depth reading, a misdetection): RANSAC around it.

* 500 iterations: solve from 3 random pairs, count inliers with
  `|a_i - (R b_i + t)| < 1 cm + 0.5% of the distance to camera A`.
* Re-solve on all inliers of the best model; repeat the inlier test once more.
* Optional weighting: depth noise grows with distance, weight each pair by `1 / z^2`
  (weighted Kabsch: weighted means and `H = sum w_i (...)(...)^T`).

Report: inliers / total, RMS residual, residual **by distance band** (0-1, 1-2, 2-3, 3-4 m),
and the largest residual. Expect an RMS of 3 - 6 mm. A residual that grows with distance
means something is wrong (depth mode mismatch, colour-depth extrinsics not applied, a warm
up drift, a bent marker).

### 5.6 Optional: ICP refinement

After 5.5 the clouds overlap closely. A few iterations of point-to-plane ICP on the
overlapping region (both clouds downsampled to ~1 cm, correspondences within 2 cm,
`T_A_B` as the start) can remove the last millimetres. Limit it: accept the ICP update only
if it changes the transform by less than ~1 cm / 0.5 degrees and doesn't increase the
marker residual. On flat scenes (a room of walls) ICP can slide along the walls: the marker
solution is the anchor.

### 5.7 Camera B in the world

```
T_world_B = T_world_A * T_A_B
```

More cameras: solve each one against A the same way (or against a camera already in the
world that shares more view with it: `T_world_C = T_world_B * T_B_C`).

## 6. Save and load (by serial)

One file, `bin/data/kinect_alignment.yml` (OpenCV FileStorage format, like the
calibrations):

```yaml
%YAML:1.0
---
created: "2026-10-13 14:02:11"
world: "gravity aligned, Y up, metres, origin at the reference camera (or floor)"
reference_serial: "000123456789"
gravity_acc_reference: [ 0.12, -9.79, 0.31 ]       # raw mean, camera A IMU frame
depth_mode: "NFOV_UNBINNED"
color_resolution: "2160P"
marker: "DICT_4X4_50 id 45, 170 mm"
cameras:
   - { serial: "000123456789", camera_to_world: [ 16 numbers, row major ], pairs: 0, rms_mm: 0. }
   - { serial: "000987654321", camera_to_world: [ ... ], pairs: 112, inliers: 108, rms_mm: 4.1 }
```

At startup each app opens its Kinects by serial and looks its matrix up by serial. If a
serial isn't in the file, warn: that camera hasn't been calibrated.

Recalibrate when a camera is moved or knocked, when the depth mode changes, or when the
residual check (section 8) fails.

## 7. Applying the transform to the points (for saving a mesh)

The transform has to change the **vertex positions**, not the way the cloud is drawn
(`ofPushMatrix` / `ofMultMatrix` only moves the drawing; a saved mesh would still be in
camera coordinates).

**With ofxk4a** this already exists: set `DeviceSettings::transformPointCloud = true` and call
`setPointCloudTransformationMatrix(camera_to_world)`. ofxk4a then transforms every
position (`p = M * p`) and every normal (inverse transpose of the rotation) in the point
cloud cache, so `getPointCloud()` / `getPostitionCache()` are already in world coordinates.

* ofxk4a takes an `ofMatrix4x4`; its memory layout is the same as `glm::mat4` with column
  vectors (`v * M` in ofMatrix4x4 = `M * v` in glm on the same memory). Convert with
  `ofMatrix4x4(glmMatrix)` and **test** with one known point: put the marker in front of
  camera B, check its world position equals the one camera A gives (within ~1 cm).
* Units: the clouds are in metres, the transforms must be in metres too.
* If the point cloud uses the colour camera frame (`bigPointCloud`), use
  `T_world_B * T_depth_color` (the colour -> depth extrinsics) instead.

**Combined mesh**:

```cpp
ofMesh combined;
for (auto & kinect : kinects) {                  // each already in world coordinates
	const ofMesh & m = kinect.getPointCloud();
	const ofIndexType offset = combined.getNumVertices();
	combined.addVertices(m.getVertices());
	if (m.hasColors()) combined.addColors(m.getColors());
	if (m.hasNormals()) combined.addNormals(m.getNormals());
	for (auto i : m.getIndices()) combined.addIndex(i + offset);   // keep faces per camera
}
combined.save("scan_" + ofGetTimestampString() + ".ply", true);   // binary PLY
```

Texture coordinates refer to each camera's own colour image, so for a single file bake the
colour into per-vertex colours (ofxk4a's embedded colour option), not UVs. The
overlapping region has points from both cameras: fine for a point cloud; for a single
surface, merge later (voxel downsampling, or Poisson reconstruction in MeshLab /
Open3D).

Also save `kinect_alignment.yml`'s values (or its date) next to each exported mesh, so you
know which calibration a scan used.

## 8. Checks

* **Live residual**: with the calibration loaded, hold the marker anywhere: both cameras'
  world positions of its centre, and the distance between them, on screen. Under ~1 cm
  everywhere in the volume, up to 4 m, means a good calibration.
* **Planes**: the floor (and a wall) fitted from each camera's world points should give
  the same plane: angle < 0.2 degrees, offset < 5 mm.
* **Gravity**: the floor normal in world should be (0, 1, 0) within ~0.3 degrees.
* **Visual**: overlap both clouds coloured per camera: edges of furniture and the floor
  should coincide, also at the far end of the room.
* **Repeat**: calibrate twice; the two `T_A_B` should agree within a few mm / ~0.1 degree.

## 9. What to build

### 9.1 Changes to existing addons

* **ofxk4a / FactoryCalibration**: also export the IMU extrinsics (`ACCEL -> DEPTH`,
  `GYRO -> DEPTH`) into `AzureKinect_<serial>_extrinsics.yml`.
* **ofxk4a**: expose the capture's device timestamps (colour and depth, microseconds) per
  frame, to pair frames from two cameras; and a way to open a device by **serial number**.
* **ofxk4a**: an IMU averaging helper (mean and standard deviation over N seconds).
* **ofxAruco**: a small `ofxArucoRigidSolver` (Kabsch / Umeyama, weighted, RANSAC, residual
  report), independent of Kinects so it can be reused for VR trackers and other point
  sources; and a helper for the marker centre (diagonal intersection).

### 9.2 The app: `ofxk4a/example-align-kinects` (or in ofxAruco with ofxk4a as a dependency)

`addons.make`: ofxOpenCv, ofxAruco, ofxk4a, ofxGui.

| Part | Does |
|---|---|
| `KinectView` | one Kinect by serial: interface, factory intrinsics / extrinsics, marker detection on colour, marker point (5.2) |
| `PairCollector` | synchronisation, stillness, novelty, coverage (5.3, 5.4) |
| `GravityAligner` | IMU capture and world axes (4) |
| `ofxArucoRigidSolver` | RANSAC + Kabsch + report (5.5) |
| `AlignmentStore` | `kinect_alignment.yml` save / load by serial (6) |
| `MeshExporter` | applies the transforms through ofxk4a, combines, saves PLY (7) |

Keys:

| Key | Action |
|---|---|
| `g` | capture gravity on camera A (keep it still ~2 s) |
| `space` | start / pause collecting marker pairs |
| `backspace` | remove the last pair |
| `c` | clear all pairs |
| `s` | solve (RANSAC + Kabsch), show the report |
| `i` | ICP refinement (optional) |
| `w` / `l` | write / load `kinect_alignment.yml` |
| `t` | live residual test (8) |
| `e` | export the combined mesh as PLY |
| `1` `2` `3` | view: camera A, camera B, combined 3D |

## 10. Common problems

| Symptom | Likely cause |
|---|---|
| Error grows with distance | colour -> depth extrinsics not applied; different depth mode than calibrated; depth not warmed up; pairs only near the cameras |
| One camera's cloud tilted | pairs all at one height (roll unconstrained); IMU capture while moving |
| Noisy / holey depth when both run | the two Kinects interfere: set `depth_delay_off_color_usec` (sync cable: 160 us) |
| Marker points jump | depth read on black (use id 45's white centre); marker not flat; motion blur (stillness check) |
| Some pairs far off | misdetection or wrong frame pairing: RANSAC removes them; accept only id 45 |
| Saved mesh not aligned, display is | transform applied to drawing only: use `transformPointCloud` (section 7) |
| Wrong camera gets the wrong matrix | devices opened by index instead of serial |
