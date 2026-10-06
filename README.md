ofxAruco
========

Threaded ArUco / ChArUco marker and board detection, pose estimation and
**multi camera calibration** for openFrameworks.

Built on OpenCV's own `objdetect` module (OpenCV >= 4.7, the one inside
`ofxOpenCv`). No other dependency: no ofxCv, no Poco, no bundled ArUco library.

* Detect markers and boards with one call, on a worker thread (never blocks your app)
* 3D poses in meters, drawing helpers that line up with the camera image
* Grid boards, **ChArUco** boards (most accurate) and custom 3D boards, saved as YAML (OpenCV FileStorage, like calibration files; JSON also works)
* Camera intrinsics from OpenCV `.yml`, JSON, or plain `fx fy cx cy` + distortion (e.g. Azure Kinect factory calibration)
* Calibrate several cameras together, with bundle adjustment, saved as JSON (`cameraToWorld` per camera)
* All settings are `ofParameter`s: drop them in an `ofxPanel`, save/load JSON
* 8 bit (gray, RGB, RGBA, BGRA) and 16 bit (IR) images


Requirements
------------

* openFrameworks 0.12+ with `ofxOpenCv` (tested: OF 0.12.1 nightly, OpenCV 4.14, Windows, Visual Studio 2026)
* `addons.make`: `ofxAruco` and `ofxOpenCv` (+ `ofxGui` for the examples)


Quick start
-----------

```cpp
#include "ofxAruco.h"

ofVideoGrabber grabber;
ofxAruco aruco;

void ofApp::setup() {
    grabber.setup(1280, 720);
    aruco.setup(cv::aruco::DICT_5X5_100);   // starts the detection thread
    aruco.loadIntrinsics("intrinsics.yml");  // needed for 3D poses
    aruco.markerLength = 0.05f;              // printed marker side, in meters
}

void ofApp::update() {
    grabber.update();
    if (grabber.isFrameNew()) aruco.detect(grabber.getPixels()); // returns immediately
}

void ofApp::draw() {
    grabber.draw(0, 0);
    aruco.draw(); // outlines, ids and 3D axes

    for (int i = 0; i < aruco.getNumMarkers(); i++) {
        const auto & m = aruco.getMarkers()[i]; // m.id, m.corners (pixels), m.pose (marker -> camera, meters)
        aruco.begin(i);                         // draw in marker i's frame, in meters
        ofDrawBox(0, 0, 0.025f, 0.05f);
        aruco.end();
    }
}
```

Results are picked up automatically at the start of every frame, so
`getMarkers()` / `getBoardPose()` are stable while you draw. Use one `ofxAruco`
per camera: each has its own thread. `detect()` hands the frame to the thread;
if the thread is still busy the older pending frame is dropped, so latency
stays low. `setThreaded(false)` runs detection immediately instead.


Coordinate conventions (read this once)
---------------------------------------

* **Camera frame = OpenCV**: x right, y **down**, z **forward** (out of the lens).
  The Azure Kinect SDK uses the same convention (in millimeters).
* **Units** are the units of your marker / board sizes. Use **meters**.
* A pose is `objectToCamera`: `p_camera = pose * p_object`. `pose[3]` is the
  object's position seen from the camera.
* Marker frame: origin at the marker center, x right, y up, z towards the camera.
* Board frame (OpenCV's): origin at the top-left corner of the printed board,
  x right, y down the board, z into the board.
* `ofxArucoUtils::cvToGl()` converts between the OpenCV and OpenGL camera frames.
  `aruco.beginCamera()` / `intrinsics.begin(viewport)` let you draw directly
  in OpenCV camera coordinates on top of the image.


Boards
------

```cpp
// ChArUco: chessboard + markers. Best for calibration (sub-pixel corners, works partially occluded)
auto board = ofxArucoBoard::makeCharuco(7, 5, 0.055f, 0.041f, cv::aruco::DICT_5X5_100);
board.save("board.yml");                   // or aruco.loadBoard("board.yml")
int index = aruco.addBoard(board);

const auto & bp = aruco.getBoardPose(index);
if (bp.found) { bp.pose; bp.numPoints; bp.reprojectionError; }
```

`board.yml`:

```yaml
%YAML:1.0
---
type: "charuco"
dictionary: "DICT_5X5_100"
squaresX: 7
squaresY: 5
squareLength: 0.055
markerLength: 0.041
```

The format follows the extension: `.yml` / `.yaml` (default) or `.json`, with the same keys.

Also `"type": "grid"` (`markersX`, `markersY`, `markerLength`, `markerSeparation`)
and `"type": "custom"` (markers with their 4 corners in 3D, written as an ArUco marker map:
`aruco_bc_markers`, in meters, so ArUco tools can read it; `load()` also reads ArUco marker maps in meters). Old ArUco 1.x
`boardConfiguration.yml` files load with `board.loadLegacyAruco(path, markerLength, dictionary)`.

Print boards with **example-print-boards**: it writes a PNG that prints at the
right physical size at 100% scale, plus the matching YAML. Always measure the
printed board and fix the lengths in the YAML if the printer scaled it.
It has presets on a slider: A4 ChArUco boards 4x3, 7x5 and 10x8, A4 sheets of
cut-out marker cards with the same counts and marker sizes (ids 20 and up), an
A1 single marker, and "user" for your own settings.

The detector dictionary must match the boards' dictionary.


Camera intrinsics
-----------------

```cpp
ofxArucoIntrinsics intrinsics;
intrinsics.load("intrinsics.yml"); // OpenCV calibration file (.yml/.xml), .json, or legacy .int

// or directly, e.g. from an Azure Kinect (k4a_calibration_t):
//   auto & p = calibration.color_camera_calibration.intrinsics.parameters.param;
//   intrinsics.setup(p.fx, p.fy, p.cx, p.cy, width, height, { p.k1, p.k2, p.p1, p.p2, p.k3, p.k4, p.k5, p.k6 });
aruco.setIntrinsics(intrinsics);
```

If the image size differs from the calibration size (same aspect ratio), the
intrinsics are scaled automatically.


Calibrating a camera (intrinsics)
---------------------------------

`ofxArucoCalibrator` calibrates one camera with a ChArUco board: feed it frames, it keeps
views that are still, well detected and different from the previous ones (it tells the user
what to do next), then solves focal length, principal point and distortion.

```cpp
calibrator.setup(board);                       // a ChArUco board
calibrator.start({ 1920, 1080 });
// update(): if (grabber.isFrameNew()) calibrator.update(grabber.getPixels());
// draw():   grabber.draw(r); calibrator.draw(r); ofDrawBitmapString(calibrator.getStatus(), 10, 20);
auto result = calibrator.calibrate();          // result.rms: under 0.5 px is good
ofxArucoCalibrator::save("calibrations/mycam/1920x1080.yml", result);
```

`ofxArucoCalibrationPlan` plans all the modes of a camera: a full calibration for the largest
mode of each aspect ratio, a quick verification of the scaled result for the others. Load the
result for the resolution you run at with
`intrinsics.loadForResolution("calibrations/mycam", width, height)`.
`example-calibrate-camera` is a complete tool built on these: it lists a USB / built in camera's native modes (macOS, Windows) and calibrates them all, saving `calibrations/<camera>/<camera>_<width>x<height>.yml`. `example-calibrate-blackmagic` does the same for cameras on a Blackmagic capture device, `example-calibrate-ios` for the back cameras of iPhones and iPads.


Multi camera calibration
------------------------

`ofxArucoMultiCamCalibration` finds where every camera is relative to the others
from a board that 2+ cameras see at the same time.

```cpp
ofxArucoMultiCamCalibration calibration;
calibration.setup(numCameras);                  // camera 0 = reference = world origin

// every frame (auto capture when the board has been held still in 2+ cameras):
calibration.update({ &arucoA, &arucoB, &arucoC });
// or on demand:
calibration.addSample({ &arucoA, &arucoB, &arucoC });

calibration.getCameraToWorld(1);                // 4x4, meters
calibration.getCamera(1).reprojectionError;     // RMS pixels: < 1 px is good
calibration.save("multicam_calibration.json");
```

How it works: each sample gives, for every pair of cameras that saw the board,
`cameraJ -> cameraI = boardToCamI * inverse(boardToCamJ)`. Those are averaged
(with outlier rejection) and chained to the reference camera, so a camera that
never sees the board together with camera 0 can still be calibrated through
another one. Then a **bundle adjustment** refines all camera and board poses
together by minimizing the reprojection error of every board corner in every
camera. In the self test this brings the error from a few mm down to around 1 mm
or less.

Tips: rigid, flat board (A3 ChArUco glued on foam board), 15-30 samples spread
over the whole shared volume at different angles, hold it still (auto capture
waits for it), and check the per camera reprojection error. `setWorldFromBoard()`
moves the world origin onto the board, e.g. lying on the floor at the center of
your scene.

`multicam_calibration.json` (matrices are row-major, 4 rows of 4):

```json
{
  "referenceCamera": 0,
  "cameras": [
    { "index": 0, "name": "kinect A", "cameraToWorld": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], "reprojectionErrorPx": 0.21, ... },
    { "index": 1, "name": "kinect B", "cameraToWorld": [[0.8,0.06,-0.6,1.3], ...], ... }
  ]
}
```

### Using it for a merged point cloud (e.g. Azure Kinect)

Detect the board on each Kinect's **color** image with its color intrinsics.
Kinect points come in the depth camera frame, in millimeters, so per point:

```
p_world = cameraToWorld[k] * depthToColor[k] * (p_depth_mm * 0.001)
```

`depthToColor` is the device's own extrinsic from `k4a_calibration_t`
(`extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR]`, rotation + translation in mm).
If you transform the depth image to the color camera with the SDK
(`k4a_transformation_depth_image_to_color_camera`), the points are already in
the color frame and `depthToColor` is the identity. ofxAruco has no Kinect
dependency: any source of pixels + intrinsics works.


Settings and GUI
----------------

All detector settings are `ofParameter`s in `aruco.parameters`:

```cpp
gui.setup("settings", "settings.json");
gui.add(aruco.parameters);             // dictionary, marker length, corner refinement, thresholds...
gui.add(calibration.parameters);       // auto capture, still time, quality limits...
gui.loadFromFile("settings.json");
```

or without ofxGui: `aruco.saveSettings("aruco.json")` / `aruco.loadSettings("aruco.json")`.

Speed: detection of a 1280x720 frame takes ~6-9 ms on the worker thread; the
`detect()` call itself costs ~0.2 ms (copy + grayscale). For 4K images enable
`useAruco3Detection` (fast mode). It only finds markers bigger than about
`aruco3MinSide + aruco3MinMarkerRatio * imageWidth` pixels.


Examples
--------

| folder | what | needs |
|---|---|---|
| `example` | markers + board on the included video, 3D cubes on the markers. `v` switches to a webcam | nothing |
| `example-print-boards` | make printable ChArUco / grid boards / markers / cut-out cards + their YAML, with presets, self-checked by detection | nothing |
| `example-calibrate-camera` | calibrate every native resolution of a USB / built in camera with a ChArUco board (macOS, Windows) | a camera + printed ChArUco board |
| `example-calibrate-ios` | calibrate every native format of the back lenses of an iPhone / iPad (lens, focus lock, field of view aware) | iOS device + board |
| `example-calibrate-blackmagic` | the same through a Blackmagic capture device: calibrates each resolution the camera outputs, asks for camera body, sensor setting, lens and lens settings, which name the files | ofxBlackmagic, Desktop Video, camera + board |
| `example-multi-camera` | calibrate several cameras together, 3D view, save JSON. Runs with **3 simulated cameras** (with ground truth), `m` switches to webcams (`bin/data/cameras.json`) | nothing / 2+ webcams |
| `tests/ofxArucoTests` | automated self test: renders boards with known poses and checks detection, poses, calibration, threading, files, the video | nothing |

The Visual Studio projects were made with the projectGenerator. To regenerate
them, run the projectGenerator on the example folder (its `addons.make` lists the addons).


Upgrading from the old ofxAruco (ArUco 1.x)
-------------------------------------------

* `setup(calibrationFile, w, h, boardConfig, markerSize)` becomes:
  `setup(dictionary)`, `loadIntrinsics(file)`, `markerLength = ...`, `loadBoard("board.yml")` (or `loadLegacyAruco` for old ArUco 1.x `.yml` boards)
* `detectMarkers(pixels)` / `detectBoards(pixels)` become `detect(pixels)` (markers and boards in one pass)
* `getMarkers()`, `getNumMarkers()`, `begin(i)`, `beginBoard(i)`, `end()`, `getProjectionMatrix()`, `getModelViewMatrix(i)` still exist
* `aruco::Marker` becomes `ofxArucoMarker` (`id`, `corners`, `pose`); `getBoardProbability()` becomes `getBoardPose(i).found` / `.numPoints`
* Old 1024-marker ArUco markers: `cv::aruco::DICT_ARUCO_ORIGINAL`. "Highly reliable markers" dictionaries are replaced by OpenCV's predefined dictionaries (`DICT_5X5_100`, `DICT_6X6_250`, AprilTag...)
* Poses are in meters, OpenCV camera frame


Troubleshooting
---------------

* **Link errors** `unresolved external symbol __std_rotate / __std_max_element_d_` with
  ofxOpenCv's OpenCV 4.14: OpenCV was built with Visual Studio 2026 (toolset v145).
  Build with VS 2026 / v145 (the included projects use it).
* **Crash at the first log line** (null logger channel) with some OF nightlies + VS 2026:
  call `ofLogToConsole();` at the start of `main()` (the examples do).
* Board not found: check the dictionary, print size (measure it!), glare, and that the
  board has a white margin around it. Turn on "draw rejected" in the example.
* Poses wrong scale: `markerLength` / board lengths must be in meters and match the print.
* Poses slightly off: use real intrinsics for your camera, not guessed ones.


License
-------

The original ofxAruco by Arturo Castro. ArUco algorithms by Rafael Muñoz-Salinas
et al., now part of OpenCV (Apache 2).
