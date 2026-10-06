# example-calibrate-camera: calibrate class compliant cameras (macOS, Windows)

Calibrates every native mode of a USB / built in camera with a ChArUco board,
using `ofxArucoCalibrator` from ofxAruco.

## Setup

* Addons: `ofxOpenCv`, `ofxAruco` (already in `addons.make`). Generate the project with the projectGenerator.
* Results in `bin/data/calibrations/` are camera specific and ignored by git.
* macOS: the app needs camera permission (`NSCameraUsageDescription` in `openFrameworks-Info.plist`).
* Board: `bin/data/board.yml` (copied from the A4 ChArUco 7x5, 36 mm / 27 mm, DICT_5X5_100 you printed
  with `ofxAruco/example-print-boards`). Use any ChArUco board: put its .yml there as `board.yml`.
  Glue the print on something flat and rigid. The square size only affects poses, not the lens
  calibration, but measure it and fix the .yml anyway.

## How it works

1. The app lists the cameras and their native modes (AVFoundation on macOS, DirectShow on Windows,
   in the same order as `ofVideoGrabber`).
2. Modes are grouped by aspect ratio. The largest mode of each group gets a **full** calibration
   (~20 views covering the whole image). The other modes get a **verification**: ~8 views, solving
   only focal length and principal point starting from the full result scaled to that mode. If the
   difference is under 1.5%, the scaled intrinsics are saved. If not, the mode crops the sensor and
   it gets its own full calibration (the views already taken are kept).
3. Views are taken automatically when the board is still and the view is different from the
   previous ones. Follow the hints: cover the whole image including the corners (the board may stick
   out of the image), tilt it 30 to 45 degrees in all directions, vary the distance.
4. Each finished mode is saved immediately, so you can stop (esc) and resume (ENTER) later.

## Results

`bin/data/calibrations/<camera name>/`

* `<camera name>_<width>x<height>.yml` (e.g. `Logitech_BRIO_1920x1080.yml`), one per native resolution: OpenCV calibration format (`camera_matrix`, `distortion_coefficients`,
  `image_width`, `image_height`, `avg_reprojection_error`, `calibration_method` calibrated / scaled)
* `index.yml`: all modes with their status and error

Files are named by camera name and native resolution, so every mode has its own file and
the files still say what they are when copied elsewhere. Only if two cameras with the same name
are connected at the same time, a short id is added to the name to keep them apart.

Use them in an app:

```cpp
ofxArucoIntrinsics intrinsics;
intrinsics.loadForResolution("calibrations/Logitech_BRIO", grabber.getWidth(), grabber.getHeight());
aruco.setIntrinsics(intrinsics);
```

`loadForResolution` takes the exact mode, or scales the largest one with the same aspect ratio.

Reprojection error: under 0.5 px is good, above 1 px means something is wrong (board not flat,
motion blur, autofocus changing during the session: lock focus if the camera allows it).

## Keys

Camera list: up/down choose, ENTER start (resume), F start fresh, F5 refresh.
Running: space pause, c capture now, backspace remove last view, s solve now, r restart mode,
n skip mode, esc stop.
