# example-calibrate-ios: calibrate the back cameras of an iPhone / iPad

Every native format of every back lens (wide, ultra wide, telephoto), with a ChArUco board,
using `ofxArucoCalibrator` / `ofxArucoCalibrationPlan` and its own AVFoundation capture
(`IOSCamera`): the openFrameworks iOS grabber can't choose the lens or the exact sensor format,
lock the focus, or turn off stabilisation, and calibration needs all of these.

Addons: `ofxOpenCv`, `ofxAruco` (in `addons.make`). Board: `bin/data/board.yml`.

## Use

1. Pick a lens. Its formats are planned: the largest of each aspect ratio and field of view gets
   a full calibration, the others a short verification of the scaled result (iOS reports each
   format's field of view, so cropped formats are recognised and calibrated fully).
   Ultra wide lenses have a distortion correction switch: calibrate with the setting your app uses
   (files get `_DC` when it is on).
2. Point at the board, let it focus, tap **Lock focus**. The intrinsics depend on the focus: the
   lens position stays locked for every format of the session and is written in the files.
3. Hold the phone still and move the board (or the other way round). Views are taken
   automatically when the image is still and new: follow the hints and fill the red cells.
4. Results: `Documents/calibrations/<model>_<lens>/<model>_<lens>_<width>x<height>.yml` + `index.yml`,
   in the Files app under On My iPhone. Each file also has the focus position, the distortion
   correction setting and the intrinsics iOS reports itself (`ios_reported_intrinsics`), for comparison.

Frames are calibrated in the sensor's native landscape orientation, which is what openFrameworks'
iOS video grabber delivers; the preview is only rotated to stand upright in portrait.

```cpp
intrinsics.loadForResolution(documents + "calibrations/iPhone16,1_BackWide", 1920, 1080);
```
