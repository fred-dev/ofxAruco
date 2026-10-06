# example-calibrate-blackmagic: calibrate a camera through a Blackmagic capture device

Same calibration as `example-calibrate-camera`, with frames from a DeckLink / UltraStudio input
through [ofxBlackmagic](https://github.com/fred-dev/ofxBlackmagic) (macOS, Windows and Linux, Desktop Video 16 or newer installed).

Addons: `ofxOpenCv`, `ofxAruco`, `ofxBlackmagic` (in `addons.make`).

## How it differs from USB cameras

The resolution is the camera's output format. The capture card reports it, and you set the
capture to it (m), so there is no list of modes to work through:

1. Choose the Blackmagic input and fill in what the calibration is valid for: **camera body**,
   **sensor setting** (sensor mode / crop), **lens**, **focal length**, **focus** (plus aperture and
   notes, recorded only). The resolution comes from the camera's output format. Lock focus and zoom:
   any change of these needs a new calibration. The last values are remembered (`bin/data/lens_settings.json`).
2. Calibrate the incoming signal with the ChArUco board (`bin/data/board.yml`).
3. The capture uses the mode you set (it starts at 1920x1080 8 bit YUV). If the camera sends
   something else, a warning shows what the signal is: press **m** to set the capture to it. To
   calibrate another resolution, switch the camera's output format and press m again. If a resolution
   with the same aspect ratio is already calibrated, the new one only gets a short verification (the
   scaled result is saved when it matches, otherwise a full calibration runs).

Results: `bin/data/calibrations/<name>/<name>_<width>x<height>.yml` (+ `index.yml`), where the name
is built from the fields, e.g. `FX6_FF-4K_Sigma18-35_24mm_focus2m_1920x1080.yml`. Every field is also
written in the file, with the capture device and the display mode (e.g. `1080p25`). Previous results are kept: a
resolution already in the folder is skipped (R redoes it).

```cpp
intrinsics.loadForResolution("calibrations/FX6_FF-4K_Sigma18-35_24mm_focus2m", 1920, 1080);
```
