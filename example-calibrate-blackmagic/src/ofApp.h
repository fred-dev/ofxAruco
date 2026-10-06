#pragma once

// Camera calibration through a Blackmagic capture device (ofxBlackmagic: DeckLink /
// UltraStudio, macOS and Linux).
//
// With a capture card the resolution is set by the CAMERA's output, not by this app:
// the card detects the incoming format. So:
// 1. Pick the Blackmagic input and fill in the camera: body, sensor setting (the
//    camera's sensor mode / crop, e.g. "S35 4K", "FF 6K", "HD crop"), lens, focal
//    length, focus. They decide the intrinsics (the capture resolution is detected),
//    and together they name the files, e.g. FX6_FF-4K_Sigma18-35_24mm_focus2m.
//    Lock focus and zoom; any change of these = a new calibration. Aperture and
//    notes are recorded in the files but not in the name.
// 2. Calibrate the signal that arrives: hold the ChArUco board (bin/data/board.yml)
//    in front of the camera, views are taken automatically (same as
//    example-calibrate-camera).
// 3. The capture uses exactly the mode you set (it starts at 1920x1080). When the
//    camera sends something else, a warning shows what the signal is: press m to set
//    the capture to it. To calibrate another resolution, switch the camera's output
//    format and press m again. If a resolution with the same aspect ratio is already
//    calibrated, the new one only gets a short verification and the scaled result is
//    saved (or a full calibration if it doesn't match).
// 4. Results: bin/data/calibrations/<name>/<name>_<width>x<height>.yml + index.yml.
//    Use them with intrinsics.loadForResolution("calibrations/<name>", width, height).
//    The last entered settings are kept in bin/data/lens_settings.json.

#include "ofMain.h"
#include "ofxAruco.h"
#include "ofxBlackMagic.h"

class ofApp : public ofBaseApp {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void keyPressed(int key) override;
	void exit() override;

	enum class Screen { Setup, Running };

	void startCapture();
	void stopCapture();
	void onMode(int width, int height);
	void matchSignal(); // set the capture to the format that is arriving
	void startTask();
	void finishTask();
	void saveIndex();
	void say(const std::string & text);
	std::string safeLabel() const;  // file-safe name from the fields
	std::string labelText() const;  // the same, readable
	std::string missingField() const;
	void loadFields();
	void saveFields();
	bool taskIsOpen() const;

	void drawSetup();
	void drawRunning();
	void drawPlan(float x, float y);

	Screen screen = Screen::Setup;
	ofxBlackMagic cam;
	std::vector<ofVideoDevice> devices;
	int selected = 0;
	// the form: what the calibration is valid for
	struct Field {
		std::string key;   // key in the .yml files
		std::string title;
		std::string hint;
		std::string value;
		bool inName;       // part of the file name
		bool required;
		std::string suffix; // added to the value in the name (e.g. "mm")
	};
	std::vector<Field> fields;
	int field = 0;         // 0 = input device, 1.. = fields[field - 1]

	ofxArucoBoard board;
	ofxArucoCalibrator calibrator;
	ofxArucoCalibrationPlan plan;
	std::vector<ofxArucoCameraMode> seenModes; // resolutions that arrived in this session
	std::string folder;
	int current = -1;           // task of the incoming resolution
	int currentWidth = 0, currentHeight = 0;
	int framesSinceMode = 0;
	bool paused = false;
	std::string message;
	std::string boardError;

	int fullViews = 20;
	float fullCoverage = 0.7f;
	int verifyViews = 8;
	float verifyCoverage = 0.35f;
	float verifyTolerance = 0.015f;
	double maxRms = 1.0;
	int warmupFrames = 20;
};
