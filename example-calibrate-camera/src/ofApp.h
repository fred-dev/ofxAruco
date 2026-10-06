#pragma once

// Camera calibration for class compliant (USB / built in) cameras, macOS and Windows.
//
// 1. Pick a camera. Its native modes are listed and planned:
//      FULL    the largest mode of each aspect ratio: a complete calibration (~20 views)
//      VERIFY  the other modes: ~8 views to check that the FULL result, scaled,
//              fits. If it does, the scaled intrinsics are saved. If not (the mode
//              crops the sensor), the mode gets a complete calibration.
// 2. Hold the ChArUco board (bin/data/board.yml, print it with
//    ofxAruco/example-print-boards at 100%) in front of the camera. Views are taken
//    automatically when the board is still and the view is new. Follow the hints:
//    cover the whole image (corners too, the board may stick out), tilt the board,
//    change the distance.
// 3. Results: bin/data/calibrations/<camera>/<camera>_<width>x<height>.yml + index.yml,
//    e.g. calibrations/Logitech_BRIO/Logitech_BRIO_1920x1080.yml.
//    Use them with intrinsics.loadForResolution(folder, width, height).
//
// Sessions can be resumed: modes already in the folder are skipped (F starts fresh).

#include "ofMain.h"
#include "ofxAruco.h"
#include "CameraDevices.h"

class ofApp : public ofBaseApp {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void keyPressed(int key) override;
	void exit() override;

	enum class Screen { Cameras, Running, Summary };

	void refreshCameras();
	void buildPlan(const CameraDevice & camera, bool resume);
	void startCamera(bool resume);
	void startTask(int index);
	void nextTask();
	void finishTask();
	void saveIndex();
	void say(const std::string & text);

	void drawCameras();
	void drawRunning();
	void drawSummary();
	void drawPlan(float x, float y, int highlight);

	Screen screen = Screen::Cameras;
	std::vector<CameraDevice> cameras;
	int selected = 0;

	ofxArucoBoard board;
	ofxArucoCalibrator calibrator;
	ofxArucoCalibrationPlan plan;
	std::string folder;
	int current = -1;

	ofVideoGrabber grabber;
	int framesSinceOpen = 0;
	bool paused = false;
	std::string message;
	std::string boardError;

	// when a mode is done
	int fullViews = 20;            // views for a full calibration
	float fullCoverage = 0.7f;     // fraction of the image covered by corners
	int verifyViews = 8;           // views to verify a scaled calibration
	float verifyCoverage = 0.35f;
	float verifyTolerance = 0.015f; // max relative difference of fx, fy, cx, cy (1.5%)
	double maxRms = 1.0;           // above this a result is flagged
	int warmupFrames = 20;         // let auto exposure settle after opening a mode
};
