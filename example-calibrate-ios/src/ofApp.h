#pragma once

// Camera calibration on iPhone / iPad: every native format of every back lens (wide,
// ultra wide, telephoto), with a ChArUco board.
//
// 1. Pick a lens. Its native formats are planned: the largest of each aspect ratio
//    and field of view gets a full calibration (~20 views), the others a short
//    verification of the scaled result (formats with a different field of view crop
//    the sensor, iOS reports it, so they are calibrated fully).
// 2. Point at the board, let it focus, tap "Lock focus". The intrinsics depend on the
//    focus: it stays locked at that lens position for every format of the session.
// 3. Hold the phone still and move the board, or keep the board still and move the
//    phone. Views are taken automatically when the image is still and new; follow the
//    hints and fill the red cells.
// 4. Results: Documents/calibrations/<model>_<lens>/<model>_<lens>_<width>x<height>.yml
//    + index.yml, visible in the Files app (On My iPhone > example-calibrate-ios).
//    Each file also has the focus position, the distortion correction setting and the
//    intrinsics iOS itself reports, for comparison.
//
// Frames are calibrated in the sensor's native landscape orientation (what
// openFrameworks' iOS grabber delivers). The preview is shown rotated upright.
//
// bin/data/board.yml: the printed ChArUco board (ofxAruco/example-print-boards).

#include "ofxiOS.h"
#include "ofxAruco.h"
#include "IOSCamera.h"

class ofApp : public ofxiOSApp {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void exit() override;

	void touchDown(ofTouchEventArgs & touch) override;
	void lostFocus() override;
	void gotFocus() override;

	enum class Screen { Lenses, Running, Summary };

	struct Button {
		std::string label;
		ofRectangle rect;
		std::function<void()> action;
		bool highlighted = false;
	};

	void buildPlan(bool resume);
	void startLens(bool resume);
	void startTask(int index);
	void nextTask();
	void finishTask();
	void stopLens();
	void saveIndex();
	void say(const std::string & text);
	std::string lensLabel() const;    // <model>_<lens>[_DC]
	std::string orientHint(const std::string & hint) const; // image directions -> screen directions

	void drawLenses();
	void drawRunning();
	void drawSummary();
	float drawPlan(float x, float y, int highlight);
	void drawText(const std::string & text, float x, float y, const ofColor & color = ofColor(255));
	void addButton(const std::string & label, const ofRectangle & rect, std::function<void()> action, bool highlighted = false);
	void drawButtons();
	float lineHeight() const { return 14 * textScale; }

	Screen screen = Screen::Lenses;
	std::vector<IOSCameraDevice> lenses;
	int selected = 0;
	std::string model;
	bool distortionCorrection = false;

	IOSCamera camera;
	ofTexture preview;
	ofxArucoBoard board;
	ofxArucoCalibrator calibrator;
	ofxArucoCalibrationPlan plan;
	std::string folder; // absolute, in Documents
	int current = -1;
	int framesSinceOpen = 0;
	bool paused = false;
	std::string message;
	std::string boardError;

	std::vector<Button> buttons; // rebuilt every frame, hit tested in touchDown
	float textScale = 2;

	// when a format is done
	int fullViews = 20;
	float fullCoverage = 0.7f;
	int verifyViews = 8;
	float verifyCoverage = 0.35f;
	float verifyTolerance = 0.015f;
	double maxRms = 1.0;
	int warmupFrames = 20;
};
