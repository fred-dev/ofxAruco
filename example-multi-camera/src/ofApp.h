#pragma once

// ofxAruco: calibrate several cameras together (extrinsics) with a ChArUco board.
//
// Runs out of the box with 3 SIMULATED cameras ('m' switches to real webcams,
// configured in bin/data/cameras.json).
//
// Workflow:
//   1. print bin/data/board.yml with example-print-boards (default: ChArUco 7x5,
//      55 mm squares, A3), glue it on something flat. Measure it, fix board.yml.
//   2. show the board to 2+ cameras at once and HOLD IT STILL: a sample is taken
//      automatically (green bar). Move it, tilt it, repeat ~15-30 times all over
//      the shared volume. 'space' takes a sample by hand.
//   3. watch the reprojection error per camera (< 1 px is good) and the 3D view.
//   4. optional: put the board where you want the world origin (e.g. on the floor)
//      and press 'w'.
//   5. 's' saves bin/data/multicam_calibration.json: one cameraToWorld 4x4 per camera.
//
// Each camera has its own ofxAruco detector (each with its own thread).

#include "ofMain.h"
#include "ofxAruco.h"
#include "ofxGui.h"
#include "CameraSource.h"
#include "SimulatedCameras.h"

class ofApp : public ofBaseApp {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void keyPressed(int key) override;
	void exit() override;

	void setupCameras(bool simulated);
	void loadOrCreateBoard();
	void drawScene3D(const ofRectangle & viewport);
	void drawFrustum(const ofxArucoIntrinsics & intrinsics, float depth) const;
	void setWorldFromCurrentBoard();
	std::vector<const ofxAruco *> getDetectors() const;

	// cameras + one detector per camera
	std::vector<std::unique_ptr<CameraSource>> cameras;
	std::vector<std::unique_ptr<ofxAruco>> detectors;
	std::shared_ptr<SimulatedScene> simScene;
	bool simulated = true;

	ofxArucoBoard board;
	ofxArucoMultiCamCalibration calibration;
	// where the board was when each sample was taken (camera index + pose), to show coverage
	std::vector<std::pair<int, glm::mat4>> samplePoses;

	// GUI: the detector settings are edited here and copied to every detector
	ofxPanel gui;
	ofxAruco detectorSettings;
	ofParameter<bool> showSamples{ "show samples", true };
	ofParameter<bool> showTruth{ "show ground truth (sim)", true };
	ofEventListeners listeners;

	ofEasyCam easyCam;
	std::string message;
	float messageTime = 0;
};
