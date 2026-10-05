#pragma once

// ofxAruco basic example: detect markers and a board in one camera, draw
// their 3D poses on top of the image.
//
// It runs out of the box on bin/data/videoboard.mp4 (a board made of
// DICT_ARUCO_ORIGINAL markers). Press 'v' to switch to a webcam.
//
// Files in bin/data:
//   intrinsics.yml          camera calibration of the camera that filmed the video
//   boardConfiguration.yml  old ArUco 1.x board layout of the video
//   settings.json           GUI settings (created when you press 's')

#include "ofMain.h"
#include "ofxAruco.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void keyPressed(int key) override;

	void useVideoFile();
	void useWebcam(int deviceId);

	ofVideoPlayer player;
	ofVideoGrabber grabber;
	ofBaseVideoDraws * video = nullptr;
	bool usingWebcam = false;

	ofxAruco aruco;

	ofxPanel gui;
	bool showGui = true;
	ofParameter<bool> drawMarkers{ "draw markers", true };
	ofParameter<bool> drawCubes{ "draw 3D cubes", true };
	ofParameter<bool> drawBoard{ "draw board", true };
	ofParameter<bool> drawRejected{ "draw rejected", false };
	ofParameter<bool> threaded{ "threaded", true };
	ofEventListener threadedListener;
};
