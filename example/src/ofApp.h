#pragma once

// ofxAruco basic example: detect markers and a board in one camera, draw
// their 3D poses on top of the image.
//
// It runs out of the box on bin/data/videoboard.mp4 (a board made of
// DICT_ARUCO_ORIGINAL markers). Press 'v' to switch to a webcam: the webcam
// uses YOUR printed board, myBoardFile below (saved by example-print-boards),
// and its dictionary, so single markers of that dictionary are found as well.
//
// The detector finds every marker of its dictionary: there is nothing to
// declare for single markers. You only need the right dictionary, and the
// printed marker side (aruco.markerLength) for their 3D poses.
//
// Files in bin/data:
//   intrinsics.yml          camera calibration of the camera that filmed the video
//   boardConfiguration.yml  old ArUco 1.x board layout of the video
//   charuco_...yml          your board, copied from example-print-boards/bin/data
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
	void useVideoBoard();
	void useMyBoard();

	// the board you printed, used with the webcam
	std::string myBoardFile = "charuco_7x5_36mm_27mm_DICT_5X5_100_id0.yml";

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
