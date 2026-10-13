#pragma once

// ofxAruco basic example: recognise any marker, and any board you give it, in one
// camera, and draw their 3D poses on top of the image.
//
// - Markers: with "any dictionary" on (key 'a'), markers of every OpenCV dictionary
//   are recognised (ArUco 4X4 .. 7X7, original ArUco, AprilTag, ArUco MIP). Each one is
//   labelled with its id, and its dictionary when it isn't the main one.
// - Boards: every board .yml in bin/data is loaded (the ones example-print-boards saves:
//   ChArUco, grid, cards, single markers), whatever its dictionary. 'r' reloads them.
// - Sizes: a single marker's pose needs its printed side. Markers that are in a loaded
//   .yml use that file's marker length (so copy the .yml of a marker you printed, e.g.
//   marker_45_170mm_DICT_4X4_50.yml, into bin/data). Others use "marker length (m)" in
//   the aruco panel.
//
// It runs out of the box on bin/data/videoboard.mp4 (a board of original ArUco markers,
// boardConfiguration.yml). Press 'v' to switch to a webcam.
//
// Files in bin/data:
//   intrinsics.yml          camera calibration of the camera that filmed the video
//                           (your webcam needs its own: example-calibrate-camera)
//   boardConfiguration.yml  old ArUco 1.x board layout of the video
//   *.yml                   your boards and markers, from example-print-boards/bin/data
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
	void loadBoards();

	ofVideoPlayer player;
	ofVideoGrabber grabber;
	ofBaseVideoDraws * video = nullptr;
	bool usingWebcam = false;

	ofxAruco aruco;
	std::vector<std::string> boardInfo; // what was loaded, and warnings

	ofxPanel gui;
	bool showGui = true;
	ofParameter<bool> drawMarkers{ "draw markers", true };
	ofParameter<bool> drawCubes{ "draw 3D cubes", true };
	ofParameter<bool> drawBoard{ "draw boards", true };
	ofParameter<bool> drawRejected{ "draw rejected", false };
	ofParameter<bool> threaded{ "threaded", true };
	ofEventListener threadedListener;
};
