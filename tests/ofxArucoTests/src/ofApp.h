#pragma once

// Automated self test for ofxAruco. Runs without any camera:
//   - renders a ChArUco board with virtual cameras of known intrinsics/poses
//     (OpenGL, through ofxArucoIntrinsics::begin) and checks the detected poses
//   - checks multi camera calibration against the ground truth
//   - threaded vs main thread results, 16 bit input, JSON round trips
//   - detection on the example video (../../example/bin/data/videoboard.mp4)
// Prints a report, writes bin/data/test_report.txt and exits with the number of failures.

#include "ofMain.h"
#include "ofxAruco.h"

class ofApp : public ofBaseApp {
public:
	void setup() override;
	void update() override;
	void draw() override;

	// renders the board seen from a virtual camera into `pixels`
	void render(const ofxArucoIntrinsics & cam, const glm::mat4 & boardToCamera, ofPixels & pixels);
	void check(bool ok, const std::string & what);
	void setBoard(const ofxArucoBoard & b);
	void runSyntheticTests();
	void finish();

	ofxArucoBoard board;
	ofTexture boardTexture;
	float boardMarginMeters = 0.02f;
	ofFbo fbo;
	ofPixels lastRender;
	ofImage lastRenderImage;

	ofVideoPlayer video;
	ofxAruco videoAruco;
	int videoFrames = 0, videoFramesWithMarkers = 0, videoFramesWithBoard = 0;
	double videoMarkerSum = 0, videoReprojSum = 0;
	float videoStart = 0;

	int failures = 0;
	std::stringstream report;
	int stage = 0;
};
