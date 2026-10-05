#pragma once

// ofxAruco: make printable boards and markers.
//
// Pick a board type, dictionary and sizes in the GUI, then press "save".
// It writes to bin/data/:
//   <name>.png   the image at the chosen DPI: print it at 100% scale (no "fit to page")
//   <name>.json  the board definition for ofxAruco (load it with aruco.loadBoard(...))
//
// After printing, MEASURE the printed square/marker with a ruler and fix the
// length in the .json if your printer scaled it: poses are only as accurate
// as these numbers.
//
// The generated image is detected again here, as a self-check.
//
// The default is the board example-multi-camera expects: ChArUco 7x5, 55 mm
// squares, 41 mm markers, DICT_5X5_100, which fits on A3. Only have A4? Use
// 35 mm / 26 mm squares/markers and copy the saved .json over
// example-multi-camera/bin/data/board.json.
// For calibrating cameras together: as big as you can print it, glued on
// something flat and rigid (foam board, glass, aluminium).

#include "ofMain.h"
#include "ofxAruco.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void keyPressed(int key) override;

	void rebuild();
	void save();
	std::string getFileName() const;

	ofxPanel gui;
	ofParameterGroup params{ "board" };
	ofParameter<int> type{ "type (0 charuco 1 grid 2 marker)", 0, 0, 2 };
	ofReadOnlyParameter<std::string, ofApp> typeName{ "type name", "" };
	ofParameter<int> dictionary{ "dictionary", cv::aruco::DICT_5X5_100, 0, cv::aruco::DICT_ARUCO_MIP_36h12 };
	ofReadOnlyParameter<std::string, ofApp> dictionaryName{ "dictionary name", "" };
	ofParameter<int> countX{ "squares / markers X", 7, 1, 20 };
	ofParameter<int> countY{ "squares / markers Y", 5, 1, 20 };
	ofParameter<float> squareMm{ "square (mm, charuco)", 55, 5, 300 };
	ofParameter<float> markerMm{ "marker (mm)", 41, 5, 300 };
	ofParameter<float> separationMm{ "separation (mm, grid)", 10, 1, 100 };
	ofParameter<int> firstId{ "first marker id", 0, 0, 999 };
	ofParameter<int> dpi{ "dpi", 300, 72, 600 };
	ofParameter<float> marginMm{ "white margin (mm)", 10, 0, 50 };
	ofxButton saveButton;

	ofxArucoBoard board;
	ofPixels pixels;
	ofTexture preview;
	ofxAruco checker;
	std::string status;
	bool dirty = true;
	ofEventListeners listeners;
};
