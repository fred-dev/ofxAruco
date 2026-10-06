#pragma once

// ofxAruco: make printable boards and markers.
//
// Pick a preset with the "preset" slider, or move it to the last position
// ("user") and set everything by hand. Changing any value while a preset is
// selected switches to "user" and keeps your changes. Press "save" (or 's').
// It writes to bin/data/:
//   <name>.png   the image at the chosen DPI: print it at 100% scale (no "fit to page")
//   <name>.yml   the board definition for ofxAruco (load it with aruco.loadBoard(...))
//
// Presets (the dictionary is not part of a preset, it stays as you set it):
//   A4 ChArUco 4x3   60 mm squares, 45 mm markers, ids 0-5
//   A4 ChArUco 7x5   36 mm squares, 27 mm markers, ids 0-16
//   A4 ChArUco 10x8  23 mm squares, 17 mm markers, ids 0-39
//   A4 cards 4x3     12 single 45 mm markers to cut out, ids 20-31
//   A4 cards 7x5     35 single 27 mm markers to cut out, ids 20-54
//   A4 cards 10x8    80 single 17 mm markers to cut out, ids 20-99
//   A1 single marker 500 mm, id 0
//   user             your own settings (remembered in settings.json)
// Cards are a grid with wide gaps and grey cut lines in the middle of the
// gaps, so every cut-out marker keeps a white border around it. Their .yml is
// a grid board: use it for the ids and the marker length, not as a board
// (once cut out, the markers are no longer where the board says).
//
// After printing, MEASURE the printed square/marker with a ruler and fix the
// length in the .yml if your printer scaled it: poses are only as accurate
// as these numbers.
//
// The generated image is detected again here, as a self-check, and drawn on
// the selected paper size so you can see if it fits.
//
// example-multi-camera expects a ChArUco 7x5 board with 55 mm squares and
// 41 mm markers (A3). If you print one of the A4 boards instead, copy its
// saved .yml over example-multi-camera/bin/data/board.yml.
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
	void exit() override;

	enum BoardType { Charuco = 0, Grid = 1, SingleMarker = 2, Cards = 3 };

	// everything a preset sets
	struct Values {
		std::string name;
		int type, paper, countX, countY;
		float squareMm, markerMm, separationMm;
		int firstId, dpi;
		float marginMm;
	};
	static const std::vector<Values> & getPresets();
	int userPreset() const { return (int)getPresets().size(); }
	Values capture() const;
	void apply(const Values & v);
	void onPresetChanged();
	void loadSettings();
	void saveSettings();

	void rebuild();
	void drawCutLines();
	void save();
	std::string getFileName() const;

	ofxPanel gui;
	ofParameterGroup params{ "board" };
	ofParameter<int> preset{ "preset", 1, 0, 7 };
	ofReadOnlyParameter<std::string, ofApp> presetName{ "preset name", "" };
	ofParameter<int> type{ "type (0 charuco 1 grid 2 marker 3 cards)", 0, 0, 3 };
	ofReadOnlyParameter<std::string, ofApp> typeName{ "type name", "" };
	ofParameter<int> paper{ "paper (0 A4 1 A3 2 A2 3 A1 4 A0 5 Letter)", 0, 0, 5 };
	ofReadOnlyParameter<std::string, ofApp> paperName{ "paper name", "" };
	ofParameter<int> dictionary{ "dictionary", cv::aruco::DICT_5X5_100, 0, cv::aruco::DICT_ARUCO_MIP_36h12 };
	ofReadOnlyParameter<std::string, ofApp> dictionaryName{ "dictionary name", "" };
	ofParameter<int> countX{ "squares / markers X", 7, 1, 20 };
	ofParameter<int> countY{ "squares / markers Y", 5, 1, 20 };
	ofParameter<float> squareMm{ "square (mm, charuco)", 36, 5, 1000 };
	ofParameter<float> markerMm{ "marker (mm)", 27, 5, 1000 };
	ofParameter<float> separationMm{ "separation (mm, grid / cards)", 10, 1, 100 };
	ofParameter<int> firstId{ "first marker id", 0, 0, 999 };
	ofParameter<int> dpi{ "dpi", 300, 72, 600 };
	ofParameter<float> marginMm{ "white margin (mm)", 10, 0, 100 };
	ofxButton saveButton;

	Values userValues;    // what "user" holds while a preset is selected
	int lastPreset = -1;
	bool applying = false; // true while we set parameters ourselves

	ofxArucoBoard board;
	ofPixels pixels;
	ofTexture preview;
	ofxAruco checker;
	std::string status;
	bool fitsPaper = true;
	bool dirty = true;
	ofEventListeners listeners;
};
