#pragma once

// A known arrangement of markers with physical sizes, used to estimate one
// accurate pose from many corners.
//
//   ChArUco (recommended for calibration): a chessboard with markers in the
//   white squares. Corners are chessboard corners -> sub-pixel accurate and
//   the board still works when partially occluded.
//   Grid: a plain grid of markers.
//   Custom: any 3D layout (also used to read legacy ArUco 1.x board .yml files).
//
// Sizes are in METERS (or whatever unit you want your poses in).
//
// Board frame (as defined by OpenCV): origin at the top-left corner of the
// printed board, x to the right, y DOWN the page, z INTO the board (away from
// the camera looking at it). This is a right-handed frame.
//
// Boards can be saved/loaded as JSON, e.g. bin/data/board.json:
//   { "type": "charuco", "dictionary": "DICT_5X5_100", "squaresX": 7, "squaresY": 5,
//     "squareLength": 0.04, "markerLength": 0.03 }

#include "ofMain.h"
#include <opencv2/objdetect/aruco_board.hpp>

class ofxArucoBoard {
public:
	enum class Type { None, Grid, Charuco, Custom };

	static ofxArucoBoard makeCharuco(int squaresX, int squaresY, float squareLength, float markerLength,
		int dictionary, int firstMarkerId = 0, bool legacyPattern = false);
	static ofxArucoBoard makeGrid(int markersX, int markersY, float markerLength, float markerSeparation,
		int dictionary, int firstMarkerId = 0);
	// corners per marker: top-left, top-right, bottom-right, bottom-left (as printed), in board units
	static ofxArucoBoard makeCustom(int dictionary, const std::vector<int> & ids,
		const std::vector<std::array<glm::vec3, 4>> & corners);

	// Loads a board .json (see header comment). Relative paths use ofToDataPath().
	bool load(const std::string & path);
	bool save(const std::string & path) const;
	// Loads an ArUco 1.x "boardConfiguration.yml" (corners in pixels). The board is
	// scaled so that one marker side measures markerLength.
	bool loadLegacyAruco(const std::string & path, float markerLength, int dictionary);

	ofJson toJson() const;
	bool fromJson(const ofJson & json);

	bool isValid() const { return type != Type::None; }
	Type getType() const { return type; }
	std::string getTypeName() const;
	int getDictionary() const { return dictionary; }
	const std::string & getName() const { return name; }
	void setName(const std::string & n) { name = n; }

	// Physical extent of the printable area (without margin), in board units.
	glm::vec2 getSize() const;
	// Center of the board in its own frame (handy for drawing).
	glm::vec3 getCenter() const;
	int getNumMarkers() const;

	// Printable image. widthPixels is the width of the board area, marginPixels a
	// white border around it. To print at a known size use getImageForPrint().
	void getImage(ofPixels & pixels, int widthPixels, int marginPixels = 0) const;
	// Image whose pixel size matches the board's physical size at the given DPI
	// (board units are assumed to be meters).
	void getImageForPrint(ofPixels & pixels, float dpi = 300, float marginMeters = 0.01f) const;

	// Access to the OpenCV objects.
	const cv::aruco::Board & getCvBoard() const { return board; }
	const cv::aruco::CharucoBoard & getCvCharuco() const { return charuco; }

private:
	void rebuild();

	Type type = Type::None;
	std::string name = "board";
	int dictionary = 0;
	int firstMarkerId = 0;
	// grid / charuco
	int countX = 0, countY = 0;
	float squareLength = 0, markerLength = 0, markerSeparation = 0;
	bool legacyPattern = false;
	// custom
	std::vector<int> customIds;
	std::vector<std::array<glm::vec3, 4>> customCorners;

	cv::aruco::Board board;
	cv::aruco::CharucoBoard charuco;
};
