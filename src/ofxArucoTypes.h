#pragma once

// Shared types for ofxAruco.
//
// COORDINATE CONVENTION (read this once, it applies everywhere):
//   All poses are expressed in the OpenCV camera frame:
//       x -> right, y -> down, z -> forward (out of the lens)
//   Units are whatever you used for marker / board sizes. Use METERS.
//   This is the same convention the Azure Kinect SDK uses for its color,
//   depth and IR cameras (k4a uses millimeters: multiply by 0.001).
//
//   A "pose" is a rigid transform objectToCamera:
//       p_camera = pose * p_object
//   so pose[3] (the translation column) is the object origin seen from the camera.

#include "ofMain.h"
#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <array>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// One detected square marker.
struct ofxArucoMarker {
	int id = -1;
	// Image corners in pixels, clockwise starting at the marker's top-left
	// (as printed): top-left, top-right, bottom-right, bottom-left.
	std::array<glm::vec2, 4> corners;

	// Pose is only valid when the detector has camera intrinsics
	// and a marker length > 0.
	bool hasPose = false;
	glm::mat4 pose{1.f};     // marker -> camera. Marker frame: origin at center, x right, y up, z out of the marker (towards the camera)
	cv::Vec3d rvec, tvec;    // same pose as OpenCV Rodrigues + translation
	float reprojectionError = 0.f; // RMS, pixels

	glm::vec2 getCenter() const { return (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f; }
	glm::vec3 getPosition() const { return glm::vec3(pose[3]); }
};

// ---------------------------------------------------------------------------
// Result of looking for one board (grid / ChArUco / custom) in one frame.
struct ofxArucoBoardPose {
	int boardIndex = -1;
	std::string name;
	bool found = false;          // true when the pose was estimated
	int numPoints = 0;           // ChArUco corners (charuco) or marker corners (grid/custom) used for the pose
	glm::mat4 pose{1.f};         // board -> camera (board frame is the one OpenCV defines for the board type)
	cv::Vec3d rvec, tvec;
	float reprojectionError = 0.f; // RMS, pixels

	// The 2D/3D correspondences used for the pose. Handy for drawing and
	// for feeding your own calibration routines.
	std::vector<glm::vec2> imagePoints;
	std::vector<glm::vec3> objectPoints;
	std::vector<int> charucoIds; // only for ChArUco boards

	glm::vec3 getPosition() const { return glm::vec3(pose[3]); }
};

// ---------------------------------------------------------------------------
// Everything found in one processed frame.
struct ofxArucoResult {
	uint64_t frameNumber = 0;  // increments for every processed frame
	uint64_t userTag = 0;      // whatever you passed to detect(), e.g. a device timestamp
	uint64_t timeMicros = 0;   // ofGetElapsedTimeMicros() when the frame was submitted
	float detectMillis = 0.f;  // time spent in detection on the worker thread
	glm::ivec2 imageSize{0, 0};

	std::vector<ofxArucoMarker> markers;
	std::vector<ofxArucoBoardPose> boards;
	std::vector<std::array<glm::vec2, 4>> rejected; // rejected candidates, useful when tuning

	const ofxArucoMarker * findMarker(int id) const {
		for (auto & m : markers) if (m.id == id) return &m;
		return nullptr;
	}
};

// ---------------------------------------------------------------------------
// Small helpers shared by the addon (and handy in apps).
namespace ofxArucoUtils {
	// Names of OpenCV's predefined dictionaries, e.g. "DICT_5X5_100".
	const std::vector<std::string> & getDictionaryNames();
	// Name <-> enum. Returns -1 / "" if unknown.
	int dictionaryFromName(const std::string & name);
	std::string dictionaryToName(int dictionary);
	cv::aruco::Dictionary getDictionary(int dictionary);

	// Rodrigues rvec/tvec <-> 4x4 matrix (OpenCV camera convention).
	glm::mat4 toMat4(const cv::Vec3d & rvec, const cv::Vec3d & tvec);
	void fromMat4(const glm::mat4 & m, cv::Vec3d & rvec, cv::Vec3d & tvec);

	// Converts between the OpenCV camera frame (y down, z forward) and the
	// OpenGL / openFrameworks camera frame (y up, z backward). It is its own inverse.
	glm::mat4 cvToGl();

	// Rigid transform helpers.
	glm::mat4 rigidInverse(const glm::mat4 & m);
	// Angle in degrees of the rotation between two transforms.
	float rotationAngleDeg(const glm::mat4 & a, const glm::mat4 & b);

	// JSON helpers: 4x4 matrix as 4 rows of 4 numbers (row-major, human readable).
	ofJson toJson(const glm::mat4 & m);
	glm::mat4 mat4FromJson(const ofJson & j);
	// Like ofSavePrettyJson, but arrays of numbers stay on one line (readable matrices).
	bool saveJson(const std::string & path, const ofJson & json);

	// YAML in the OpenCV FileStorage dialect (%YAML:1.0, the format of OpenCV
	// calibration files and ArUco board files), going through ofJson so the same
	// code reads and writes both formats. Booleans are written as 0 / 1.
	// commentLines are written as "# ..." under the header.
	bool saveYaml(const std::string & path, const ofJson & json, const std::vector<std::string> & commentLines = {});
	// Returns null if the file can't be read.
	ofJson loadYaml(const std::string & path);
	// Reads a bool written as true/false or 0/1 (YAML files use 0/1).
	bool getBool(const ofJson & json, const std::string & key, bool defaultValue);

	// Saves a PNG that remembers its DPI, so printing it at "100% / actual size"
	// gives the right physical size (plain ofImage::save() PNGs print at 72 dpi).
	bool savePngWithDpi(const ofPixels & pixels, const std::string & path, float dpi);
}
