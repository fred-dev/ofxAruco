#pragma once

// Pinhole camera intrinsics + lens distortion, in OpenCV convention.
//
// Ways to fill it:
//   intrinsics.load("intrinsics.yml");   // OpenCV calibration file (.yml/.yaml/.xml),
//                                        // ofxAruco .json, or legacy ArUco .int
//   intrinsics.setup(fx, fy, cx, cy, width, height, { k1, k2, p1, p2, k3, k4, k5, k6 });
//
// Azure Kinect: k4a_calibration_t::color_camera_calibration.intrinsics.parameters.param
// gives fx, fy, cx, cy, k1..k6, p1, p2. Pass the distortion in OpenCV order:
//   { k1, k2, p1, p2, k3, k4, k5, k6 }
// and width/height = color_camera_calibration.resolution_width/height.

#include "ofMain.h"
#include <opencv2/core.hpp>

class ofxArucoIntrinsics {
public:
	ofxArucoIntrinsics() = default;

	void setup(double fx, double fy, double cx, double cy, int width, int height,
		const std::vector<double> & distortion = {});
	void setup(const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs, cv::Size imageSize);

	// Loads .yml / .yaml / .xml (OpenCV FileStorage), .json or legacy .int.
	// Relative paths are resolved with ofToDataPath().
	bool load(const std::string & path);
	// Saves .json, or .yml/.xml through OpenCV FileStorage.
	bool save(const std::string & path) const;

	ofJson toJson() const;
	bool fromJson(const ofJson & json);

	bool isValid() const { return !cameraMatrix.empty() && imageSize.width > 0 && imageSize.height > 0; }

	// Intrinsics for the same lens at another resolution (same aspect ratio expected),
	// e.g. when the camera calibration was done at 1920x1080 and you detect at 960x540.
	ofxArucoIntrinsics getScaled(int width, int height) const;

	// OpenGL projection matrix that reproduces this camera when the image is
	// drawn in a viewport of any size (aspect ratio of the image is assumed).
	// Use with a view matrix of ofxArucoUtils::cvToGl() so you can draw in
	// OpenCV camera coordinates.
	glm::mat4 getProjectionMatrix(float nearDist = 0.01f, float farDist = 100.f) const;

	// Draw "through" this camera: between begin() and end() you draw in the OpenCV
	// camera frame (x right, y down, z forward, your units) and it lands on the
	// same pixels as the real camera image drawn in `viewport`.
	// Works on screen and inside an ofFbo.
	void begin(const ofRectangle & viewport, float nearDist = 0.01f, float farDist = 100.f) const;
	void end() const;

	double getFx() const;
	double getFy() const;
	double getCx() const;
	double getCy() const;
	int getWidth() const { return imageSize.width; }
	int getHeight() const { return imageSize.height; }
	// Horizontal / vertical field of view in degrees.
	glm::vec2 getFov() const;

	const cv::Mat & getCameraMatrix() const { return cameraMatrix; }
	const cv::Mat & getDistCoeffs() const { return distCoeffs; }
	cv::Size getImageSize() const { return imageSize; }

	// Projects a 3D point (camera frame, OpenCV convention) to pixels, with distortion.
	glm::vec2 project(const glm::vec3 & pointInCamera) const;

private:
	cv::Mat cameraMatrix; // 3x3 CV_64F
	cv::Mat distCoeffs;   // 1xN CV_64F (N = 0, 4, 5, 8, 12 or 14)
	cv::Size imageSize;
};
