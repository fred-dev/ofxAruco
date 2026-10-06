#pragma once

// Camera intrinsic calibration (focal length, principal point, lens distortion)
// with a ChArUco board, plus a plan to calibrate every mode of a camera.
//
// ofxArucoCalibrator: feed it frames, it keeps the views that are still,
// detected well and different from the views it already has, then solves.
//
//   calibrator.setup(board);                     // a ChArUco ofxArucoBoard
//   calibrator.start({ 1920, 1080 });            // new resolution, clears samples
//   update(): if (grabber.isFrameNew()) calibrator.update(grabber.getPixels());
//   draw():   grabber.draw(r); calibrator.draw(r);
//   if (calibrator.getNumSamples() >= 20 && calibrator.getCoverage() > 0.7) {
//       auto result = calibrator.calibrate();
//       ofxArucoCalibrator::save("1920x1080.yml", result, { { "camera_name", "..." } });
//   }
//
// Why ChArUco: corners are found even when part of the board is outside the
// image, so you can push the board into the image corners, where lens
// distortion is strongest and a plain chessboard would not be detected.
//
// ofxArucoCalibrationPlan: given the modes (resolutions) of a camera, decides
// what to calibrate. Modes with the same aspect ratio are usually the same
// sensor area scaled, so only the largest one of each aspect ratio gets a full
// calibration; the others get a short verification (a few views, solving only
// focal length and principal point against the scaled intrinsics). If they
// match, the scaled intrinsics are saved; if not (the mode crops the sensor),
// that mode gets a full calibration too.
//
// Files: <folder>/<prefix><width>x<height>.yml (prefix: the camera name, e.g.
// Logitech_BRIO_1920x1080.yml), OpenCV calibration format
// (camera_matrix, distortion_coefficients, image_width, image_height), plus
// <folder>/index.yml. Load the right one with
// ofxArucoIntrinsics::loadForResolution(folder, width, height).

#include "ofMain.h"
#include "ofxArucoBoard.h"
#include "ofxArucoIntrinsics.h"
#include <opencv2/objdetect/charuco_detector.hpp>
#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

class ofxArucoCalibrator {
public:
	struct Detection {
		std::vector<cv::Point2f> corners; // ChArUco corners, pixels
		std::vector<int> ids;
		cv::Size imageSize;
	};

	struct Sample {
		std::vector<cv::Point2f> imagePoints;
		std::vector<cv::Point3f> objectPoints; // board frame, board units (meters)
		std::array<float, 5> signature {};     // center x, y, scale, tilt x, tilt y (to compare views)
		double error = -1;                     // reprojection error after calibrate(), pixels
	};

	struct Result {
		bool ok = false;
		ofxArucoIntrinsics intrinsics;
		double rms = 0;          // reprojection error, pixels (below 0.5 is good, above 1 is suspect)
		int numSamples = 0;
		int removedSamples = 0;  // outliers dropped
		std::string message;
	};

	struct Settings {
		int minCorners = 12;        // ChArUco corners needed in one view
		float stillPixels = 1.0f;   // max mean corner motion between detections, px per 1000 px of image width
		float stillSeconds = 0.5f;  // the board must be still this long before a view is taken
		float minNovelty = 0.12f;   // how different a view must be from all accepted ones (0..1+)
		float minInterval = 0.4f;   // seconds between two samples
		int coverageCols = 8;
		int coverageRows = 6;
		bool autoCapture = true;
	};

	ofxArucoCalibrator() = default;
	~ofxArucoCalibrator();
	ofxArucoCalibrator(const ofxArucoCalibrator &) = delete;
	ofxArucoCalibrator & operator=(const ofxArucoCalibrator &) = delete;

	// The board must be a ChArUco board (sizes in meters).
	bool setup(const ofxArucoBoard & board);
	const ofxArucoBoard & getBoard() const { return board; }

	// Starts a calibration at this resolution: clears samples.
	void start(cv::Size imageSize);
	void clear();
	cv::Size getImageSize() const { return imageSize; }

	// Call with every new camera frame. Detection runs on a worker thread, this
	// never blocks. Returns true when a sample was added.
	bool update(const ofPixels & pixels);
	// Adds the current detection as a sample now, if it has enough corners.
	bool captureNow();
	void removeLastSample();

	int getNumSamples() const { return int(samples.size()); }
	const std::vector<Sample> & getSamples() const { return samples; }
	// Fraction (0..1) of the coverage grid cells that contain corners of a sample.
	float getCoverage() const;
	// Fraction of samples where the board is clearly tilted.
	float getTiltedFraction() const;
	const Detection & getDetection() const { return detection; }
	bool isStill() const;
	// What happens now / what the user should do, one line.
	const std::string & getStatus() const { return status; }

	// Full calibration (fx, fy, cx, cy, k1, k2, p1, p2, k3). flags: cv::CALIB_*
	// (e.g. cv::CALIB_RATIONAL_MODEL for very wide lenses). With removeOutliers,
	// views with a much larger error than the others are dropped and it solves again.
	Result calibrate(int flags = 0, bool removeOutliers = true);
	// Only fx, fy, cx, cy, starting from `guess`, distortion fixed to the guess.
	// Needs far fewer views than calibrate(): used to verify scaled intrinsics.
	Result refineFocal(const ofxArucoIntrinsics & guess) const;

	// Draws coverage, accepted corners and the current detection over the camera
	// image drawn in `viewport`.
	void draw(const ofRectangle & viewport) const;

	// Saves in the OpenCV calibration format, plus the key/values in `info`.
	static bool save(const std::string & path, const Result & result, const std::map<std::string, std::string> & info = {});

	Settings settings;

private:
	struct Worker;
	void stopWorker();
	bool makeSample(const Detection & d, Sample & s) const;
	float novelty(const Sample & s) const;

	ofxArucoBoard board;
	cv::Size imageSize;
	std::vector<Sample> samples;
	Detection detection;
	Detection previousDetection;
	float stillSince = -1;
	float lastSampleTime = -1000;
	std::string status = "show the board to the camera";
	std::unique_ptr<Worker> worker;
};

//--------------------------------------------------------------
struct ofxArucoCameraMode {
	int width = 0;
	int height = 0;
	float fps = 0; // max frame rate (0 = unknown)
	std::string getName() const { return ofToString(width) + "x" + ofToString(height); }
};

class ofxArucoCalibrationPlan {
public:
	enum class Kind { Full, Verify };
	enum class Status { Pending, Calibrated, Scaled, Failed, Skipped };

	struct Task {
		ofxArucoCameraMode mode;
		Kind kind = Kind::Full;
		Status status = Status::Pending;
		int group = 0;         // modes with the same aspect ratio
		double rms = 0;
		int samples = 0;
		std::string note;
		std::string fileName;  // <prefix><width>x<height>.yml
		std::string getFileName() const { return fileName.empty() ? mode.getName() + ".yml" : fileName; }
	};

	// Groups the modes by aspect ratio, largest first. Duplicates are removed.
	// File names are <filePrefix><width>x<height>.yml, e.g. "Logitech_BRIO_" -> Logitech_BRIO_1920x1080.yml
	void build(const std::vector<ofxArucoCameraMode> & modes, const std::string & filePrefix = "");
	// Marks tasks whose .yml already exists in `folder` as done (to resume a long session).
	void resumeFrom(const std::string & folder);
	// The full calibration of the task's group, if done (to scale from / verify against).
	const Task * getReference(const Task & task) const;
	// For a Verify task: the reference intrinsics scaled to this mode.
	bool getScaledReference(const Task & task, const std::string & folder, ofxArucoIntrinsics & out) const;

	bool saveIndex(const std::string & folder, const std::map<std::string, std::string> & info) const;

	static std::string toString(Kind k) { return k == Kind::Full ? "full" : "verify"; }
	static std::string toString(Status s);
	static std::string aspectName(int w, int h);

	std::vector<Task> tasks;
};
