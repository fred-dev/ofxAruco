#pragma once

// Extrinsic calibration of several cameras that look at the same board.
//
// How it works: put a board (ChArUco recommended) where 2 or more cameras can
// see it, hold it STILL, take a sample. Move it, repeat (10-30 samples, different
// positions/angles, filling the shared volume). Each sample gives, for every pair
// of cameras that saw the board:  cameraJ -> cameraI = boardToCamI * inverse(boardToCamJ).
// Those are averaged (with outlier rejection) and chained through the cameras
// to the reference camera, so cameras that never see the board together with
// the reference can still be calibrated through a camera in between.
// Finally everything is refined together (bundle adjustment): camera poses and
// board poses are optimized to minimize the reprojection error of every board
// corner in every camera. This is what makes the result accurate.
//
// Result: for every camera, cameraToWorld (4x4, meters, OpenCV camera frame):
//     p_world = cameraToWorld * p_camera
// World = reference camera by default, or the board frame with setWorldFromBoard().
//
// Depth cameras (Azure Kinect): detect on the COLOR image (or the IR image for
// the depth camera itself). Points from the depth camera must first be moved to
// the color camera frame with the device's own depth->color extrinsics, then
// multiplied by cameraToWorld (remember k4a uses millimeters).
//
// Typical use:
//   calib.setup(2);
//   every frame:  calib.update({ &arucoA, &arucoB });  // auto capture when still
//   or on a key:  calib.addSample({ &arucoA, &arucoB });
//   calib.solve(); calib.save("calibration.json");

#include "ofMain.h"
#include "ofxArucoTypes.h"
#include "ofxArucoIntrinsics.h"

class ofxAruco;

class ofxArucoMultiCamCalibration {
public:
	struct Observation {
		bool valid = false;
		glm::mat4 boardToCamera{1.f};
		float reprojectionError = 0;
		int numPoints = 0;
		// Optional, enables the final bundle adjustment (filled automatically when
		// sampling from ofxAruco detectors): the board corners used for the pose
		// and the intrinsics of the camera at the image size they were found in.
		std::vector<glm::vec3> objectPoints;
		std::vector<glm::vec2> imagePoints;
		ofxArucoIntrinsics intrinsics;
	};

	struct CameraResult {
		std::string name;
		bool solved = false;
		int samples = 0;              // samples on the link used to reach this camera
		int linkedTo = -1;            // camera it was chained from (-1 for the reference)
		float rotationStdDeg = 0;     // spread of the samples on that link (before refinement)
		float translationStdMm = 0;
		bool refined = false;         // bundle adjusted
		float reprojectionError = 0;  // RMS pixels of all board corners after refinement: the quality number to watch
		glm::mat4 cameraToReference{1.f};
	};

	ofxArucoMultiCamCalibration();

	void setup(int numCameras, int referenceCamera = 0);
	void setCameraName(int camera, const std::string & name);
	int getNumCameras() const { return (int)cameras.size(); }
	int getReferenceCamera() const { return referenceCamera; }

	// --- sampling ------------------------------------------------------------
	// One observation per camera (invalid when the camera didn't see the board).
	// Returns false if fewer than 2 cameras saw the board or the quality checks fail.
	bool addSample(const std::vector<Observation> & observations);
	// Uses the latest result of every detector (one per camera, same order as setup()).
	bool addSample(const std::vector<const ofxAruco *> & detectors, int boardIndex = 0);
	// Call once per frame. With autoCapture on, adds a sample whenever the board
	// has been still for `stillTime` in every camera that sees it, and has moved
	// enough since the last sample. Returns true when a sample was added.
	bool update(const std::vector<const ofxAruco *> & detectors, int boardIndex = 0);
	// 0..1 progress of the "hold still" timer (for UI feedback).
	float getStillProgress() const { return stillProgress; }
	int getNumSamples() const { return (int)samples.size(); }
	void clear();
	void removeLastSample();

	// --- solving -------------------------------------------------------------
	// Recomputes the transforms from all samples. Called automatically after every
	// added sample. Returns true when every camera is connected to the reference.
	bool solve();
	bool isSolved() const;
	bool isSolved(int camera) const;
	const CameraResult & getCamera(int camera) const { return cameras[camera]; }

	glm::mat4 getCameraToReference(int camera) const;
	glm::mat4 getCameraToWorld(int camera) const;
	// Transform that maps points of camera `from` into camera `to`.
	glm::mat4 getCameraToCamera(int from, int to) const;

	// World frame. Default: the reference camera.
	void setWorldToReferenceCamera();
	// World = board frame, as seen by `camera` (e.g. board lying on the floor at the
	// origin of your scene). Board frame: x right, y down the board, z into the board.
	bool setWorldFromBoard(int camera, const glm::mat4 & boardToCamera);
	const glm::mat4 & getReferenceToWorld() const { return referenceToWorld; }

	// --- persistence (JSON) --------------------------------------------------
	bool save(const std::string & path = "multicam_calibration.json") const;
	bool load(const std::string & path = "multicam_calibration.json");
	ofJson toJson() const;
	bool fromJson(const ofJson & json);

	// --- settings ------------------------------------------------------------
	ofParameterGroup parameters;
	ofParameter<bool> autoCapture;
	ofParameter<float> stillTime;           // seconds the board must not move
	ofParameter<float> stillTranslationMm;  // max motion between frames to count as still
	ofParameter<float> stillRotationDeg;
	ofParameter<float> minMoveMm;           // the board must move this much between samples...
	ofParameter<float> minMoveDeg;          // ...or rotate this much
	ofParameter<float> maxReprojectionError; // pixels, reject noisy board poses
	ofParameter<int> minPoints;             // minimum board corners per camera
	ofParameter<bool> bundleAdjust;         // refine everything by minimizing the reprojection error of all corners

private:
	void refine(); // bundle adjustment of all camera poses and board poses
	struct Sample {
		std::vector<Observation> observations;
	};
	struct PairEstimate {
		bool valid = false;
		int count = 0;
		glm::mat4 jToI{1.f};
		float rotationStdDeg = 0;
		float translationStdMm = 0;
	};
	PairEstimate estimatePair(int i, int j) const;
	std::vector<Observation> collect(const std::vector<const ofxAruco *> & detectors, int boardIndex) const;

	int referenceCamera = 0;
	std::vector<CameraResult> cameras;
	std::vector<Sample> samples;
	glm::mat4 referenceToWorld{1.f};

	// auto capture state
	struct CamTrack {
		bool seen = false;
		glm::mat4 last{1.f};
		uint64_t lastFrame = 0;
		float stillSince = -1;
	};
	std::vector<CamTrack> tracks;
	std::vector<Observation> lastCaptured;
	float stillProgress = 0;
};
