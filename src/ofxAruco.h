#pragma once

// ofxAruco: threaded ArUco / ChArUco marker + board detection and pose
// estimation for openFrameworks, built on OpenCV's objdetect module
// (OpenCV >= 4.7, as shipped with ofxOpenCv).
//
// Minimal use:
//
//   ofxAruco aruco;
//   aruco.setup(cv::aruco::DICT_5X5_100);      // starts the worker thread
//   aruco.loadIntrinsics("intrinsics.yml");    // optional: needed for 3D poses
//   aruco.addBoard(ofxArucoBoard::makeCharuco(7, 5, 0.04f, 0.03f, cv::aruco::DICT_5X5_100));
//
//   update(): if (grabber.isFrameNew()) aruco.detect(grabber.getPixels());
//   draw():   grabber.draw(0, 0); aruco.draw();
//             for (auto & m : aruco.getMarkers()) { ... m.id, m.corners, m.pose ... }
//
// detect() never blocks: the frame is converted to grayscale and handed to a
// worker thread. If the worker is still busy, the pending frame is replaced
// by the newest one (old frames are dropped, latency stays low). Results are
// picked up automatically at the start of every OF frame (before your
// ofApp::update), so getResult() is stable for the whole frame.
//
// Use one ofxAruco per camera; each one has its own thread.
//
// All poses are in the OpenCV camera frame (x right, y down, z forward), in the
// units of your marker/board sizes (use meters). See ofxArucoTypes.h.

#include "ofMain.h"
#include "ofxArucoTypes.h"
#include "ofxArucoIntrinsics.h"
#include "ofxArucoBoard.h"
#include "ofxArucoMultiCamCalibration.h"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

class ofxAruco {
public:
	ofxAruco();
	~ofxAruco();
	ofxAruco(const ofxAruco &) = delete;
	ofxAruco & operator=(const ofxAruco &) = delete;

	// -- setup ---------------------------------------------------------------
	// dictionary: one of cv::aruco::PredefinedDictionaryType (e.g. cv::aruco::DICT_5X5_100).
	// threaded: run detection on a worker thread (recommended). Can be changed later.
	void setup(int dictionary = cv::aruco::DICT_5X5_100, bool threaded = true);
	void setThreaded(bool threaded);
	bool isThreaded() const { return threaded; }

	// Camera intrinsics are required for 3D poses. Without them you still get
	// 2D marker corners and ids.
	void setIntrinsics(const ofxArucoIntrinsics & intrinsics);
	bool loadIntrinsics(const std::string & path);
	const ofxArucoIntrinsics & getIntrinsics() const { return intrinsics; }

	// Boards: returns the board index, or -1 on failure. All boards should use the
	// same dictionary as the detector (a warning is logged otherwise).
	int addBoard(const ofxArucoBoard & board);
	int loadBoard(const std::string & jsonPath);
	void clearBoards();
	const std::vector<ofxArucoBoard> & getBoards() const { return boards; }

	// -- settings (ofParameters: add `aruco.parameters` to an ofxPanel) --------
	ofParameterGroup parameters;
	ofParameter<int> dictionary;              // cv::aruco::PredefinedDictionaryType
	ofReadOnlyParameter<std::string, ofxAruco> dictionaryName;
	ofParameter<float> markerLength;          // side of single markers (meters). Needed for single marker poses
	ofParameter<bool> estimateMarkerPoses;
	ofParameter<bool> estimateBoardPoses;
	ofParameter<bool> refineWithBoards;       // recover missed board markers (cv::aruco::refineDetectedMarkers)
	// Fast mode (ArUco3): much faster on big images (e.g. 4K) but only finds markers
	// whose side is larger than about  aruco3MinSide + aruco3MinMarkerRatio * imageWidth  pixels
	// (the image is downscaled internally when the ratio is > 0).
	ofParameter<bool> useAruco3Detection;
	ofParameter<int> aruco3MinSide;            // pixels, OpenCV minSideLengthCanonicalImg
	ofParameter<float> aruco3MinMarkerRatio;   // 0..1,  OpenCV minMarkerLengthRatioOriginalImg
	ofParameter<int> cornerRefinement;        // 0 none, 1 subpixel, 2 contour, 3 apriltag
	ofParameter<int> adaptiveThreshWinSizeMin;
	ofParameter<int> adaptiveThreshWinSizeMax;
	ofParameter<int> adaptiveThreshWinSizeStep;
	ofParameter<float> adaptiveThreshConstant;
	ofParameter<float> minMarkerPerimeterRate; // relative to the largest image side
	ofParameter<float> maxMarkerPerimeterRate;
	ofParameter<float> polygonalApproxAccuracyRate;
	ofParameter<float> errorCorrectionRate;
	ofParameter<bool> detectInvertedMarker;
	ofParameter<float> shortPixelsScale;      // for 16 bit input (e.g. Kinect IR): 8bit = value * scale. 0 = automatic

	// JSON (ofSerialize of `parameters`). Relative paths use ofToDataPath().
	bool saveSettings(const std::string & path = "aruco_settings.json") const;
	bool loadSettings(const std::string & path = "aruco_settings.json");

	// -- detection -----------------------------------------------------------
	// Any channel count (gray, RGB, RGBA, BGRA, ...). userTag is copied into the
	// result (e.g. a device timestamp, to pair frames from several cameras).
	void detect(const ofPixels & pixels, uint64_t userTag = 0);
	void detect(const ofShortPixels & pixels, uint64_t userTag = 0); // e.g. IR images
	void detect(const cv::Mat & image, uint64_t userTag = 0);        // 8 bit 1, 3 (BGR) or 4 (BGRA) channels

	// Normally called automatically before ofApp::update(). Call it yourself if you
	// detect and read results within the same frame and don't want to wait one frame.
	// Returns true if a new result arrived.
	bool update();
	// Blocks until the worker has processed every submitted frame (threaded mode).
	void waitForResult(uint64_t timeoutMillis = 1000);

	// -- results (main thread) ---------------------------------------------
	const ofxArucoResult & getResult() const { return result; }
	bool isFrameNew() const { return frameNew; }
	const std::vector<ofxArucoMarker> & getMarkers() const { return result.markers; }
	int getNumMarkers() const { return (int)result.markers.size(); }
	// One entry per added board; check .found.
	const std::vector<ofxArucoBoardPose> & getBoardPoses() const { return result.boards; }
	const ofxArucoBoardPose & getBoardPose(int boardIndex = 0) const;
	int getNumBoards() const { return (int)boards.size(); }
	// Detection rate of the worker (results per second) and cost of the last frame.
	float getDetectionFps() const { return detectionFps; }
	float getDetectMillis() const { return result.detectMillis; }

	// -- drawing -----------------------------------------------------------
	// `viewport` is the rectangle where you drew the camera image. Default
	// (empty) = the image at 0,0 with its native size.
	void draw(const ofRectangle & viewport = ofRectangle()) const; // markers + boards + axes
	void drawMarkers(const ofRectangle & viewport = ofRectangle(), bool drawIds = true) const;
	void drawBoards(const ofRectangle & viewport = ofRectangle()) const;
	void drawRejected(const ofRectangle & viewport = ofRectangle()) const;
	void drawAxes(const ofRectangle & viewport = ofRectangle()) const; // 3D axes of every pose

	// 3D overlay. Inside begin*/end you draw in real units on top of the image:
	//   beginCamera(): OpenCV camera frame (x right, y down, z forward)
	//   begin(i):      frame of marker i (origin at center, x right, y up, z towards the camera)
	//   beginBoard(i): frame of board i (origin top-left, x right, y down, z into the board)
	void beginCamera(const ofRectangle & viewport = ofRectangle()) const;
	void begin(int markerIndex, const ofRectangle & viewport = ofRectangle()) const;
	void beginBoard(int boardIndex = 0, const ofRectangle & viewport = ofRectangle()) const;
	void end() const;

	// OpenGL matrices matching beginCamera / begin (view = cvToGl * pose).
	glm::mat4 getProjectionMatrix(float nearDist = 0.01f, float farDist = 100.f) const;
	glm::mat4 getModelViewMatrix(int markerIndex) const;
	glm::mat4 getModelViewMatrixBoard(int boardIndex) const;

	// Printable marker image (white quiet zone included).
	static void getMarkerImage(int dictionary, int markerId, int sizePixels, ofPixels & pixels, int borderBits = 1);

private:
	struct Config;
	struct Worker;

	std::shared_ptr<const Config> makeConfig() const;
	void markConfigDirty();
	void submit(cv::Mat && gray, uint64_t userTag);
	void startThread();
	void stopThread();
	void threadedFunction();
	void onUpdate(ofEventArgs &);
	ofRectangle resolveViewport(const ofRectangle & viewport) const;

	// main thread state
	bool threaded = true;
	bool isSetup = false;
	ofxArucoIntrinsics intrinsics;
	std::vector<ofxArucoBoard> boards;
	std::shared_ptr<const Config> config;
	bool configDirty = true;
	ofxArucoResult result;
	bool frameNew = false;
	float detectionFps = 0;
	uint64_t lastResultTime = 0;
	uint64_t submittedFrames = 0;
	ofEventListeners listeners;

	// shared with the worker (guarded by mutex)
	std::mutex mutex;
	std::condition_variable condition;
	std::condition_variable resultCondition;
	cv::Mat pendingImage;
	uint64_t pendingTag = 0;
	uint64_t pendingTime = 0;
	uint64_t pendingFrameNumber = 0;
	std::shared_ptr<const Config> pendingConfig;
	bool hasPending = false;
	bool workerBusy = false;
	ofxArucoResult latestResult;
	bool hasNewResult = false;
	bool running = false;
	std::thread thread;

	// worker-only state (detectors are rebuilt when the config changes)
	std::unique_ptr<Worker> worker;
	cv::Mat shortScratch;
};
