#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowTitle("ofxAruco - example");
	ofSetVerticalSync(true);
	ofBackground(30);

	// 1. detector: the video uses the original ArUco dictionary.
	//    For your own markers prefer DICT_5X5_100 or DICT_6X6_250 (see example-print-boards).
	aruco.setup(cv::aruco::DICT_ARUCO_ORIGINAL);

	// 2. camera intrinsics (needed for 3D poses). The video was shot at 640x480 with this calibration.
	aruco.loadIntrinsics("intrinsics.yml");

	// 3. side of one printed marker, in meters (needed for single marker poses)
	aruco.markerLength = 0.035f;

	// 4. optional: a board. This one is an old ArUco 1.x layout, scaled so one marker is 3.5cm.
	ofxArucoBoard board;
	if (board.loadLegacyAruco("boardConfiguration.yml", 0.035f, cv::aruco::DICT_ARUCO_ORIGINAL)) {
		aruco.addBoard(board);
	}

	// GUI: ofxAruco exposes all its settings as ofParameters
	gui.setup("settings", "settings.json", 650, 10);
	gui.add(threaded);
	gui.add(drawMarkers);
	gui.add(drawCubes);
	gui.add(drawBoard);
	gui.add(drawRejected);
	gui.add(aruco.parameters);
	gui.getGroup("aruco").minimize();
	if (ofFile::doesFileExist("settings.json")) gui.loadFromFile("settings.json");
	threadedListener = threaded.newListener([this](bool & t) { aruco.setThreaded(t); });
	aruco.setThreaded(threaded);
	aruco.dictionary = cv::aruco::DICT_ARUCO_ORIGINAL; // the video needs this one

	useVideoFile();
}

void ofApp::useVideoFile() {
	grabber.close();
	player.load("videoboard.mp4");
	player.setLoopState(OF_LOOP_NORMAL);
	player.play();
	video = &player;
	usingWebcam = false;
}

void ofApp::useWebcam(int deviceId) {
	player.close();
	grabber.close();
	grabber.setDeviceID(deviceId);
	if (!grabber.setup(640, 480)) {
		ofLogError() << "could not open webcam " << deviceId << ", back to the video file";
		useVideoFile();
		return;
	}
	video = &grabber;
	usingWebcam = true;
	// NOTE: intrinsics.yml is for the camera of the video, not yours. Poses will be
	// approximate until you calibrate your camera and load its intrinsics.
}

//--------------------------------------------------------------
void ofApp::update() {
	video->update();
	if (video->isFrameNew()) {
		// never blocks: the frame is handed to the worker thread
		aruco.detect(video->getPixels());
	}
	// results are picked up automatically every frame, read them in draw()
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofSetColor(255);
	const ofRectangle view(0, 0, 640, 480);
	video->draw(view);

	if (drawRejected) aruco.drawRejected(view);
	if (drawMarkers) aruco.drawMarkers(view);
	if (drawBoard) aruco.drawBoards(view);

	// 3D: begin(i) puts you in marker i's frame (meters, z towards the camera)
	ofEnableDepthTest();
	if (drawCubes) {
		const float s = aruco.markerLength;
		for (int i = 0; i < aruco.getNumMarkers(); i++) {
			if (!aruco.getMarkers()[i].hasPose) continue;
			aruco.begin(i, view);
			ofDrawAxis(s * 0.75f);
			ofTranslate(0, 0, s * 0.5f);
			ofNoFill();
			ofSetColor(255, 255, 0);
			ofDrawBox(s);
			aruco.end();
		}
	}
	if (drawBoard && aruco.getNumBoards() > 0 && aruco.getBoardPose(0).found) {
		// board frame: origin at the top-left of the board, x right, y down, z into the board
		const auto & b = aruco.getBoards()[0];
		aruco.beginBoard(0, view);
		ofDrawAxis(0.1f);
		ofTranslate(b.getCenter());
		ofSetColor(0, 200, 255);
		ofNoFill();
		ofDrawBox(glm::vec3(0, 0, -0.025f), b.getSize().x, b.getSize().y, 0.05f);
		aruco.end();
	}
	ofDisableDepthTest();

	// info
	const auto & r = aruco.getResult();
	std::stringstream ss;
	ss << "source: " << (usingWebcam ? "webcam" : "videoboard.mp4") << "\n"
	   << "markers: " << aruco.getNumMarkers();
	if (aruco.getNumBoards() > 0) {
		const auto & bp = aruco.getBoardPose(0);
		ss << "   board: " << (bp.found ? "found" : "-") << " (" << bp.numPoints << " pts, "
		   << ofToString(bp.reprojectionError, 2) << " px)";
		if (bp.found) {
			ss << "\nboard position (m): " << ofToString(bp.getPosition(), 3);
		}
	}
	ss << "\napp fps: " << ofToString(ofGetFrameRate(), 0)
	   << "   detection: " << ofToString(aruco.getDetectionFps(), 0) << " fps, "
	   << ofToString(r.detectMillis, 1) << " ms"
	   << (aruco.isThreaded() ? " (threaded)" : " (main thread)");
	ofDrawBitmapStringHighlight(ss.str(), 10, 20);

	ofDrawBitmapStringHighlight("v: video file / webcam   0-9: webcam id\n"
								"s: save settings   l: load settings   g: hide gui",
		10, ofGetHeight() - 24);

	if (showGui) gui.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (key == 'v') {
		if (usingWebcam) {
			useVideoFile();
		} else {
			useWebcam(0);
		}
	}
	if (key >= '0' && key <= '9') useWebcam(key - '0');
	if (key == 's') gui.saveToFile("settings.json");
	if (key == 'l' && ofFile::doesFileExist("settings.json")) gui.loadFromFile("settings.json");
	if (key == 'g') showGui = !showGui;
}
