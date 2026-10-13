#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowTitle("ofxAruco - example");
	ofSetVerticalSync(true);
	ofBackground(30);

	// 1. detector: the main dictionary is set by loadBoards(); every other one is found too
	aruco.setup(cv::aruco::DICT_ARUCO_ORIGINAL);
	aruco.anyDictionary = true;

	// 2. camera intrinsics (needed for 3D poses). The video was shot at 640x480 with this calibration.
	aruco.loadIntrinsics("intrinsics.yml");

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

	loadBoards();
	useVideoFile();
}

// Every board in bin/data: the video's old ArUco 1.x board, and every .yml board file
// (example-print-boards' ChArUco boards, grids, cards and single markers). The main
// dictionary is the one of your first board; the others are searched in extra passes.
void ofApp::loadBoards() {
	aruco.clearBoards();
	boardInfo.clear();

	// the video: original ArUco dictionary, 3.5 cm markers
	ofxArucoBoard videoBoard;
	if (videoBoard.loadLegacyAruco("boardConfiguration.yml", 0.035f, cv::aruco::DICT_ARUCO_ORIGINAL)) {
		videoBoard.setName("video board");
		aruco.addBoard(videoBoard);
		boardInfo.push_back("video board (ARUCO_ORIGINAL, 35 mm markers)");
	}

	ofDirectory dir(ofToDataPath("", true));
	dir.allowExt("yml");
	dir.allowExt("yaml");
	dir.listDir();
	dir.sort();
	int mainDictionary = -1;
	for (auto & file : dir) {
		const std::string name = file.getFileName();
		if (name == "boardConfiguration.yml" || ofIsStringInString(name, "intrinsics")) continue;
		const ofJson j = ofxArucoUtils::loadYaml(file.getAbsolutePath());
		if (!j.is_object() || (!j.contains("type") && !j.contains("aruco_bc_markers"))) continue; // not a board
		ofxArucoBoard board;
		if (!board.load(name)) continue;
		board.setName(file.getBaseName());

		// boards that share markers (same dictionary, same ids): single markers take the
		// first one's size, and every one of them reports a pose
		const int family = ofxArucoUtils::getDictionaryFamily(board.getDictionary());
		const auto & ids = board.getCvBoard().getIds();
		for (auto & other : aruco.getBoards()) {
			if (ofxArucoUtils::getDictionaryFamily(other.getDictionary()) != family) continue;
			const auto & otherIds = other.getCvBoard().getIds();
			const bool shared = std::any_of(ids.begin(), ids.end(), [&](int id) {
				return std::find(otherIds.begin(), otherIds.end(), id) != otherIds.end();
			});
			if (shared) boardInfo.push_back("  ! " + board.getName() + " uses the same markers as " + other.getName());
		}

		aruco.addBoard(board);
		if (mainDictionary < 0) mainDictionary = board.getDictionary();
		boardInfo.push_back(board.getName() + " (" + board.getTypeName() + ", " + ofToString(board.getNumMarkers()) + " markers, "
			+ ofToString(board.getMarkerLength() * 1000.f, 0) + " mm)");
	}
	if (mainDictionary >= 0) aruco.dictionary = mainDictionary;
	if (boardInfo.empty()) boardInfo.push_back("no boards in bin/data: markers only");
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
		for (int i = 0; i < aruco.getNumMarkers(); i++) {
			const auto & m = aruco.getMarkers()[i];
			if (!m.hasPose) continue;
			const float s = m.length; // this marker's printed side
			aruco.begin(i, view);
			ofDrawAxis(s * 0.75f);
			ofTranslate(0, 0, s * 0.5f);
			ofNoFill();
			ofSetColor(255, 255, 0);
			ofDrawBox(s);
			aruco.end();
		}
	}
	if (drawBoard) {
		// board frame: origin at the top-left of the board, x right, y down, z into the board
		for (int i = 0; i < aruco.getNumBoards(); i++) {
			if (!aruco.getBoardPose(i).found) continue;
			const auto & b = aruco.getBoards()[i];
			aruco.beginBoard(i, view);
			ofDrawAxis(0.1f);
			ofTranslate(b.getCenter());
			ofSetColor(0, 200, 255);
			ofNoFill();
			ofDrawBox(glm::vec3(0, 0, -0.025f), b.getSize().x, b.getSize().y, 0.05f);
			aruco.end();
		}
	}
	ofDisableDepthTest();

	// info
	const auto & r = aruco.getResult();
	std::stringstream ss;
	ss << "source: " << (usingWebcam ? "webcam" : "videoboard.mp4") << "\n"
	   << "main dictionary: " << aruco.dictionaryName.get() << (aruco.anyDictionary ? " + any dictionary" : "")
	   << "   markers: " << aruco.getNumMarkers() << "\n";
	int shown = 0;
	for (auto & m : aruco.getMarkers()) {
		if (shown++ == 8) {
			ss << "  ...\n";
			break;
		}
		ss << "  " << m.id << " " << ofxArucoUtils::dictionaryToShortName(m.dictionary) << "  " << ofToString(m.length * 1000.f, 0) << " mm";
		if (m.hasPose) ss << "  at " << ofToString(m.getPosition(), 3) << " m";
		ss << "\n";
	}
	ss << "boards:\n";
	for (size_t i = 0; i < boardInfo.size(); i++) ss << "  " << boardInfo[i] << "\n";
	for (int i = 0; i < aruco.getNumBoards(); i++) {
		const auto & bp = aruco.getBoardPose(i);
		if (!bp.found) continue;
		ss << "  found " << aruco.getBoards()[i].getName() << " (" << bp.numPoints << " pts, "
		   << ofToString(bp.reprojectionError, 2) << " px) at " << ofToString(bp.getPosition(), 3) << " m\n";
	}
	ss << "app fps: " << ofToString(ofGetFrameRate(), 0)
	   << "   detection: " << ofToString(aruco.getDetectionFps(), 0) << " fps, "
	   << ofToString(r.detectMillis, 1) << " ms"
	   << (aruco.isThreaded() ? " (threaded)" : " (main thread)");
	ofDrawBitmapStringHighlight(ss.str(), 10, 20);

	ofDrawBitmapStringHighlight("v: video file / webcam   0-9: webcam id   a: any dictionary on/off   r: reload boards\n"
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
	if (key == 'a') aruco.anyDictionary = !aruco.anyDictionary;
	if (key == 'r') loadBoards();
	if (key == 's') gui.saveToFile("settings.json");
	if (key == 'l' && ofFile::doesFileExist("settings.json")) gui.loadFromFile("settings.json");
	if (key == 'g') showGui = !showGui;
}
