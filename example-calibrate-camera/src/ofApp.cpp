#include "ofApp.h"

using Plan = ofxArucoCalibrationPlan;

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowTitle("ofxAruco - calibrate camera (class compliant)");
	ofSetVerticalSync(true);
	ofSetEscapeQuitsApp(false);
	ofBackground(25);

	// the printed board
	if (!board.load("board.yml")) {
		board = ofxArucoBoard::makeCharuco(7, 5, 0.036f, 0.027f, cv::aruco::DICT_5X5_100);
		board.setName("charuco_7x5_36mm_27mm_DICT_5X5_100_id0");
		board.save("board.yml");
		ofLogNotice() << "created bin/data/board.yml";
	}
	if (!calibrator.setup(board)) {
		boardError = "bin/data/board.yml must be a ChArUco board (make one with ofxAruco/example-print-boards)";
	}
	refreshCameras();
}

void ofApp::refreshCameras() {
	cameras = listCameraDevices();
	assignLabels(cameras);
	selected = ofClamp(selected, 0, std::max(0, int(cameras.size()) - 1));
	if (!cameras.empty()) buildPlan(cameras[selected], true);
	for (auto & c : cameras) {
		ofLogNotice() << c.grabberIndex << ": " << c.name << " (" << c.modes.size() << " modes) id " << c.uniqueId;
	}
}

void ofApp::buildPlan(const CameraDevice & camera, bool resume) {
	folder = calibrationFolder(camera);
	plan.build(camera.modes, camera.label + "_"); // files: <camera>_<width>x<height>.yml
	if (resume) plan.resumeFrom(folder);
}

void ofApp::say(const std::string & text) {
	message = text;
	ofLogNotice() << text;
}

//--------------------------------------------------------------
void ofApp::startCamera(bool resume) {
	if (cameras.empty() || !boardError.empty()) return;
	buildPlan(cameras[selected], resume);
	if (plan.tasks.empty()) {
		say("this camera reports no modes");
		return;
	}
	screen = Screen::Running;
	current = -1;
	nextTask();
}

void ofApp::nextTask() {
	for (int i = current + 1; i < int(plan.tasks.size()); i++) {
		if (plan.tasks[i].status == Plan::Status::Pending) {
			startTask(i);
			return;
		}
	}
	grabber.close();
	saveIndex();
	screen = Screen::Summary;
	current = -1;
}

void ofApp::startTask(int index) {
	current = index;
	auto & task = plan.tasks[index];
	// a verification needs the full calibration of its group
	if (task.kind == Plan::Kind::Verify && !plan.getReference(task)) {
		task.kind = Plan::Kind::Full;
		task.note = "no reference for this aspect ratio: full calibration";
	}
	const auto & camera = cameras[selected];
	grabber.close();
	grabber.setDeviceID(camera.grabberIndex);
	if (!grabber.setup(task.mode.width, task.mode.height)) {
		task.status = Plan::Status::Failed;
		task.note = "could not open this mode";
		say(task.mode.getName() + ": could not open this mode");
		nextTask();
		return;
	}
	framesSinceOpen = 0;
	paused = false;
	calibrator.start(cv::Size(task.mode.width, task.mode.height));
	say(task.mode.getName() + ": " + (task.kind == Plan::Kind::Full ? "full calibration" : "verification"));
}

void ofApp::finishTask() {
	if (current < 0) return;
	auto & task = plan.tasks[current];
	const auto & camera = cameras[selected];
	std::map<std::string, std::string> info = {
		{ "camera_name", camera.name },
		{ "camera_id", camera.uniqueId },
		{ "camera_model", camera.model },
		{ "board", board.getName() },
	};
	const std::string path = ofFilePath::join(folder, task.getFileName());

	if (task.kind == Plan::Kind::Verify) {
		ofxArucoIntrinsics scaled;
		const Plan::Task * ref = plan.getReference(task);
		if (!ref || !plan.getScaledReference(task, folder, scaled)) {
			task.kind = Plan::Kind::Full;
			say("reference missing: switching to a full calibration");
			return;
		}
		auto result = calibrator.refineFocal(scaled);
		if (!result.ok) {
			say(result.message);
			return;
		}
		const auto & r = result.intrinsics;
		const double dfx = std::abs(r.getFx() - scaled.getFx()) / scaled.getFx();
		const double dfy = std::abs(r.getFy() - scaled.getFy()) / scaled.getFy();
		const double dcx = std::abs(r.getCx() - scaled.getCx()) / task.mode.width;
		const double dcy = std::abs(r.getCy() - scaled.getCy()) / task.mode.height;
		const double worst = std::max({ dfx, dfy, dcx, dcy });
		if (worst <= verifyTolerance && result.rms <= maxRms * 1.5) {
			ofxArucoCalibrator::Result out;
			out.ok = true;
			out.intrinsics = scaled;
			out.rms = result.rms;
			out.numSamples = result.numSamples;
			info["calibration_method"] = "scaled";
			info["scaled_from"] = ref->getFileName();
			info["verify_max_difference"] = ofToString(worst * 100, 2) + "%";
			ofxArucoCalibrator::save(path, out, info);
			task.status = Plan::Status::Scaled;
			task.rms = result.rms;
			task.samples = result.numSamples;
			task.note = "verified against " + ref->mode.getName() + " (max diff " + ofToString(worst * 100, 2) + "%)";
			say(task.mode.getName() + ": " + task.note);
			saveIndex();
			nextTask();
		} else {
			// this mode is not just a scaled version: calibrate it fully, keep the views
			task.kind = Plan::Kind::Full;
			task.note = "differs from " + ref->mode.getName() + " by " + ofToString(worst * 100, 1) + "% (cropped mode?): full calibration";
			say(task.mode.getName() + ": " + task.note);
		}
		return;
	}

	auto result = calibrator.calibrate();
	if (!result.ok) {
		say(result.message);
		return;
	}
	info["calibration_method"] = "calibrated";
	ofxArucoCalibrator::save(path, result, info);
	task.status = Plan::Status::Calibrated;
	task.rms = result.rms;
	task.samples = result.numSamples;
	task.note = result.message + (result.rms > maxRms ? "  HIGH ERROR: check the board is flat, delete the .yml and run again" : "");
	say(task.mode.getName() + ": " + task.note);
	saveIndex();
	nextTask();
}

void ofApp::saveIndex() {
	if (cameras.empty()) return;
	const auto & camera = cameras[selected];
	plan.saveIndex(folder, { { "camera_name", camera.name }, { "camera_id", camera.uniqueId }, { "camera_model", camera.model }, { "board", board.getName() } });
}

//--------------------------------------------------------------
void ofApp::update() {
	if (screen != Screen::Running || current < 0) return;
	grabber.update();
	if (!grabber.isFrameNew()) return;
	auto & task = plan.tasks[current];
	framesSinceOpen++;
	if (framesSinceOpen == 1 && (int(grabber.getWidth()) != task.mode.width || int(grabber.getHeight()) != task.mode.height)) {
		task.status = Plan::Status::Failed;
		task.note = "the camera delivered " + ofToString(grabber.getWidth()) + "x" + ofToString(grabber.getHeight());
		say(task.mode.getName() + ": " + task.note);
		nextTask();
		return;
	}
	if (framesSinceOpen < warmupFrames || paused) return;
	calibrator.update(grabber.getPixels());

	const int n = calibrator.getNumSamples();
	const float coverage = calibrator.getCoverage();
	if (task.kind == Plan::Kind::Full) {
		// done when there are enough views covering the image (or plenty of views anyway)
		if ((n >= fullViews && coverage >= fullCoverage) || n >= fullViews * 2) finishTask();
	} else {
		if (n >= verifyViews && coverage >= verifyCoverage) finishTask();
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	switch (screen) {
	case Screen::Cameras: drawCameras(); break;
	case Screen::Running: drawRunning(); break;
	case Screen::Summary: drawSummary(); break;
	}
}

static std::string statusLabel(const Plan::Task & t) {
	return Plan::toString(t.status);
}

void ofApp::drawPlan(float x, float y, int highlight) {
	ofDrawBitmapString("modes (largest of each aspect ratio = full, others = verify)", x, y);
	y += 20;
	int lastGroup = -1;
	for (int i = 0; i < int(plan.tasks.size()); i++) {
		const auto & t = plan.tasks[i];
		if (t.group != lastGroup) {
			lastGroup = t.group;
			ofSetColor(150);
			ofDrawBitmapString(Plan::aspectName(t.mode.width, t.mode.height), x, y);
			y += 16;
		}
		ofColor c = t.status == Plan::Status::Calibrated ? ofColor(80, 220, 80)
			: t.status == Plan::Status::Scaled           ? ofColor(80, 200, 255)
			: t.status == Plan::Status::Failed           ? ofColor(255, 80, 80)
			: t.status == Plan::Status::Skipped          ? ofColor(140)
														 : ofColor(230);
		if (i == highlight) c = ofColor(255, 220, 0);
		ofSetColor(c);
		std::string line = (i == highlight ? "> " : "  ") + ofToString(t.mode.getName(), 11, ' ') + " " + ofToString(Plan::toString(t.kind), 7, ' ')
			+ ofToString(statusLabel(t), 11, ' ');
		if (t.rms > 0) line += " " + ofToString(t.rms, 2) + " px";
		ofDrawBitmapString(line, x, y);
		y += 16;
	}
	ofSetColor(255);
}

void ofApp::drawCameras() {
	ofSetColor(255);
	float y = 30;
	ofDrawBitmapString("CAMERA CALIBRATION - class compliant cameras", 20, y);
	y += 30;
	ofDrawBitmapString("board: " + board.getName() + "  (" + ofToString(board.getSize().x * 100, 1) + " x " + ofToString(board.getSize().y * 100, 1)
			+ " cm, from bin/data/board.yml)",
		20, y);
	y += 20;
	if (!boardError.empty()) {
		ofSetColor(255, 80, 80);
		ofDrawBitmapString(boardError, 20, y);
		ofSetColor(255);
		y += 20;
	}
	y += 20;
	if (cameras.empty()) {
		ofDrawBitmapString("no cameras found (F5 to refresh)", 20, y);
		return;
	}
	for (int i = 0; i < int(cameras.size()); i++) {
		ofSetColor(i == selected ? ofColor(255, 220, 0) : ofColor(220));
		ofDrawBitmapString((i == selected ? "> " : "  ") + ofToString(cameras[i].grabberIndex) + ": " + cameras[i].name + "   (" + ofToString(cameras[i].modes.size()) + " modes)", 20, y);
		y += 18;
	}
	ofSetColor(255);
	y += 20;
	ofDrawBitmapString("folder: bin/data/" + folder, 20, y);
	drawPlan(20, y + 30, -1);

	ofDrawBitmapStringHighlight("up/down: camera   ENTER: start (resumes, skips modes already done)   F: start fresh (redo all)   F5: refresh list\n"
								"Then hold the board in front of the camera. Views are taken automatically when it is still.",
		20, ofGetHeight() - 30);
	if (!message.empty()) ofDrawBitmapStringHighlight(message, 20, ofGetHeight() - 60, ofColor(80, 0, 0), ofColor(255));
}

void ofApp::drawRunning() {
	if (current < 0) return;
	const auto & task = plan.tasks[current];
	const float panel = 430;
	ofRectangle area(10, 10, ofGetWidth() - panel - 20, ofGetHeight() - 20);
	if (grabber.isInitialized() && grabber.getWidth() > 0) {
		ofRectangle r(0, 0, grabber.getWidth(), grabber.getHeight());
		r.scaleTo(area);
		ofSetColor(255);
		grabber.draw(r);
		calibrator.draw(r);
	}

	// panel
	const float x = ofGetWidth() - panel;
	float y = 30;
	ofSetColor(255);
	ofDrawBitmapString(cameras[selected].name, x, y);
	y += 20;
	ofDrawBitmapString("mode " + ofToString(current + 1) + " / " + ofToString(plan.tasks.size()) + ":  " + task.mode.getName() + "  "
			+ Plan::aspectName(task.mode.width, task.mode.height) + "  " + ofToUpper(Plan::toString(task.kind)),
		x, y);
	y += 30;

	const int n = calibrator.getNumSamples();
	const int target = task.kind == Plan::Kind::Full ? fullViews : verifyViews;
	const float coverageTarget = task.kind == Plan::Kind::Full ? fullCoverage : verifyCoverage;
	auto bar = [&](const std::string & label, float value, float targetValue, const std::string & text) {
		ofDrawBitmapString(label, x, y);
		ofNoFill();
		ofSetColor(120);
		ofDrawRectangle(x + 90, y - 11, 260, 14);
		ofFill();
		ofSetColor(value >= targetValue ? ofColor(80, 220, 80) : ofColor(255, 180, 0));
		ofDrawRectangle(x + 90, y - 11, 260 * ofClamp(value / std::max(targetValue, 1e-6f), 0, 1), 14);
		ofSetColor(255);
		ofDrawBitmapString(text, x + 360, y);
		y += 22;
	};
	bar("views", float(n), float(target), ofToString(n) + "/" + ofToString(target));
	bar("coverage", calibrator.getCoverage(), coverageTarget, ofToString(int(calibrator.getCoverage() * 100)) + "%");
	bar("tilted", calibrator.getTiltedFraction(), 0.35f, ofToString(int(calibrator.getTiltedFraction() * 100)) + "%");
	y += 10;

	std::string status = framesSinceOpen < warmupFrames ? "opening the camera..." : paused ? "PAUSED (space)" : calibrator.getStatus();
	ofDrawBitmapStringHighlight(status, x, y, ofColor(0, 60, 120), ofColor(255));
	y += 30;
	if (!message.empty()) {
		ofDrawBitmapStringHighlight(message, x, y, ofColor(60), ofColor(255));
		y += 30;
	}
	if (!task.note.empty()) {
		ofSetColor(255, 220, 0);
		ofDrawBitmapString(task.note, x, y);
		ofSetColor(255);
		y += 20;
	}
	y += 10;
	drawPlan(x, y, current);

	ofDrawBitmapStringHighlight("space: pause   c: capture now   backspace: remove last view\n"
								"s: solve now   r: restart this mode   n: skip mode   esc: stop",
		x, ofGetHeight() - 30);
}

void ofApp::drawSummary() {
	ofSetColor(255);
	ofDrawBitmapString("DONE: " + cameras[selected].name, 20, 30);
	ofDrawBitmapString("saved in bin/data/" + folder + "  (" + cameras[selected].label + "_<width>x<height>.yml per mode + index.yml)", 20, 50);
	drawPlan(20, 90, -1);
	float y = 110 + 16 * (plan.tasks.size() + 6);
	for (auto & t : plan.tasks) {
		if (t.note.empty()) continue;
		ofDrawBitmapString(t.mode.getName() + ": " + t.note, 20, y);
		y += 16;
	}
	ofDrawBitmapStringHighlight("ENTER / esc: back to the camera list", 20, ofGetHeight() - 20);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (screen == Screen::Cameras) {
		if (key == OF_KEY_UP && !cameras.empty()) {
			selected = (selected + int(cameras.size()) - 1) % int(cameras.size());
			buildPlan(cameras[selected], true);
		}
		if (key == OF_KEY_DOWN && !cameras.empty()) {
			selected = (selected + 1) % int(cameras.size());
			buildPlan(cameras[selected], true);
		}
		if (key == OF_KEY_RETURN) startCamera(true);
		if (key == 'f' || key == 'F') startCamera(false);
		if (key == OF_KEY_F5) refreshCameras();
		return;
	}
	if (screen == Screen::Summary) {
		if (key == OF_KEY_RETURN || key == OF_KEY_ESC) {
			screen = Screen::Cameras;
			message.clear();
			buildPlan(cameras[selected], true);
		}
		return;
	}
	// running
	if (current < 0) return;
	auto & task = plan.tasks[current];
	switch (key) {
	case ' ':
		paused = !paused;
		break;
	case 'c':
		calibrator.captureNow();
		break;
	case OF_KEY_BACKSPACE:
	case OF_KEY_DEL:
		calibrator.removeLastSample();
		break;
	case 's':
		finishTask();
		break;
	case 'r':
		calibrator.start(cv::Size(task.mode.width, task.mode.height));
		say(task.mode.getName() + ": restarted");
		break;
	case 'n':
		task.status = Plan::Status::Skipped;
		say(task.mode.getName() + ": skipped");
		saveIndex();
		nextTask();
		break;
	case OF_KEY_ESC:
		grabber.close();
		saveIndex();
		screen = Screen::Cameras;
		current = -1;
		say("stopped. ENTER resumes where you left off.");
		buildPlan(cameras[selected], true);
		break;
	}
}

void ofApp::exit() {
	grabber.close();
	if (screen == Screen::Running) saveIndex();
}
