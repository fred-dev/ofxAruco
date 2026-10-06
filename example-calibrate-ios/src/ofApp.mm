#include "ofApp.h"

using Plan = ofxArucoCalibrationPlan;

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetOrientation(OF_ORIENTATION_DEFAULT); // portrait
	ofSetFrameRate(60);
	ofBackground(25);
	ofxiOSDisableIdleTimer();
	textScale = std::max(1.5f, ofGetWidth() / 400.f);

	model = IOSCamera::getModelIdentifier();
	if (!board.load("board.yml")) {
		// bin/data is read only on iOS: the default board is only used in memory
		board = ofxArucoBoard::makeCharuco(7, 5, 0.036f, 0.027f, cv::aruco::DICT_5X5_100);
		board.setName("charuco_7x5_36mm_27mm_DICT_5X5_100_id0");
		message = "no bin/data/board.yml: using the default A4 ChArUco 7x5 36 mm";
	}
	if (!calibrator.setup(board)) {
		boardError = "board.yml must be a ChArUco board (ofxAruco/example-print-boards)";
	}
	lenses = IOSCamera::listDevices();
	if (!lenses.empty()) buildPlan(true);
}

std::string ofApp::lensLabel() const {
	if (lenses.empty()) return model;
	const auto & lens = lenses[selected];
	return model + "_" + lens.lens + (distortionCorrection && lens.distortionCorrectionSupported ? "_DC" : "");
}

void ofApp::buildPlan(bool resume) {
	if (lenses.empty()) return;
	folder = ofxiOSGetDocumentsDirectory() + "calibrations/" + lensLabel();
	std::vector<ofxArucoCameraMode> modes;
	for (auto & f : lenses[selected].formats) {
		ofxArucoCameraMode m;
		m.width = f.width;
		m.height = f.height;
		m.fps = f.maxFps;
		m.fov = f.fov;
		modes.push_back(m);
	}
	plan.build(modes, lensLabel() + "_");
	if (resume) plan.resumeFrom(folder);
}

void ofApp::say(const std::string & text) {
	message = text;
	ofLogNotice() << text;
}

//--------------------------------------------------------------
void ofApp::startLens(bool resume) {
	if (lenses.empty() || !boardError.empty()) return;
	camera.setDistortionCorrection(distortionCorrection);
	buildPlan(resume);
	if (plan.tasks.empty()) {
		say("this lens reports no formats");
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
	camera.close();
	saveIndex();
	screen = Screen::Summary;
	current = -1;
}

void ofApp::startTask(int index) {
	current = index;
	auto & task = plan.tasks[index];
	if (task.kind == Plan::Kind::Verify && !plan.getReference(task)) {
		task.kind = Plan::Kind::Full;
		task.note = "no reference for this aspect ratio: full calibration";
	}
	if (!camera.open(lenses[selected].uniqueId, task.mode.width, task.mode.height)) {
		task.status = Plan::Status::Failed;
		task.note = "could not open this format";
		say(task.mode.getName() + ": could not open this format");
		nextTask();
		return;
	}
	framesSinceOpen = 0;
	paused = false;
	calibrator.start(cv::Size(task.mode.width, task.mode.height));
	say(task.mode.getName() + ": " + (task.kind == Plan::Kind::Full ? "full calibration" : "verification"));
}

void ofApp::stopLens() {
	camera.close();
	saveIndex();
	screen = Screen::Lenses;
	current = -1;
	buildPlan(true);
}

void ofApp::finishTask() {
	if (current < 0) return;
	auto & task = plan.tasks[current];
	const auto & lens = lenses[selected];
	std::map<std::string, std::string> info = {
		{ "camera_name", model + " " + lens.name },
		{ "camera_model", model },
		{ "lens", lens.lens },
		{ "focus_lens_position", ofToString(camera.getLensPosition(), 4) },
		{ "distortion_correction", lens.distortionCorrectionSupported ? (distortionCorrection ? "on" : "off") : "not supported" },
		{ "image_orientation", "sensor native (landscape)" },
		{ "board", board.getName() },
	};
	glm::vec4 reported(0);
	if (camera.hasReportedIntrinsics()) {
		reported = camera.getReportedIntrinsics();
		info["ios_reported_intrinsics"] = "fx " + ofToString(reported.x, 2) + " fy " + ofToString(reported.y, 2) + " cx " + ofToString(reported.z, 2)
			+ " cy " + ofToString(reported.w, 2);
	}
	auto compareWithIOS = [&](const ofxArucoIntrinsics & k) -> std::string {
		if (reported.x <= 0) return "";
		return ", iOS fx differs " + ofToString(std::abs(k.getFx() - reported.x) / reported.x * 100, 1) + "%";
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
		const double worst = std::max({ std::abs(r.getFx() - scaled.getFx()) / scaled.getFx(), std::abs(r.getFy() - scaled.getFy()) / scaled.getFy(),
			std::abs(r.getCx() - scaled.getCx()) / task.mode.width, std::abs(r.getCy() - scaled.getCy()) / task.mode.height });
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
			task.note = "verified against " + ref->mode.getName() + " (" + ofToString(worst * 100, 2) + "%)" + compareWithIOS(scaled);
			say(task.mode.getName() + ": " + task.note);
			saveIndex();
			nextTask();
		} else {
			task.kind = Plan::Kind::Full;
			task.note = "differs from " + ref->mode.getName() + " by " + ofToString(worst * 100, 1) + "%: full calibration";
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
	task.note = result.message + compareWithIOS(result.intrinsics) + (result.rms > maxRms ? "  HIGH ERROR: redo it (board flat? focus locked?)" : "");
	say(task.mode.getName() + ": " + task.note);
	saveIndex();
	nextTask();
}

void ofApp::saveIndex() {
	if (lenses.empty() || plan.tasks.empty()) return;
	plan.saveIndex(folder, { { "camera_name", model + " " + lenses[selected].name }, { "camera_model", model }, { "lens", lenses[selected].lens },
							   { "board", board.getName() } });
}

//--------------------------------------------------------------
void ofApp::update() {
	if (screen != Screen::Running || current < 0) return;
	if (!camera.update()) return;
	auto & task = plan.tasks[current];
	framesSinceOpen++;
	if (framesSinceOpen == 1 && (camera.getWidth() != task.mode.width || camera.getHeight() != task.mode.height)) {
		task.status = Plan::Status::Failed;
		task.note = "the camera delivered " + ofToString(camera.getWidth()) + "x" + ofToString(camera.getHeight());
		say(task.mode.getName() + ": " + task.note);
		nextTask();
		return;
	}
	preview.loadData(camera.getGray());
	if (framesSinceOpen < warmupFrames || paused || !camera.isFocusLocked()) return;
	calibrator.update(camera.getGray());

	const int n = calibrator.getNumSamples();
	const float coverage = calibrator.getCoverage();
	if (task.kind == Plan::Kind::Full) {
		if ((n >= fullViews && coverage >= fullCoverage) || n >= fullViews * 2) finishTask();
	} else {
		if (n >= verifyViews && coverage >= verifyCoverage) finishTask();
	}
}

//--------------------------------------------------------------
// The preview is rotated 90 degrees clockwise to stand upright in portrait, so
// directions in the calibrator's hints (image directions) are turned too.
std::string ofApp::orientHint(const std::string & hint) const {
	std::string s = hint;
	ofStringReplace(s, "top", "#1");
	ofStringReplace(s, "bottom", "#2");
	ofStringReplace(s, "left", "#3");
	ofStringReplace(s, "right", "#4");
	ofStringReplace(s, "#1", "right");
	ofStringReplace(s, "#2", "left");
	ofStringReplace(s, "#3", "top");
	ofStringReplace(s, "#4", "bottom");
	return s;
}

void ofApp::drawText(const std::string & text, float x, float y, const ofColor & color) {
	ofPushMatrix();
	ofTranslate(x, y);
	ofScale(textScale, textScale);
	ofSetColor(color);
	ofDrawBitmapString(text, 0, 0);
	ofPopMatrix();
}

void ofApp::addButton(const std::string & label, const ofRectangle & rect, std::function<void()> action, bool highlighted) {
	Button b;
	b.label = label;
	b.rect = rect;
	b.action = action;
	b.highlighted = highlighted;
	buttons.push_back(b);
}

void ofApp::drawButtons() {
	for (auto & b : buttons) {
		ofSetColor(b.highlighted ? ofColor(30, 110, 200) : ofColor(70));
		ofDrawRectRounded(b.rect, 8 * textScale / 2);
		const float w = b.label.size() * 8 * textScale;
		drawText(b.label, b.rect.getCenter().x - w / 2, b.rect.getCenter().y + 4 * textScale);
	}
	ofSetColor(255);
}

float ofApp::drawPlan(float x, float y, int highlight) {
	int lastGroup = -1;
	for (int i = 0; i < int(plan.tasks.size()); i++) {
		const auto & t = plan.tasks[i];
		if (t.group != lastGroup) {
			lastGroup = t.group;
			drawText(Plan::aspectName(t.mode.width, t.mode.height) + (t.mode.fov > 0 ? "  fov " + ofToString(t.mode.fov, 1) : ""), x, y, ofColor(150));
			y += lineHeight();
		}
		ofColor c = t.status == Plan::Status::Calibrated ? ofColor(80, 220, 80)
			: t.status == Plan::Status::Scaled           ? ofColor(80, 200, 255)
			: t.status == Plan::Status::Failed           ? ofColor(255, 80, 80)
			: t.status == Plan::Status::Skipped          ? ofColor(140)
														 : ofColor(230);
		if (i == highlight) c = ofColor(255, 220, 0);
		std::string line = (i == highlight ? "> " : "  ") + ofToString(t.mode.getName(), 10, ' ') + " " + ofToString(Plan::toString(t.kind), 7, ' ')
			+ Plan::toString(t.status);
		if (t.rms > 0) line += " " + ofToString(t.rms, 2) + "px";
		drawText(line, x, y, c);
		y += lineHeight();
	}
	return y;
}

void ofApp::draw() {
	buttons.clear();
	switch (screen) {
	case Screen::Lenses: drawLenses(); break;
	case Screen::Running: drawRunning(); break;
	case Screen::Summary: drawSummary(); break;
	}
	drawButtons();
}

void ofApp::drawLenses() {
	const float W = ofGetWidth(), H = ofGetHeight();
	const float pad = 10 * textScale, buttonH = 30 * textScale;
	float y = pad * 3;
	drawText("CAMERA CALIBRATION", pad, y);
	y += lineHeight() * 1.5f;
	drawText(model + "   board: " + board.getName(), pad, y, ofColor(180));
	y += lineHeight();
	if (!boardError.empty()) {
		drawText(boardError, pad, y, ofColor(255, 80, 80));
		y += lineHeight();
	}
	y += lineHeight() * 0.5f;
	if (lenses.empty()) {
		drawText("no back camera found", pad, y);
		return;
	}
	for (int i = 0; i < int(lenses.size()); i++) {
		const auto & lens = lenses[i];
		addButton(lens.name + " (" + ofToString(lens.formats.size()) + ")", ofRectangle(pad, y, W - 2 * pad, buttonH), [this, i]() {
			selected = i;
			buildPlan(true);
		}, i == selected);
		y += buttonH + pad / 2;
	}
	if (lenses[selected].distortionCorrectionSupported) {
		addButton(std::string("distortion correction: ") + (distortionCorrection ? "ON" : "OFF"), ofRectangle(pad, y, W - 2 * pad, buttonH), [this]() {
			distortionCorrection = !distortionCorrection;
			buildPlan(true);
		}, distortionCorrection);
		y += buttonH + pad / 2;
		drawText("use the setting your app will use", pad, y + lineHeight(), ofColor(150));
		y += lineHeight() * 1.5f;
	}
	y += lineHeight();
	drawText("files: Documents/calibrations/" + lensLabel(), pad, y, ofColor(150));
	y += lineHeight() * 1.5f;
	drawPlan(pad, y, -1);
	if (!message.empty()) drawText(message, pad, H - buttonH * 2 - pad * 3, ofColor(255, 200, 0));

	const float bw = (W - 3 * pad) / 2;
	addButton("Start", ofRectangle(pad, H - buttonH - pad * 2, bw, buttonH), [this]() { startLens(true); }, true);
	addButton("Start fresh", ofRectangle(pad * 2 + bw, H - buttonH - pad * 2, bw, buttonH), [this]() { startLens(false); });
}

void ofApp::drawRunning() {
	if (current < 0) return;
	const auto & task = plan.tasks[current];
	const float W = ofGetWidth(), H = ofGetHeight();
	const float pad = 10 * textScale, buttonH = 30 * textScale;

	// preview, rotated upright (the frames stay in sensor orientation)
	ofRectangle area(pad, pad * 3, W - 2 * pad, H * 0.5f);
	if (preview.isAllocated()) {
		const float bw = preview.getWidth(), bh = preview.getHeight();
		const float s = std::min(area.width / bh, area.height / bw);
		ofPushMatrix();
		ofTranslate(area.getCenter());
		ofRotateDeg(90);
		const ofRectangle r(-bw * s / 2, -bh * s / 2, bw * s, bh * s);
		ofSetColor(255);
		preview.draw(r);
		calibrator.draw(r);
		ofPopMatrix();
	}

	float y = area.getBottom() + lineHeight() * 1.5f;
	const int n = calibrator.getNumSamples();
	const int target = task.kind == Plan::Kind::Full ? fullViews : verifyViews;
	drawText(lenses[selected].name + "  " + task.mode.getName() + "  " + ofToUpper(Plan::toString(task.kind)) + "  (" + ofToString(current + 1) + "/"
			+ ofToString(plan.tasks.size()) + ")",
		pad, y);
	y += lineHeight();
	drawText("views " + ofToString(n) + "/" + ofToString(target) + "   coverage " + ofToString(int(calibrator.getCoverage() * 100)) + "%   tilted "
			+ ofToString(int(calibrator.getTiltedFraction() * 100)) + "%",
		pad, y);
	y += lineHeight() * 1.5f;
	std::string status;
	ofColor statusColor(255, 220, 0);
	if (framesSinceOpen < warmupFrames) {
		status = "opening the camera...";
	} else if (!camera.isFocusLocked()) {
		status = "point at the board, let it focus, then Lock focus";
		statusColor = ofColor(255, 120, 0);
	} else if (paused) {
		status = "PAUSED";
	} else {
		status = orientHint(calibrator.getStatus());
	}
	// wrap
	const size_t perLine = size_t((W - 2 * pad) / (8 * textScale));
	for (size_t i = 0; i < status.size(); i += perLine) {
		drawText(status.substr(i, perLine), pad, y, statusColor);
		y += lineHeight();
	}
	if (!message.empty()) {
		for (size_t i = 0; i < message.size(); i += perLine) {
			drawText(message.substr(i, perLine), pad, y, ofColor(180));
			y += lineHeight();
		}
	}
	if (camera.isFocusLocked()) {
		drawText("focus locked at " + ofToString(camera.getLensPosition(), 3), pad, y, ofColor(150));
	}

	// buttons: 2 rows of 4
	const float bw = (W - 5 * pad) / 4;
	const float row2 = H - buttonH - pad * 2, row1 = row2 - buttonH - pad;
	auto at = [&](int col, float row) { return ofRectangle(pad + col * (bw + pad), row, bw, buttonH); };
	if (camera.isFocusLocked()) {
		addButton("Unlock", at(0, row1), [this]() { camera.unlockFocus(); });
	} else {
		addButton("Lock focus", at(0, row1), [this]() { camera.lockFocus(); }, true);
	}
	addButton(paused ? "Resume" : "Pause", at(1, row1), [this]() { paused = !paused; }, paused);
	addButton("Capture", at(2, row1), [this]() { calibrator.captureNow(); });
	addButton("Undo", at(3, row1), [this]() { calibrator.removeLastSample(); });
	addButton("Solve", at(0, row2), [this]() { finishTask(); });
	addButton("Restart", at(1, row2), [this]() {
		calibrator.start(cv::Size(plan.tasks[current].mode.width, plan.tasks[current].mode.height));
		say("views cleared");
	});
	addButton("Skip", at(2, row2), [this]() {
		plan.tasks[current].status = Plan::Status::Skipped;
		saveIndex();
		nextTask();
	});
	addButton("Stop", at(3, row2), [this]() { stopLens(); });
}

void ofApp::drawSummary() {
	const float W = ofGetWidth(), H = ofGetHeight();
	const float pad = 10 * textScale, buttonH = 30 * textScale;
	float y = pad * 3;
	drawText("DONE: " + lenses[selected].name, pad, y);
	y += lineHeight();
	drawText("Files app: On My iPhone > this app", pad, y, ofColor(150));
	y += lineHeight();
	drawText("calibrations/" + lensLabel(), pad, y, ofColor(150));
	y += lineHeight() * 1.5f;
	y = drawPlan(pad, y, -1) + lineHeight();
	const size_t perLine = size_t((W - 2 * pad) / (8 * textScale));
	for (auto & t : plan.tasks) {
		if (t.note.empty()) continue;
		const std::string line = t.mode.getName() + ": " + t.note;
		for (size_t i = 0; i < line.size(); i += perLine) {
			drawText(line.substr(i, perLine), pad, y, ofColor(200));
			y += lineHeight();
		}
	}
	addButton("Back", ofRectangle(pad, H - buttonH - pad * 2, W - 2 * pad, buttonH), [this]() {
		screen = Screen::Lenses;
		message.clear();
		buildPlan(true);
	}, true);
}

//--------------------------------------------------------------
void ofApp::touchDown(ofTouchEventArgs & touch) {
	for (auto & b : buttons) {
		if (b.rect.inside(touch.x, touch.y)) {
			auto action = b.action; // the action may rebuild the buttons
			action();
			return;
		}
	}
}

void ofApp::lostFocus() {
	if (screen == Screen::Running) paused = true;
}

void ofApp::gotFocus() {
}

void ofApp::exit() {
	camera.close();
	if (screen == Screen::Running) saveIndex();
}
