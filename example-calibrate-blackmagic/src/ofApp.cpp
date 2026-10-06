#include "ofApp.h"

using Plan = ofxArucoCalibrationPlan;

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowTitle("ofxAruco - calibrate camera (Blackmagic)");
	ofSetVerticalSync(true);
	ofSetEscapeQuitsApp(false);
	ofBackground(25);

	if (!board.load("board.yml")) {
		board = ofxArucoBoard::makeCharuco(7, 5, 0.036f, 0.027f, cv::aruco::DICT_5X5_100);
		board.setName("charuco_7x5_36mm_27mm_DICT_5X5_100_id0");
		board.save("board.yml");
	}
	if (!calibrator.setup(board)) {
		boardError = "bin/data/board.yml must be a ChArUco board (make one with ofxAruco/example-print-boards)";
	}
	devices = cam.listDevices();

	fields = {
		{ "camera_body", "camera body", "e.g. FX6, URSA Mini Pro 12K, BMPCC6K", "", true, true, "" },
		{ "sensor_setting", "sensor setting", "sensor mode / crop, e.g. FF 4K, S35, 6K 2.4:1, HD crop", "", true, true, "" },
		{ "lens", "lens", "e.g. Sigma 18-35 f1.8, Zeiss CP3 25", "", true, true, "" },
		{ "focal_length_mm", "focal length (mm)", "the zoom setting, e.g. 24", "", true, true, "mm" },
		{ "focus", "focus distance", "e.g. 2m, inf (lock it)", "", true, false, "" },
		{ "aperture", "aperture", "e.g. f4 (recorded only)", "", false, false, "" },
		{ "notes", "notes", "anything else (recorded only)", "", false, false, "" },
	};
	loadFields();
}

void ofApp::loadFields() {
	if (!ofFile::doesFileExist("lens_settings.json")) return;
	const ofJson j = ofLoadJson("lens_settings.json");
	for (auto & f : fields) {
		if (j.contains(f.key) && j[f.key].is_string()) f.value = j[f.key].get<std::string>();
	}
}

void ofApp::saveFields() {
	ofJson j;
	for (auto & f : fields) j[f.key] = f.value;
	ofSavePrettyJson("lens_settings.json", j);
}

std::string ofApp::labelText() const {
	std::string text;
	for (auto & f : fields) {
		const std::string v = ofTrim(f.value);
		if (!f.inName || v.empty()) continue;
		if (!text.empty()) text += "_";
		if (f.key == "focus") text += "focus";
		text += v + f.suffix;
	}
	return text;
}

std::string ofApp::safeLabel() const {
	std::string safe;
	for (char c : labelText()) {
		if (std::isalnum((unsigned char)c) || c == '-' || c == '.') {
			safe += c;
		} else if (c == '_' || c == ' ') {
			if (!safe.empty() && safe.back() != '_') safe += '_';
		}
	}
	return safe.empty() ? "camera" : safe;
}

std::string ofApp::missingField() const {
	for (auto & f : fields) {
		if (f.required && ofTrim(f.value).empty()) return f.title;
	}
	return "";
}

void ofApp::say(const std::string & text) {
	message = text;
	ofLogNotice() << text;
}

bool ofApp::taskIsOpen() const {
	return current >= 0 && plan.tasks[current].status == Plan::Status::Pending;
}

//--------------------------------------------------------------
void ofApp::startCapture() {
	if (devices.empty() || !boardError.empty()) return;
	const std::string missing = missingField();
	if (!missing.empty()) {
		say("fill in: " + missing);
		return;
	}
	saveFields();
	folder = "calibrations/" + safeLabel();
	seenModes.clear();
	plan.tasks.clear();
	current = -1;
	currentWidth = currentHeight = 0;
	cam.setDeviceID(devices[selected].id);
	// starts at 1920x1080; if the camera sends something else, press m (matchSignal)
	cam.setPixelFormat(bmdFormat8BitYUV);
	if (!cam.setup(1920, 1080)) {
		say("could not start capture on " + devices[selected].deviceName);
		return;
	}
	screen = Screen::Running;
	say("waiting for the signal...");
}

void ofApp::matchSignal() {
	const DeckLinkSignalInfo signal = cam.getSignal();
	if (!signal.detected) {
		say("this device doesn't report the signal format: set the camera to 1920x1080");
		return;
	}
	// the calibration only uses the grey image: any pixel format works, the matching one avoids a conversion
	cam.setPixelFormat(signal.rgb ? (signal.bitDepth >= 10 ? bmdFormat10BitRGB : bmdFormat8BitARGB)
								  : (signal.bitDepth >= 10 ? bmdFormat10BitYUV : bmdFormat8BitYUV));
	if (!cam.setup(signal.displayMode, devices[selected].id)) {
		say("could not capture " + signal.describe());
		return;
	}
	say("capture set to " + signal.describe());
}

void ofApp::stopCapture() {
	cam.close();
	saveIndex();
	screen = Screen::Setup;
	current = -1;
}

// A resolution arrived (first frame, or the camera's output format changed).
void ofApp::onMode(int width, int height) {
	currentWidth = width;
	currentHeight = height;
	framesSinceMode = 0;
	const bool known = std::any_of(seenModes.begin(), seenModes.end(), [&](const ofxArucoCameraMode & m) { return m.width == width && m.height == height; });
	if (!known) seenModes.push_back({ width, height, cam.getFrameRate() });
	// rebuild the plan from the resolutions seen so far, statuses from the files already saved
	plan.build(seenModes, safeLabel() + "_");
	plan.resumeFrom(folder);
	current = -1;
	for (int i = 0; i < int(plan.tasks.size()); i++) {
		if (plan.tasks[i].mode.width == width && plan.tasks[i].mode.height == height) current = i;
	}
	if (current < 0) return;
	auto & task = plan.tasks[current];
	if (task.status != Plan::Status::Pending) {
		say(task.mode.getName() + " (" + cam.getDisplayModeName() + ") is already " + Plan::toString(task.status)
			+ ". R: redo it. Or switch the camera's output to another resolution and press m.");
		return;
	}
	startTask();
}

void ofApp::startTask() {
	auto & task = plan.tasks[current];
	task.status = Plan::Status::Pending;
	// a resolution with the same aspect ratio is already calibrated: verify the scaled result
	task.kind = plan.getReference(task) ? Plan::Kind::Verify : Plan::Kind::Full;
	task.note.clear();
	paused = false;
	calibrator.start(cv::Size(task.mode.width, task.mode.height));
	say(task.mode.getName() + " (" + cam.getDisplayModeName() + "): " + (task.kind == Plan::Kind::Full ? "full calibration" : "verification"));
}

void ofApp::finishTask() {
	if (!taskIsOpen()) return;
	auto & task = plan.tasks[current];
	std::map<std::string, std::string> info = {
		{ "camera_name", labelText() },
		{ "capture_device", devices[selected].deviceName },
		{ "display_mode", cam.getDisplayModeName() },
		{ "board", board.getName() },
	};
	for (auto & f : fields) info[f.key] = f.value;
	const std::string path = ofFilePath::join(folder, task.getFileName());
	const std::string next = "  Switch the camera's output format and press m for another resolution, or esc.";

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
			task.note = "verified against " + ref->mode.getName() + " (max diff " + ofToString(worst * 100, 2) + "%)";
			say(task.mode.getName() + ": " + task.note + "." + next);
			saveIndex();
		} else {
			task.kind = Plan::Kind::Full;
			task.note = "differs from " + ref->mode.getName() + " by " + ofToString(worst * 100, 1) + "% (cropped format?): full calibration";
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
	task.note = result.message + (result.rms > maxRms ? "  HIGH ERROR: check the board is flat and focus/zoom are locked, then R to redo" : "");
	say(task.mode.getName() + ": " + task.note + "." + next);
	saveIndex();
}

void ofApp::saveIndex() {
	if (plan.tasks.empty() || devices.empty()) return;
	std::map<std::string, std::string> info = { { "camera_name", labelText() }, { "capture_device", devices[selected].deviceName }, { "board", board.getName() } };
	for (auto & f : fields) info[f.key] = f.value;
	plan.saveIndex(folder, info);
}

//--------------------------------------------------------------
void ofApp::update() {
	if (screen != Screen::Running) return;
	if (!cam.update()) return;
	const int w = int(cam.getWidth()), h = int(cam.getHeight());
	if (w <= 0 || h <= 0) return;
	if (w != currentWidth || h != currentHeight) onMode(w, h);
	framesSinceMode++;
	if (!taskIsOpen() || framesSinceMode < warmupFrames || paused) return;

	calibrator.update(cam.getGrayPixels()); // grey: no colour conversion needed

	const auto & task = plan.tasks[current];
	const int n = calibrator.getNumSamples();
	const float coverage = calibrator.getCoverage();
	if (task.kind == Plan::Kind::Full) {
		if ((n >= fullViews && coverage >= fullCoverage) || n >= fullViews * 2) finishTask();
	} else {
		if (n >= verifyViews && coverage >= verifyCoverage) finishTask();
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (screen == Screen::Setup) {
		drawSetup();
	} else {
		drawRunning();
	}
}

void ofApp::drawPlan(float x, float y) {
	ofDrawBitmapString("resolutions this session (" + folder + ")", x, y);
	y += 20;
	for (int i = 0; i < int(plan.tasks.size()); i++) {
		const auto & t = plan.tasks[i];
		ofColor c = t.status == Plan::Status::Calibrated ? ofColor(80, 220, 80)
			: t.status == Plan::Status::Scaled           ? ofColor(80, 200, 255)
			: t.status == Plan::Status::Failed           ? ofColor(255, 80, 80)
														 : ofColor(230);
		if (i == current) c = ofColor(255, 220, 0);
		ofSetColor(c);
		std::string line = (i == current ? "> " : "  ") + ofToString(t.mode.getName(), 11, ' ') + " "
			+ ofToString(ofxArucoCalibrationPlan::aspectName(t.mode.width, t.mode.height), 6, ' ') + ofToString(Plan::toString(t.status), 11, ' ');
		if (t.rms > 0) line += " " + ofToString(t.rms, 2) + " px";
		ofDrawBitmapString(line, x, y);
		y += 16;
	}
	ofSetColor(255);
}

void ofApp::drawSetup() {
	ofSetColor(255);
	float y = 30;
	ofDrawBitmapString("CAMERA CALIBRATION - Blackmagic input", 20, y);
	y += 30;
	ofDrawBitmapString("board: " + board.getName() + "  (bin/data/board.yml)", 20, y);
	y += 20;
	if (!boardError.empty()) {
		ofSetColor(255, 80, 80);
		ofDrawBitmapString(boardError, 20, y);
		ofSetColor(255);
		y += 20;
	}
	y += 20;
	if (devices.empty()) {
		ofDrawBitmapString("no Blackmagic devices found (is Desktop Video installed and the device connected?)", 20, y);
		return;
	}
	// 0: input device
	ofSetColor(field == 0 ? ofColor(255, 220, 0) : ofColor(220));
	ofDrawBitmapString(std::string(field == 0 ? "> " : "  ") + "input (left/right):  " + ofToString(devices[selected].id) + ": " + devices[selected].deviceName
			+ "   (" + ofToString(selected + 1) + "/" + ofToString(devices.size()) + ")",
		20, y);
	y += 26;
	// the camera fields
	const bool blink = int(ofGetElapsedTimef() * 2) % 2 == 0;
	for (int i = 0; i < int(fields.size()); i++) {
		const auto & f = fields[i];
		const bool active = field == i + 1;
		ofSetColor(active ? ofColor(255, 220, 0) : ofColor(220));
		ofDrawBitmapString(std::string(active ? "> " : "  ") + ofToString(f.title + (f.required ? " *" : ""), 22, ' '), 20, y);
		ofDrawBitmapStringHighlight(f.value + f.suffix + (active && blink ? "_" : " "), 220, y, active ? ofColor(0, 60, 120) : ofColor(50), ofColor(255));
		ofSetColor(140);
		ofDrawBitmapString(f.hint, 520, y);
		y += 22;
	}
	ofSetColor(255);
	y += 20;
	ofDrawBitmapString("name:  " + safeLabel(), 20, y);
	y += 16;
	ofSetColor(180);
	ofDrawBitmapString("files: bin/data/calibrations/" + safeLabel() + "/" + safeLabel() + "_<width>x<height>.yml  (resolution from the capture signal)", 20, y);
	y += 16;
	ofDrawBitmapString("The intrinsics are only valid for these settings: lock focus and zoom, recalibrate when any of them changes.", 20, y);
	ofSetColor(255);
	if (!message.empty()) ofDrawBitmapStringHighlight(message, 20, ofGetHeight() - 60, ofColor(80, 0, 0), ofColor(255));
	ofDrawBitmapStringHighlight("up/down/tab: field   left/right: input   type: value   ENTER: start", 20, ofGetHeight() - 20);
}

void ofApp::drawRunning() {
	const float panel = 430;
	ofRectangle area(10, 10, ofGetWidth() - panel - 20, ofGetHeight() - 20);
	if (currentWidth > 0) {
		ofRectangle r(0, 0, currentWidth, currentHeight);
		r.scaleTo(area);
		ofSetColor(255);
		cam.drawGray(r.x, r.y, r.width, r.height);
		if (taskIsOpen()) calibrator.draw(r);
	}

	const float x = ofGetWidth() - panel;
	float y = 30;
	ofSetColor(255);
	ofDrawBitmapString(labelText(), x, y);
	y += 16;
	ofDrawBitmapString("via " + devices[selected].deviceName, x, y);
	y += 20;
	ofDrawBitmapString("capture: " + cam.getDisplayModeName() + "  " + ofToString(currentWidth) + "x" + ofToString(currentHeight), x, y);
	y += 16;
	ofDrawBitmapString("signal:  " + cam.getSignal().describe(), x, y);
	y += 20;
	const std::string warning = cam.getSignalWarning();
	if (!warning.empty()) {
		std::string m = "WARNING: " + warning + ". Press m to set the capture to the signal.";
		while (!m.empty()) {
			ofDrawBitmapStringHighlight(m.substr(0, 52), x, y, ofColor(150, 0, 0), ofColor(255));
			m = m.size() > 52 ? m.substr(52) : "";
			y += 18;
		}
	}
	y += 12;

	if (taskIsOpen()) {
		const auto & task = plan.tasks[current];
		ofDrawBitmapString(ofToUpper(Plan::toString(task.kind)), x, y);
		y += 22;
		const int target = task.kind == Plan::Kind::Full ? fullViews : verifyViews;
		const float coverageTarget = task.kind == Plan::Kind::Full ? fullCoverage : verifyCoverage;
		auto bar = [&](const std::string & name, float value, float targetValue, const std::string & text) {
			ofDrawBitmapString(name, x, y);
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
		const int n = calibrator.getNumSamples();
		bar("views", float(n), float(target), ofToString(n) + "/" + ofToString(target));
		bar("coverage", calibrator.getCoverage(), coverageTarget, ofToString(int(calibrator.getCoverage() * 100)) + "%");
		bar("tilted", calibrator.getTiltedFraction(), 0.35f, ofToString(int(calibrator.getTiltedFraction() * 100)) + "%");
		y += 10;
		const std::string status = framesSinceMode < warmupFrames ? "signal changed, settling..." : paused ? "PAUSED (space)" : calibrator.getStatus();
		ofDrawBitmapStringHighlight(status, x, y, ofColor(0, 60, 120), ofColor(255));
		y += 30;
	}
	if (!message.empty()) {
		// wrap long messages
		std::string m = message, line;
		while (!m.empty()) {
			line = m.substr(0, 52);
			m = m.size() > 52 ? m.substr(52) : "";
			ofDrawBitmapStringHighlight(line, x, y, ofColor(60), ofColor(255));
			y += 18;
		}
		y += 12;
	}
	drawPlan(x, y);

	ofDrawBitmapStringHighlight("space: pause   c: capture now   backspace: remove last view\n"
								"s: solve now   r: restart views   R: redo this resolution\n"
								"m: set the capture to the signal   esc: stop",
		x, ofGetHeight() - 44);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (screen == Screen::Setup) {
		const int numFields = int(fields.size()) + 1;
		if (key == OF_KEY_UP) {
			field = (field + numFields - 1) % numFields;
		} else if (key == OF_KEY_DOWN || key == OF_KEY_TAB) {
			field = (field + 1) % numFields;
		} else if (key == OF_KEY_LEFT && !devices.empty()) {
			if (field == 0) selected = (selected + int(devices.size()) - 1) % int(devices.size());
		} else if (key == OF_KEY_RIGHT && !devices.empty()) {
			if (field == 0) selected = (selected + 1) % int(devices.size());
		} else if (key == OF_KEY_RETURN) {
			startCapture();
		} else if (field > 0) {
			auto & value = fields[field - 1].value;
			if (key == OF_KEY_BACKSPACE || key == OF_KEY_DEL) {
				if (!value.empty()) value.pop_back();
			} else if (key >= 32 && key < 127 && value.size() < 48) {
				value += char(key);
			}
		}
		return;
	}
	switch (key) {
	case ' ': paused = !paused; break;
	case 'c': if (taskIsOpen()) calibrator.captureNow(); break;
	case OF_KEY_BACKSPACE:
	case OF_KEY_DEL: if (taskIsOpen()) calibrator.removeLastSample(); break;
	case 's': finishTask(); break;
	case 'm': matchSignal(); break;
	case 'r':
		if (taskIsOpen()) {
			calibrator.start(cv::Size(currentWidth, currentHeight));
			say("views cleared");
		}
		break;
	case 'R':
		if (current >= 0) {
			plan.tasks[current].status = Plan::Status::Pending;
			startTask();
		}
		break;
	case OF_KEY_ESC: stopCapture(); say("stopped"); break;
	}
}

void ofApp::exit() {
	if (screen == Screen::Running) stopCapture();
}
