#include "ofApp.h"

using namespace ofxArucoUtils;

namespace {
	const std::vector<ofColor> cameraColors = { ofColor(255, 90, 90), ofColor(90, 220, 90), ofColor(90, 160, 255),
		ofColor(255, 200, 60), ofColor(220, 90, 255), ofColor(60, 230, 230) };
	const ofColor & colorFor(int i) { return cameraColors[i % cameraColors.size()]; }
}

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowTitle("ofxAruco - multi camera calibration");
	ofSetVerticalSync(true);
	ofBackground(25);

	loadOrCreateBoard();

	// GUI
	gui.setup("settings", "settings.json", ofGetWidth() - 230, 10);
	gui.add(calibration.parameters);
	gui.add(showSamples);
	gui.add(showTruth);
	gui.add(detectorSettings.parameters);
	gui.getGroup("aruco").minimize();
	if (ofFile::doesFileExist("settings.json")) gui.loadFromFile("settings.json");
	detectorSettings.dictionary = board.getDictionary(); // must match the board
	// copy any detector setting change to every detector
	listeners.push(detectorSettings.parameters.parameterChangedE().newListener([this](ofAbstractParameter &) {
		ofJson j;
		ofSerialize(j, detectorSettings.parameters);
		for (auto & d : detectors) ofDeserialize(j, d->parameters);
	}));

	// 3D view: look at the scene (~1.8 m in front of camera 0) from above, behind and to the side.
	// (the scene is drawn with y up: OpenCV's z forward becomes -z here)
	easyCam.setNearClip(0.01f);
	easyCam.setFarClip(100.f);
	easyCam.setTarget(glm::vec3(0, 0, -1.2f));
	easyCam.setPosition(glm::vec3(2.2f, 2.0f, 2.2f));
	easyCam.lookAt(glm::vec3(0, 0, -1.2f));

	setupCameras(true);
}

void ofApp::loadOrCreateBoard() {
	// The board must match what you printed. Default: ChArUco 7x5, 55mm squares, 41mm markers, DICT_5X5_100 (A3).
	if (!ofFile::doesFileExist("board.yml") || !board.load("board.yml")) {
		board = ofxArucoBoard::makeCharuco(7, 5, 0.055f, 0.041f, cv::aruco::DICT_5X5_100);
		board.setName("charuco_7x5_55mm_41mm_DICT_5X5_100_id0");
		board.save("board.yml");
		ofLogNotice() << "created bin/data/board.yml (print it with example-print-boards)";
	}
}

void ofApp::setupCameras(bool sim) {
	simulated = sim;
	cameras.clear();
	detectors.clear();
	samplePoses.clear();

	if (simulated) {
		// 3 virtual 1280x720 cameras around the scene
		simScene = std::make_shared<SimulatedScene>();
		simScene->setup(board);
		ofxArucoIntrinsics intr;
		intr.setup(920, 920, 639.5, 359.5, 1280, 720);
		const glm::vec3 target(0, 0, 1.8f);
		const std::vector<glm::mat4> poses = {
			glm::mat4(1.f),                         // camera 0 = the reference / world
			lookAtCv({ 1.3f, -0.3f, 0.5f }, target), // right, a bit higher (y is down)
			lookAtCv({ -1.1f, 0.2f, 0.7f }, target), // left, a bit lower
		};
		for (size_t i = 0; i < poses.size(); i++) {
			auto cam = std::make_unique<SimulatedCamera>();
			cam->setup("sim " + ofToString(i), intr, poses[i], simScene);
			cameras.push_back(std::move(cam));
		}
	} else {
		// real webcams, from bin/data/cameras.json
		if (!ofFile::doesFileExist("cameras.json")) {
			ofJson j;
			j["about"] = "One entry per webcam. intrinsics: OpenCV calibration file of that webcam (optional). "
						 "hfov: horizontal field of view used when there is no intrinsics file.";
			j["webcams"] = { { { "device", 0 }, { "width", 1280 }, { "height", 720 }, { "intrinsics", "webcam0_intrinsics.yml" }, { "hfov", 70 } },
				{ { "device", 1 }, { "width", 1280 }, { "height", 720 }, { "intrinsics", "webcam1_intrinsics.yml" }, { "hfov", 70 } } };
			ofSavePrettyJson("cameras.json", j);
		}
		for (auto & c : ofLoadJson("cameras.json")["webcams"]) {
			auto cam = std::make_unique<WebcamSource>();
			if (cam->setup(c.value("device", 0), c.value("width", 1280), c.value("height", 720), c.value("intrinsics", std::string()),
					c.value("hfov", 70.f))) {
				cameras.push_back(std::move(cam));
			} else {
				ofLogError() << "could not open webcam " << c.value("device", 0);
			}
		}
		if (cameras.size() < 2) message = "need at least 2 webcams (edit bin/data/cameras.json), found " + ofToString(cameras.size());
	}

	// one detector per camera, all with the same board and settings
	ofJson settings;
	ofSerialize(settings, detectorSettings.parameters);
	for (auto & cam : cameras) {
		auto d = std::make_unique<ofxAruco>();
		d->setup(board.getDictionary());
		ofDeserialize(settings, d->parameters);
		d->setIntrinsics(cam->getIntrinsics());
		d->addBoard(board);
		detectors.push_back(std::move(d));
	}

	calibration.setup(int(cameras.size()), 0);
	for (size_t i = 0; i < cameras.size(); i++) calibration.setCameraName(int(i), cameras[i]->getName());
}

std::vector<const ofxAruco *> ofApp::getDetectors() const {
	std::vector<const ofxAruco *> d;
	for (auto & a : detectors) d.push_back(a.get());
	return d;
}

//--------------------------------------------------------------
void ofApp::update() {
	if (simScene && simulated) simScene->update();
	for (size_t i = 0; i < cameras.size(); i++) {
		cameras[i]->update();
		if (cameras[i]->isFrameNew()) detectors[i]->detect(cameras[i]->getPixels());
	}

	// auto capture: adds a sample when the board is still in 2+ cameras
	if (calibration.update(getDetectors())) {
		for (size_t i = 0; i < detectors.size(); i++) {
			if (detectors[i]->getBoardPose().found) {
				samplePoses.emplace_back(int(i), detectors[i]->getBoardPose().pose);
				break;
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::drawFrustum(const ofxArucoIntrinsics & intr, float depth) const {
	// pyramid from the optical center to the image corners at `depth`, in the camera frame
	auto unproject = [&](float u, float v) {
		return glm::vec3((u - intr.getCx()) / intr.getFx() * depth, (v - intr.getCy()) / intr.getFy() * depth, depth);
	};
	const float w = float(intr.getWidth()), h = float(intr.getHeight());
	const glm::vec3 c[4] = { unproject(0, 0), unproject(w, 0), unproject(w, h), unproject(0, h) };
	for (int i = 0; i < 4; i++) {
		ofDrawLine(glm::vec3(0), c[i]);
		ofDrawLine(c[i], c[(i + 1) % 4]);
	}
	// small triangle on the top edge = image "up"
	const glm::vec3 top = (c[0] + c[1]) * 0.5f;
	ofDrawTriangle(top + glm::vec3(-0.15f * depth, 0, 0), top + glm::vec3(0.15f * depth, 0, 0), top + glm::vec3(0, -0.15f * depth, 0));
}

void ofApp::drawScene3D(const ofRectangle & viewport) {
	easyCam.begin(viewport);
	ofEnableDepthTest();
	// the calibration is in OpenCV convention (y down, z forward): show it with y up
	ofPushMatrix();
	ofMultMatrix(cvToGl());

	// world axes + a 2x2 m grid on the world's floor-like plane
	ofSetLineWidth(2);
	ofDrawAxis(0.3f);
	ofSetLineWidth(1);
	ofSetColor(70);
	for (float g = -2.f; g <= 2.01f; g += 0.25f) {
		ofDrawLine(glm::vec3(g, 1.f, 0), glm::vec3(g, 1.f, 4.f)); // 1 m "below" the reference camera
	}
	for (float g = 0; g <= 4.01f; g += 0.25f) {
		ofDrawLine(glm::vec3(-2, 1.f, g), glm::vec3(2, 1.f, g));
	}

	for (size_t i = 0; i < cameras.size(); i++) {
		// ground truth (simulation only)
		if (simulated && showTruth && calibration.isSolved(int(i))) {
			auto * sim = static_cast<SimulatedCamera *>(cameras[i].get());
			const glm::mat4 refToWorldTrue = rigidInverse(static_cast<SimulatedCamera *>(cameras[calibration.getReferenceCamera()].get())->getTrueCameraToWorld());
			ofPushMatrix();
			ofMultMatrix(calibration.getReferenceToWorld() * refToWorldTrue * sim->getTrueCameraToWorld());
			ofSetColor(120);
			drawFrustum(cameras[i]->getIntrinsics(), 0.25f);
			ofPopMatrix();
		}
		if (!calibration.isSolved(int(i))) continue;
		const glm::mat4 camToWorld = calibration.getCameraToWorld(int(i));
		ofPushMatrix();
		ofMultMatrix(camToWorld);
		ofSetColor(colorFor(int(i)));
		ofSetLineWidth(2);
		drawFrustum(cameras[i]->getIntrinsics(), 0.2f);
		ofDrawBitmapString(cameras[i]->getName(), 0, 0, 0);
		ofPopMatrix();

		// what this camera sees right now: the board, placed in the world through this camera
		const auto & bp = detectors[i]->getBoardPose();
		if (bp.found) {
			ofPushMatrix();
			ofMultMatrix(camToWorld * bp.pose);
			ofNoFill();
			ofDrawRectangle(0, 0, board.getSize().x, board.getSize().y);
			ofFill();
			ofPopMatrix();
		}
	}

	// coverage: where the board was for every sample
	if (showSamples) {
		ofSetColor(255, 255, 255, 90);
		ofNoFill();
		for (auto & s : samplePoses) {
			if (!calibration.isSolved(s.first)) continue;
			ofPushMatrix();
			ofMultMatrix(calibration.getCameraToWorld(s.first) * s.second);
			ofDrawRectangle(0, 0, board.getSize().x, board.getSize().y);
			ofPopMatrix();
		}
		ofFill();
	}

	ofPopMatrix();
	ofDisableDepthTest();
	easyCam.end();
}

void ofApp::draw() {
	// --- camera views (left column)
	const float viewW = 420;
	float y = 10;
	for (size_t i = 0; i < cameras.size(); i++) {
		const auto & intr = cameras[i]->getIntrinsics();
		const float viewH = viewW * intr.getHeight() / float(intr.getWidth());
		const ofRectangle r(10, y, viewW, viewH);
		ofSetColor(255);
		cameras[i]->draw(r);
		detectors[i]->drawMarkers(r, false);
		detectors[i]->drawBoards(r);
		detectors[i]->drawAxes(r);
		ofNoFill();
		ofSetColor(colorFor(int(i)));
		ofDrawRectangle(r);
		ofFill();

		const auto & bp = detectors[i]->getBoardPose();
		const auto & res = calibration.getCamera(int(i));
		std::stringstream ss;
		ss << cameras[i]->getName() << (int(i) == calibration.getReferenceCamera() ? " (reference)" : "") << "  "
		   << (bp.found ? "board " + ofToString(bp.numPoints) + " pts" : "no board");
		if (res.solved && res.refined) ss << "  reproj " << ofToString(res.reprojectionError, 2) << " px";
		if (cameras[i]->hasEstimatedIntrinsics()) ss << "  ESTIMATED intrinsics";
		ofDrawBitmapStringHighlight(ss.str(), r.x + 4, r.y + 14, ofColor(0, 180), colorFor(int(i)));
		y += viewH + 8;
	}

	// --- 3D view (right)
	const ofRectangle view3d(viewW + 20, 10, ofGetWidth() - viewW - 260, ofGetHeight() - 190);
	ofSetColor(40);
	ofDrawRectangle(view3d);
	drawScene3D(view3d);

	// --- "hold still" progress
	const float p = calibration.getStillProgress();
	ofSetColor(60);
	ofDrawRectangle(view3d.x, view3d.getBottom() + 6, view3d.width, 10);
	ofSetColor(p >= 1 ? ofColor(0, 255, 0) : ofColor(0, 160, 255));
	ofDrawRectangle(view3d.x, view3d.getBottom() + 6, view3d.width * p, 10);

	// --- status
	std::stringstream ss;
	ss << "samples: " << calibration.getNumSamples() << "   auto capture: " << (calibration.autoCapture ? "on" : "off")
	   << "   mode: " << (simulated ? "SIMULATED cameras" : "webcams") << "\n";
	for (int i = 0; i < calibration.getNumCameras(); i++) {
		const auto & c = calibration.getCamera(i);
		ss << "  " << c.name << ": ";
		if (i == calibration.getReferenceCamera()) {
			ss << "reference";
		} else if (!c.solved) {
			ss << "not calibrated yet (show the board to it and to a calibrated camera at the same time)";
		} else {
			const glm::vec3 pos = glm::vec3(calibration.getCameraToWorld(i)[3]);
			ss << c.samples << " samples, reproj " << ofToString(c.reprojectionError, 2) << " px, position " << ofToString(pos, 3) << " m";
			if (simulated && showTruth) {
				auto * ref = static_cast<SimulatedCamera *>(cameras[calibration.getReferenceCamera()].get());
				auto * sim = static_cast<SimulatedCamera *>(cameras[i].get());
				const glm::mat4 truth = rigidInverse(ref->getTrueCameraToWorld()) * sim->getTrueCameraToWorld();
				const glm::mat4 est = calibration.getCameraToReference(i);
				ss << "   | error vs truth " << ofToString(glm::distance(glm::vec3(truth[3]), glm::vec3(est[3])) * 1000, 1) << " mm, "
				   << ofToString(rotationAngleDeg(truth, est), 2) << " deg";
			}
		}
		ss << "\n";
	}
	ss << "\nhold the board still in 2+ cameras: auto sample | space: sample now | z: undo | c: clear\n"
	   << "w: world origin = board (now) | r: world = camera 0 | s: save | l: load | m: simulated / webcams";
	ofDrawBitmapStringHighlight(ss.str(), view3d.x, view3d.getBottom() + 36);
	if (!message.empty() && ofGetElapsedTimef() - messageTime < 4) {
		ofDrawBitmapStringHighlight(message, view3d.x + 10, view3d.y + 20, ofColor(0, 0, 0, 200), ofColor::yellow);
	}

	gui.draw();
}

//--------------------------------------------------------------
void ofApp::setWorldFromCurrentBoard() {
	for (size_t i = 0; i < detectors.size(); i++) {
		if (detectors[i]->getBoardPose().found && calibration.isSolved(int(i))) {
			calibration.setWorldFromBoard(int(i), detectors[i]->getBoardPose().pose);
			message = "world origin = board (seen by " + cameras[i]->getName() + ")";
			messageTime = ofGetElapsedTimef();
			return;
		}
	}
	message = "no calibrated camera sees the board";
	messageTime = ofGetElapsedTimef();
}

void ofApp::keyPressed(int key) {
	auto say = [this](const std::string & m) {
		message = m;
		messageTime = ofGetElapsedTimef();
	};
	switch (key) {
	case ' ':
		if (calibration.addSample(getDetectors())) {
			for (size_t i = 0; i < detectors.size(); i++) {
				if (detectors[i]->getBoardPose().found) {
					samplePoses.emplace_back(int(i), detectors[i]->getBoardPose().pose);
					break;
				}
			}
			say("sample added");
		} else {
			say("sample skipped: the board must be seen by 2+ cameras");
		}
		break;
	case 'a': calibration.autoCapture = !calibration.autoCapture; break;
	case 'z':
		calibration.removeLastSample();
		if (!samplePoses.empty()) samplePoses.pop_back();
		break;
	case 'c':
		calibration.clear();
		samplePoses.clear();
		break;
	case 'w': setWorldFromCurrentBoard(); break;
	case 'r': calibration.setWorldToReferenceCamera(); break;
	case 's':
		calibration.save("multicam_calibration.json");
		gui.saveToFile("settings.json");
		say("saved bin/data/multicam_calibration.json");
		break;
	case 'l':
		say(calibration.load("multicam_calibration.json") ? "loaded multicam_calibration.json" : "no multicam_calibration.json");
		break;
	case 'm': setupCameras(!simulated); break;
	}
}

void ofApp::exit() {
	gui.saveToFile("settings.json");
}
