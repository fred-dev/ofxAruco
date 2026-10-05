#include "ofApp.h"
#include <glm/gtx/euler_angles.hpp>

using namespace ofxArucoUtils;

namespace {
	// Camera pose (camera -> world) looking from `eye` to `target`, OpenCV convention (y down).
	glm::mat4 lookAtCv(glm::vec3 eye, glm::vec3 target) {
		const glm::vec3 z = glm::normalize(target - eye);
		const glm::vec3 x = glm::normalize(glm::cross(glm::vec3(0, 1, 0), z));
		const glm::vec3 y = glm::cross(z, x);
		glm::mat4 m(1.f);
		m[0] = glm::vec4(x, 0);
		m[1] = glm::vec4(y, 0);
		m[2] = glm::vec4(z, 0);
		m[3] = glm::vec4(eye, 1);
		return m;
	}

	glm::mat4 rotXYZ(float ax, float ay, float az) {
		return glm::eulerAngleXYZ(glm::radians(ax), glm::radians(ay), glm::radians(az));
	}

	float transErrMm(const glm::mat4 & a, const glm::mat4 & b) {
		return glm::distance(glm::vec3(a[3]), glm::vec3(b[3])) * 1000.f;
	}
}

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowTitle("ofxAruco tests");
	ofSetFrameRate(60);

	setBoard(ofxArucoBoard::makeCharuco(7, 5, 0.04f, 0.03f, cv::aruco::DICT_5X5_100));
	report << "ofxAruco self test  (" << ofGetTimestampString() << ")\n\n";
}

void ofApp::setBoard(const ofxArucoBoard & b) {
	board = b;
	// texture with exactly the board's proportions (margin from whole pixels)
	ofPixels boardPixels;
	const float ppm = 5000; // pixels per meter of the texture
	const int boardPx = int(std::round(board.getSize().x * ppm));
	const int marginPx = int(std::round(0.02f * ppm));
	board.getImage(boardPixels, boardPx, marginPx);
	boardMarginMeters = marginPx * board.getSize().x / boardPx;
	boardTexture.clear();
	ofDisableArbTex(); // GL_TEXTURE_2D so it can have mipmaps
	boardTexture.allocate(boardPixels);
	ofEnableArbTex();
	boardTexture.loadData(boardPixels);
	boardTexture.generateMipmap();
	boardTexture.setTextureMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
	// anisotropic filtering: without it, boards seen at grazing angles get blurred much more
	// in one direction, which shifts the detected corners (a renderer artifact, not real optics)
	boardTexture.bind();
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 16.f);
	boardTexture.unbind();
}

void ofApp::check(bool ok, const std::string & what) {
	if (!ok) failures++;
	report << (ok ? "  [ OK ] " : "  [FAIL] ") << what << "\n";
	ofLogNotice("test") << (ok ? "[ OK ] " : "[FAIL] ") << what;
}

void ofApp::render(const ofxArucoIntrinsics & cam, const glm::mat4 & boardToCamera, ofPixels & pixels) {
	const int w = cam.getWidth(), h = cam.getHeight();
	if (!fbo.isAllocated() || fbo.getWidth() != w || fbo.getHeight() != h) {
		ofFboSettings s;
		s.width = w;
		s.height = h;
		s.internalformat = GL_RGB;
		s.useDepth = true;
		s.numSamples = 4;
		fbo.allocate(s);
	}
	const glm::vec2 size = board.getSize();
	const float m = boardMarginMeters;
	ofMesh quad;
	quad.setMode(OF_PRIMITIVE_TRIANGLE_FAN);
	// board frame: x right, y down; the texture's top-left is the board's top-left
	quad.addVertex({ -m, -m, 0 });
	quad.addTexCoord(boardTexture.getCoordFromPercent(0, 0));
	quad.addVertex({ size.x + m, -m, 0 });
	quad.addTexCoord(boardTexture.getCoordFromPercent(1, 0));
	quad.addVertex({ size.x + m, size.y + m, 0 });
	quad.addTexCoord(boardTexture.getCoordFromPercent(1, 1));
	quad.addVertex({ -m, size.y + m, 0 });
	quad.addTexCoord(boardTexture.getCoordFromPercent(0, 1));

	fbo.begin();
	ofClear(70, 70, 70, 255);
	cam.begin(ofRectangle(0, 0, w, h));
	ofMultMatrix(boardToCamera);
	ofSetColor(255);
	boardTexture.bind();
	quad.draw();
	boardTexture.unbind();
	cam.end();
	fbo.end();
	fbo.readToPixels(pixels);
}

//--------------------------------------------------------------
void ofApp::runSyntheticTests() {
	ofxArucoIntrinsics cam;
	cam.setup(900, 900, 639.5, 359.5, 1280, 720);

	// ---------------------------------------------------------------- 1. single camera pose
	report << "1. Pose of a rendered ChArUco board (1280x720, f=900)\n";
	{
		ofxAruco aruco;
		aruco.setup(cv::aruco::DICT_5X5_100, false); // main thread, results immediately
		aruco.setIntrinsics(cam);
		aruco.markerLength = 0.03f;
		aruco.addBoard(board);

		const glm::vec3 c = board.getCenter();
		std::vector<glm::mat4> poses = {
			glm::translate(glm::mat4(1), { 0.0f, 0.0f, 0.6f }) * glm::translate(glm::mat4(1), -c),
			glm::translate(glm::mat4(1), { 0.1f, -0.05f, 0.9f }) * rotXYZ(20, -30, 10) * glm::translate(glm::mat4(1), -c),
			glm::translate(glm::mat4(1), { -0.15f, 0.08f, 1.4f }) * rotXYZ(-35, 25, -20) * glm::translate(glm::mat4(1), -c),
			glm::translate(glm::mat4(1), { 0.0f, 0.0f, 2.0f }) * rotXYZ(0, 50, 0) * glm::translate(glm::mat4(1), -c),
		};
		float worstT = 0, worstR = 0;
		int found = 0;
		for (auto & gt : poses) {
			render(cam, gt, lastRender);
			aruco.detect(lastRender);
			const auto & bp = aruco.getBoardPose(0);
			if (bp.found) {
				found++;
				worstT = std::max(worstT, transErrMm(bp.pose, gt));
				worstR = std::max(worstR, rotationAngleDeg(bp.pose, gt));
				report << "     dist " << ofToString(glm::length(glm::vec3(gt[3])), 2) << " m: "
					   << bp.numPoints << " corners, err " << ofToString(transErrMm(bp.pose, gt), 2) << " mm / "
					   << ofToString(rotationAngleDeg(bp.pose, gt), 3) << " deg, reproj "
					   << ofToString(bp.reprojectionError, 3) << " px, markers " << aruco.getNumMarkers() << "\n";
			}
		}
		check(found == (int)poses.size(), "board found in all " + ofToString(poses.size()) + " views (" + ofToString(found) + ")");
		check(worstT < 5.f, "board translation error < 5 mm (worst " + ofToString(worstT, 2) + " mm)");
		check(worstR < 0.5f, "board rotation error < 0.5 deg (worst " + ofToString(worstR, 3) + " deg)");

		// single marker pose: marker frame center = marker center, z towards camera
		render(cam, poses[0], lastRender);
		aruco.detect(lastRender);
		check(aruco.getNumMarkers() == board.getNumMarkers(),
			"all " + ofToString(board.getNumMarkers()) + " markers found in the frontal view (" + ofToString(aruco.getNumMarkers()) + ")");
		bool markerOk = !aruco.getMarkers().empty();
		float worstMarker = 0, worstFacing = 0;
		for (auto & mk : aruco.getMarkers()) {
			if (!mk.hasPose) {
				markerOk = false;
				continue;
			}
			// the board's frontal pose has z into the board, the marker's z points to the camera
			const float facing = glm::dot(glm::vec3(mk.pose[2]), glm::vec3(poses[0][2]));
			worstFacing = std::max(worstFacing, glm::degrees(std::acos(glm::clamp(-facing, -1.f, 1.f))));
			const glm::vec2 pc = cam.project(mk.getPosition());
			worstMarker = std::max(worstMarker, glm::distance(pc, mk.getCenter()));
		}
		// Small frontal markers have two almost equally good orientations (tilted one way or the
		// other, here up to ~20 deg): that's why boards exist. What must never happen is a
		// flipped solution (z pointing away, ~180 deg off).
		check(markerOk && worstFacing < 30.f, "single marker poses face the camera, no flipped solutions (worst " + ofToString(worstFacing, 1) + " deg off)");
		check(worstMarker < 1.f, "single marker positions reproject onto marker centers (worst " + ofToString(worstMarker, 2) + " px)");

		// legacy draw path: projection matrix must project like OpenCV
		const glm::vec3 p(0.1f, -0.05f, 1.f);
		glm::vec4 clip = cam.getProjectionMatrix() * cvToGl() * glm::vec4(p, 1);
		glm::vec2 ndc = glm::vec2(clip) / clip.w;
		glm::vec2 px((ndc.x + 1) * 0.5f * 1280 - 0.5f, (1 - ndc.y) * 0.5f * 720 - 0.5f);
		check(glm::distance(px, cam.project(p)) < 0.01f, "OpenGL projection matches OpenCV projection");
	}

	// ---------------------------------------------------------------- 2. threaded == main thread, 16 bit
	report << "\n2. Threading and input formats\n";
	{
		const glm::vec3 c = board.getCenter();
		const glm::mat4 gt = glm::translate(glm::mat4(1), { 0.05f, 0.02f, 1.0f }) * rotXYZ(15, 20, 5) * glm::translate(glm::mat4(1), -c);
		render(cam, gt, lastRender);

		ofxAruco sync, async;
		for (auto * a : { &sync, &async }) {
			a->setup(cv::aruco::DICT_5X5_100, a == &async);
			a->setIntrinsics(cam);
			a->addBoard(board);
		}
		sync.detect(lastRender);
		async.detect(lastRender, 1234);
		async.waitForResult(2000);
		check(async.isFrameNew() && async.getResult().userTag == 1234, "threaded result arrives with its user tag");
		check(async.getNumMarkers() == sync.getNumMarkers() && async.getBoardPose().found
				&& transErrMm(async.getBoardPose().pose, sync.getBoardPose().pose) < 1e-3f,
			"threaded and main thread results are identical");

		// flood the worker: it must keep only the newest frame and never block
		const uint64_t t0 = ofGetElapsedTimeMicros();
		for (int i = 0; i < 200; i++) async.detect(lastRender, i);
		const float submitMs = (ofGetElapsedTimeMicros() - t0) / 1000.f / 200;
		async.waitForResult(5000);
		check(async.getResult().userTag == 199, "after 200 quick frames the latest result is the newest frame (tag " + ofToString(async.getResult().userTag) + ")");
		check(submitMs < async.getDetectMillis(), "detect() doesn't wait for the detection (" + ofToString(submitMs, 2) + " ms vs " + ofToString(async.getDetectMillis(), 2) + " ms)");
		report << "     detect() cost on the calling thread: " << ofToString(submitMs, 3) << " ms (1280x720 RGB)\n";
		report << "     detection time on the worker: " << ofToString(async.getDetectMillis(), 2) << " ms\n";

		async.setThreaded(false);
		async.detect(lastRender, 7);
		check(async.getResult().userTag == 7 && async.getBoardPose().found, "switching threaded -> main thread at runtime");
		async.setThreaded(true);
		async.detect(lastRender, 8);
		async.waitForResult();
		check(async.getResult().userTag == 8 && async.getBoardPose().found, "switching main thread -> threaded at runtime");

		// 16 bit input (like a Kinect IR image): gray * 12
		ofShortPixels ir;
		ir.allocate(lastRender.getWidth(), lastRender.getHeight(), 1);
		for (size_t i = 0; i < ir.size(); i++) ir[i] = lastRender[i * 3] * 12;
		sync.detect(ir);
		check(sync.getBoardPose().found && transErrMm(sync.getBoardPose().pose, gt) < 5, "16 bit (IR style) input with automatic scaling");

		// image at a different size than the intrinsics: intrinsics get scaled
		ofPixels half = lastRender;
		half.resize(640, 360, OF_INTERPOLATE_BICUBIC);
		sync.detect(half);
		check(sync.getBoardPose().found && transErrMm(sync.getBoardPose().pose, gt) < 10,
			"half resolution image with full resolution intrinsics (err " + ofToString(transErrMm(sync.getBoardPose().pose, gt), 2) + " mm)");

		// fast mode: only finds markers bigger than ~ aruco3MinSide + aruco3MinMarkerRatio * width
		// = 32 + 0.01 * 1280 = 45 px. At 0.4 m the markers are ~65 px.
		const glm::mat4 close = glm::translate(glm::mat4(1), { 0.0f, 0.0f, 0.4f }) * rotXYZ(10, 15, 0) * glm::translate(glm::mat4(1), -c);
		render(cam, close, lastRender);
		sync.detect(lastRender);
		const float normalMs = sync.getDetectMillis();
		sync.useAruco3Detection = true;
		sync.aruco3MinMarkerRatio = 0.01f; // downscale the image internally
		sync.detect(lastRender);
		report << "     aruco3: " << sync.getNumMarkers() << " markers, " << sync.getBoardPose().numPoints << " board points, "
			   << ofToString(sync.getDetectMillis(), 2) << " ms (normal mode " << ofToString(normalMs, 2) << " ms)\n";
		check(sync.getBoardPose().found && transErrMm(sync.getBoardPose().pose, close) < 5, "aruco3 fast detection finds the board");
	}

	// ---------------------------------------------------------------- 3. multi camera calibration
	report << "\n3. Multi camera calibration (3 virtual cameras, world = camera 0)\n";
	{
		const glm::vec3 target(0, 0, 1.4f);
		std::vector<glm::mat4> camToWorld = {
			glm::mat4(1.f),
			lookAtCv({ 0.9f, -0.15f, 0.2f }, target),
			lookAtCv({ -0.7f, 0.25f, 0.4f }, target),
		};
		std::vector<std::unique_ptr<ofxAruco>> detectors;
		for (int i = 0; i < 3; i++) {
			detectors.emplace_back(std::make_unique<ofxAruco>());
			detectors.back()->setup(cv::aruco::DICT_5X5_100, false);
			detectors.back()->setIntrinsics(cam);
			detectors.back()->addBoard(board);
		}
		ofxArucoMultiCamCalibration calib;
		calib.setup(3, 0);

		ofSeedRandom(42);
		const glm::vec3 c = board.getCenter();
		int samples = 0;
		for (int s = 0; s < 14; s++) {
			// board faces roughly between the cameras
			const glm::mat4 boardToWorld = glm::translate(glm::mat4(1), target + glm::vec3(ofRandom(-0.2f, 0.2f), ofRandom(-0.15f, 0.15f), ofRandom(-0.2f, 0.3f)))
				* rotXYZ(ofRandom(-25, 25), ofRandom(-25, 25), ofRandom(-30, 30)) * glm::translate(glm::mat4(1), -c);
			for (int k = 0; k < 3; k++) {
				const glm::mat4 boardToCam = rigidInverse(camToWorld[k]) * boardToWorld;
				render(cam, boardToCam, lastRender);
				detectors[k]->detect(lastRender);
			}
			// the normal way: hand the detectors to the calibrator (it takes poses, corners and intrinsics)
			if (calib.addSample({ detectors[0].get(), detectors[1].get(), detectors[2].get() })) samples++;
		}
		lastRenderImage.setFromPixels(lastRender);
		check(samples >= 10, "samples accepted: " + ofToString(samples) + " / 14");
		check(calib.isSolved(), "all cameras solved");

		auto reportCameras = [&](const std::string & label, float maxMm, float maxDeg) {
			for (int k = 1; k < 3; k++) {
				const auto & res = calib.getCamera(k);
				const float te = transErrMm(calib.getCameraToWorld(k), camToWorld[k]);
				const float re = rotationAngleDeg(calib.getCameraToWorld(k), camToWorld[k]);
				report << "     " << label << " camera " << k << ": " << res.samples << " samples, spread " << ofToString(res.translationStdMm, 2)
					   << " mm / " << ofToString(res.rotationStdDeg, 3) << " deg, reprojection " << ofToString(res.reprojectionError, 3) << " px\n";
				check(te < maxMm && re < maxDeg, label + " camera " + ofToString(k) + " pose error " + ofToString(te, 2) + " mm / " + ofToString(re, 3)
						+ " deg (< " + ofToString(maxMm) + " mm, < " + ofToString(maxDeg) + " deg)");
			}
		};
		// averaging only (no bundle adjustment): a few mm
		calib.bundleAdjust = false;
		calib.solve();
		reportCameras("averaged:", 6.f, 0.3f);
		// with bundle adjustment (default): sub-millimeter
		calib.bundleAdjust = true;
		const uint64_t t0 = ofGetElapsedTimeMicros();
		calib.solve();
		report << "     bundle adjustment of " << samples << " samples took " << ofToString((ofGetElapsedTimeMicros() - t0) / 1000.f, 1) << " ms\n";
		reportCameras("refined: ", 2.f, 0.05f);
		check(calib.getCamera(1).refined && calib.getCamera(1).reprojectionError < 0.5f, "refined reprojection error < 0.5 px");

		// a point seen by camera 2 maps to the same world point through camera 1
		const glm::vec3 pw(0.1f, 0.2f, 1.5f);
		const glm::vec3 p2 = glm::vec3(rigidInverse(camToWorld[2]) * glm::vec4(pw, 1));
		const glm::vec3 p1 = glm::vec3(calib.getCameraToCamera(2, 1) * glm::vec4(p2, 1));
		const glm::vec3 p1gt = glm::vec3(rigidInverse(camToWorld[1]) * glm::vec4(pw, 1));
		check(glm::distance(p1, p1gt) * 1000 < 2, "camera 2 -> camera 1 point transfer error " + ofToString(glm::distance(p1, p1gt) * 1000, 2) + " mm");

		// world from board
		const glm::mat4 boardToWorld = glm::translate(glm::mat4(1), target);
		calib.setWorldFromBoard(1, rigidInverse(camToWorld[1]) * boardToWorld);
		const glm::mat4 cam0InBoard = calib.getCameraToWorld(0);
		check(transErrMm(cam0InBoard, rigidInverse(boardToWorld)) < 2 && rotationAngleDeg(cam0InBoard, rigidInverse(boardToWorld)) < 0.05f,
			"setWorldFromBoard() puts the world origin on the board");

		// save / load
		calib.save("test_calibration.json");
		ofxArucoMultiCamCalibration loaded;
		loaded.load("test_calibration.json");
		check(loaded.getNumCameras() == 3 && transErrMm(loaded.getCameraToWorld(2), calib.getCameraToWorld(2)) < 0.01f
				&& rotationAngleDeg(loaded.getCameraToWorld(2), calib.getCameraToWorld(2)) < 0.001f,
			"calibration JSON round trip");
	}

	// ---------------------------------------------------------------- 3b. example-multi-camera's setup, threaded
	report << "\n3b. Same setup as example-multi-camera (55 mm board at ~1.8 m, f=920), threaded detectors\n";
	{
		const ofxArucoBoard keep = board;
		setBoard(ofxArucoBoard::makeCharuco(7, 5, 0.055f, 0.041f, cv::aruco::DICT_5X5_100));
		ofxArucoIntrinsics intr;
		intr.setup(920, 920, 639.5, 359.5, 1280, 720);
		const glm::vec3 target(0, 0, 1.8f);
		const std::vector<glm::mat4> camToWorld = { glm::mat4(1.f), lookAtCv({ 1.3f, -0.3f, 0.5f }, target), lookAtCv({ -1.1f, 0.2f, 0.7f }, target) };
		std::vector<std::unique_ptr<ofxAruco>> detectors;
		for (int i = 0; i < 3; i++) {
			detectors.emplace_back(std::make_unique<ofxAruco>());
			detectors.back()->setup(cv::aruco::DICT_5X5_100, true);
			detectors.back()->setIntrinsics(intr);
			detectors.back()->addBoard(board);
		}
		ofxArucoMultiCamCalibration calib;
		calib.setup(3, 0);
		ofSeedRandom(7);
		const glm::vec3 c = board.getCenter();
		float worstSingle = 0;
		for (int s = 0; s < 25; s++) {
			const glm::vec3 center(ofRandom(-0.35f, 0.35f), ofRandom(-0.2f, 0.25f), ofRandom(1.5f, 2.1f));
			const glm::mat4 boardToWorld = glm::translate(glm::mat4(1), center) * rotXYZ(ofRandom(-25.f, 25.f), ofRandom(-30.f, 30.f), ofRandom(-20.f, 20.f))
				* glm::translate(glm::mat4(1), -c);
			for (int k = 0; k < 3; k++) {
				const glm::mat4 boardToCam = rigidInverse(camToWorld[k]) * boardToWorld;
				render(intr, boardToCam, lastRender);
				detectors[k]->detect(lastRender);
				detectors[k]->waitForResult();
				if (detectors[k]->getBoardPose().found) worstSingle = std::max(worstSingle, transErrMm(detectors[k]->getBoardPose().pose, boardToCam));
			}
			calib.addSample({ detectors[0].get(), detectors[1].get(), detectors[2].get() });
		}
		report << "     worst single board pose error " << ofToString(worstSingle, 2) << " mm\n";
		for (int k = 1; k < 3; k++) {
			const auto & res = calib.getCamera(k);
			const float te = transErrMm(calib.getCameraToWorld(k), camToWorld[k]);
			const float re = rotationAngleDeg(calib.getCameraToWorld(k), camToWorld[k]);
			report << "     camera " << k << ": " << res.samples << " samples, reprojection " << ofToString(res.reprojectionError, 3) << " px\n";
			check(te < 2.f && re < 0.1f, "camera " + ofToString(k) + " pose error " + ofToString(te, 2) + " mm / " + ofToString(re, 3) + " deg (< 2 mm, < 0.1 deg)");
		}
		setBoard(keep);
	}

	// ---------------------------------------------------------------- 4. files
	report << "\n4. Files\n";
	{
		board.save("test_board.json");
		ofxArucoBoard b2;
		check(b2.load("test_board.json") && b2.getType() == ofxArucoBoard::Type::Charuco && b2.getNumMarkers() == board.getNumMarkers()
				&& b2.getSize() == board.getSize(),
			"board JSON round trip");
		auto grid = ofxArucoBoard::makeGrid(4, 3, 0.05f, 0.01f, cv::aruco::DICT_4X4_50, 10);
		grid.save("test_grid.json");
		check(b2.load("test_grid.json") && b2.getType() == ofxArucoBoard::Type::Grid && b2.getCvBoard().getIds().front() == 10, "grid board JSON round trip");

		cam.save("test_intrinsics.json");
		cam.save("test_intrinsics.yml");
		ofxArucoIntrinsics a, b;
		check(a.load("test_intrinsics.json") && a.getFx() == 900 && a.getCx() == 639.5 && a.getWidth() == 1280, "intrinsics JSON round trip");
		check(b.load("test_intrinsics.yml") && b.getFy() == 900 && b.getCy() == 359.5 && b.getHeight() == 720, "intrinsics OpenCV yml round trip");

		ofxArucoIntrinsics legacy;
		check(legacy.load("../../../../example/bin/data/intrinsics.int") && std::abs(legacy.getFx() - 628.158) < 1e-3, "legacy ArUco .int intrinsics");
		ofxArucoIntrinsics old;
		check(old.load("../../../../example/bin/data/intrinsics.yml") && old.getWidth() == 640, "example intrinsics.yml");

		ofxArucoBoard legacyBoard;
		check(legacyBoard.loadLegacyAruco("../../../../example/bin/data/boardConfiguration.yml", 0.035f, cv::aruco::DICT_ARUCO_ORIGINAL)
				&& legacyBoard.getNumMarkers() == 24,
			"legacy ArUco 1.x board (24 markers)");

		ofxAruco s;
		s.adaptiveThreshConstant = 11;
		s.cornerRefinement = 2;
		s.saveSettings("test_settings.json");
		ofxAruco s2;
		s2.loadSettings("test_settings.json");
		check(s2.adaptiveThreshConstant == 11 && s2.cornerRefinement == 2, "detector settings JSON round trip");

		ofPixels markerPix;
		ofxAruco::getMarkerImage(cv::aruco::DICT_5X5_100, 7, 400, markerPix);
		ofxAruco m;
		m.setup(cv::aruco::DICT_5X5_100, false);
		m.detect(markerPix);
		check(m.getNumMarkers() == 1 && m.getMarkers()[0].id == 7, "generated marker image is detected with the right id");
	}
}

//--------------------------------------------------------------
void ofApp::update() {
	if (stage == 0 && ofGetFrameNum() > 2) {
		runSyntheticTests();
		stage = 1;

		report << "\n5. Example video (videoboard.mp4, DICT_ARUCO_ORIGINAL, threaded)\n";
		videoAruco.setup(cv::aruco::DICT_ARUCO_ORIGINAL);
		videoAruco.loadIntrinsics("../../../../example/bin/data/intrinsics.yml");
		videoAruco.markerLength = 0.035f;
		ofxArucoBoard legacyBoard;
		legacyBoard.loadLegacyAruco("../../../../example/bin/data/boardConfiguration.yml", 0.035f, cv::aruco::DICT_ARUCO_ORIGINAL);
		videoAruco.addBoard(legacyBoard);
		if (video.load("../../../../example/bin/data/videoboard.mp4")) {
			video.play();
			videoStart = ofGetElapsedTimef();
		} else {
			check(false, "could not load videoboard.mp4");
			stage = 2;
		}
	}
	if (stage == 1) {
		video.update();
		if (video.isFrameNew()) videoAruco.detect(video.getPixels());
		if (videoAruco.isFrameNew()) {
			const auto & r = videoAruco.getResult();
			videoFrames++;
			if (!r.markers.empty()) videoFramesWithMarkers++;
			videoMarkerSum += r.markers.size();
			if (videoAruco.getBoardPose().found) {
				videoFramesWithBoard++;
				videoReprojSum += videoAruco.getBoardPose().reprojectionError;
			}
		}
		if (ofGetElapsedTimef() - videoStart > 8) {
			report << "     " << videoFrames << " frames processed, markers in " << videoFramesWithMarkers << ", avg "
				   << ofToString(videoMarkerSum / std::max(1, videoFrames), 1) << " markers/frame, board in " << videoFramesWithBoard
				   << " (avg reproj " << ofToString(videoReprojSum / std::max(1, videoFramesWithBoard), 2) << " px), "
				   << ofToString(videoAruco.getDetectMillis(), 1) << " ms/frame\n";
			check(videoFrames > 50, "video frames processed by the worker");
			check(videoFramesWithMarkers > videoFrames * 0.8, "markers found in > 80% of the video frames");
			check(videoFramesWithBoard > videoFrames * 0.5, "board pose in > 50% of the video frames");
			stage = 2;
		}
	}
	if (stage == 2) {
		finish();
		stage = 3;
	}
}

void ofApp::finish() {
	report << "\n" << (failures == 0 ? "ALL TESTS PASSED" : ofToString(failures) + " TEST(S) FAILED") << "\n";
	ofBufferToFile("test_report.txt", ofBuffer(report.str().data(), report.str().size()));
	std::cout << "\n" << report.str() << std::endl;
	if (lastRenderImage.isAllocated()) lastRenderImage.save("test_last_render.png");
	ofExit(failures);
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(0);
	ofSetColor(255);
	if (stage == 1) {
		video.draw(0, 0);
		videoAruco.draw();
	} else if (lastRenderImage.isAllocated()) {
		lastRenderImage.draw(0, 0, 640, 360);
	}
	ofDrawBitmapStringHighlight("running ofxAruco tests...", 20, ofGetHeight() - 20);
}
