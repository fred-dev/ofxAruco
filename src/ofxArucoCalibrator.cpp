#include "ofxArucoCalibrator.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <numeric>

namespace {
cv::Mat toGray(const ofPixels & pixels) {
	const int ch = int(pixels.getNumChannels());
	cv::Mat src(int(pixels.getHeight()), int(pixels.getWidth()), CV_8UC(ch), const_cast<unsigned char *>(pixels.getData()));
	cv::Mat gray;
	switch (ch) {
	case 1: gray = src.clone(); break;
	case 3: cv::cvtColor(src, gray, pixels.getPixelFormat() == OF_PIXELS_BGR ? cv::COLOR_BGR2GRAY : cv::COLOR_RGB2GRAY); break;
	case 4: cv::cvtColor(src, gray, pixels.getPixelFormat() == OF_PIXELS_BGRA ? cv::COLOR_BGRA2GRAY : cv::COLOR_RGBA2GRAY); break;
	default: cv::extractChannel(src, gray, 0); break;
	}
	return gray;
}
}

//--------------------------------------------------------------
struct ofxArucoCalibrator::Worker {
	std::unique_ptr<cv::aruco::CharucoDetector> detector;
	std::thread thread;
	std::mutex mutex;
	std::condition_variable condition;
	cv::Mat pending;
	bool hasPending = false;
	bool busy = false;
	bool running = true;
	Detection result;
	bool hasResult = false;

	void loop() {
		while (true) {
			cv::Mat gray;
			{
				std::unique_lock<std::mutex> lock(mutex);
				condition.wait(lock, [this] { return hasPending || !running; });
				if (!running) return;
				gray = std::move(pending);
				pending = cv::Mat();
				hasPending = false;
				busy = true;
			}
			Detection d;
			d.imageSize = gray.size();
			try {
				std::vector<std::vector<cv::Point2f>> markerCorners;
				std::vector<int> markerIds;
				detector->detectBoard(gray, d.corners, d.ids, markerCorners, markerIds);
			} catch (const cv::Exception & e) {
				ofLogError("ofxArucoCalibrator") << "detection failed: " << e.what();
				d.corners.clear();
				d.ids.clear();
			}
			{
				std::lock_guard<std::mutex> lock(mutex);
				result = std::move(d);
				hasResult = true;
				busy = false;
			}
		}
	}
};

ofxArucoCalibrator::~ofxArucoCalibrator() {
	stopWorker();
}

void ofxArucoCalibrator::stopWorker() {
	if (!worker) return;
	{
		std::lock_guard<std::mutex> lock(worker->mutex);
		worker->running = false;
	}
	worker->condition.notify_all();
	if (worker->thread.joinable()) worker->thread.join();
	worker.reset();
}

bool ofxArucoCalibrator::setup(const ofxArucoBoard & b) {
	stopWorker();
	if (b.getType() != ofxArucoBoard::Type::Charuco) {
		ofLogError("ofxArucoCalibrator") << "setup: needs a ChArUco board";
		return false;
	}
	board = b;
	worker = std::make_unique<Worker>();
	cv::aruco::CharucoParameters charucoParams;
	charucoParams.tryRefineMarkers = true;
	cv::aruco::DetectorParameters detectorParams;
	worker->detector = std::make_unique<cv::aruco::CharucoDetector>(board.getCvCharuco(), charucoParams, detectorParams);
	Worker * w = worker.get();
	worker->thread = std::thread([w] { w->loop(); });
	clear();
	return true;
}

void ofxArucoCalibrator::start(cv::Size size) {
	imageSize = size;
	clear();
}

void ofxArucoCalibrator::clear() {
	samples.clear();
	detection = Detection();
	previousDetection = Detection();
	stillSince = -1;
	lastSampleTime = -1000;
	status = "show the board to the camera";
}

bool ofxArucoCalibrator::isStill() const {
	return stillSince >= 0 && ofGetElapsedTimef() - stillSince >= settings.stillSeconds;
}

bool ofxArucoCalibrator::update(const ofPixels & pixels) {
	if (!worker || !pixels.isAllocated()) return false;
	if (imageSize.width <= 0) imageSize = cv::Size(int(pixels.getWidth()), int(pixels.getHeight()));

	// hand the newest frame to the worker when it is idle
	bool idle;
	{
		std::lock_guard<std::mutex> lock(worker->mutex);
		idle = !worker->hasPending && !worker->busy;
	}
	if (idle) {
		cv::Mat gray = toGray(pixels);
		{
			std::lock_guard<std::mutex> lock(worker->mutex);
			worker->pending = std::move(gray);
			worker->hasPending = true;
		}
		worker->condition.notify_one();
	}

	// pick up a result
	Detection d;
	{
		std::lock_guard<std::mutex> lock(worker->mutex);
		if (!worker->hasResult) return false;
		d = std::move(worker->result);
		worker->hasResult = false;
	}
	if (d.imageSize != imageSize) {
		status = "the camera delivers " + ofToString(d.imageSize.width) + "x" + ofToString(d.imageSize.height) + ", expected "
			+ ofToString(imageSize.width) + "x" + ofToString(imageSize.height);
		return false;
	}
	previousDetection = std::move(detection);
	detection = std::move(d);
	const float now = ofGetElapsedTimef();

	// still? compare the corners seen in both detections
	float motion = std::numeric_limits<float>::max();
	{
		std::map<int, cv::Point2f> previous;
		for (size_t i = 0; i < previousDetection.ids.size(); i++) previous[previousDetection.ids[i]] = previousDetection.corners[i];
		double sum = 0;
		int n = 0;
		for (size_t i = 0; i < detection.ids.size(); i++) {
			auto it = previous.find(detection.ids[i]);
			if (it == previous.end()) continue;
			sum += cv::norm(detection.corners[i] - it->second);
			n++;
		}
		if (n >= settings.minCorners) motion = float(sum / n);
	}
	const float tolerance = settings.stillPixels * imageSize.width / 1000.f;
	if (motion <= tolerance) {
		if (stillSince < 0) stillSince = now;
	} else {
		stillSince = -1;
	}

	const int n = int(detection.ids.size());
	if (n < settings.minCorners) {
		status = n == 0 ? "show the board to the camera" : ofToString(n) + " corners: show more of the board";
		return false;
	}
	if (!isStill()) {
		status = "hold the board still";
		return false;
	}
	if (!settings.autoCapture) {
		status = "still: press capture";
		return false;
	}
	if (now - lastSampleTime < settings.minInterval) return false;
	Sample s;
	if (!makeSample(detection, s)) {
		status = "corners are on one line: show more of the board";
		return false;
	}
	if (novelty(s) < settings.minNovelty) {
		// tell the user what is missing
		if (getTiltedFraction() < 0.35f) {
			status = "same view as before: TILT the board 30-45 degrees (left, right, up, down)";
		} else {
			// the region of the image with the most empty coverage cells
			const int cols = settings.coverageCols, rows = settings.coverageRows;
			std::vector<bool> hit(cols * rows, false);
			for (auto & smp : samples) {
				for (auto & p : smp.imagePoints) {
					const int cx = ofClamp(int(p.x / imageSize.width * cols), 0, cols - 1);
					const int cy = ofClamp(int(p.y / imageSize.height * rows), 0, rows - 1);
					hit[cy * cols + cx] = true;
				}
			}
			int empty[3][3] = {};
			for (int y = 0; y < rows; y++) {
				for (int x = 0; x < cols; x++) {
					if (!hit[y * cols + x]) empty[std::min(2, y * 3 / rows)][std::min(2, x * 3 / cols)]++;
				}
			}
			int bx = 1, by = 1, best = -1;
			for (int y = 0; y < 3; y++) {
				for (int x = 0; x < 3; x++) {
					if (empty[y][x] > best) {
						best = empty[y][x];
						bx = x;
						by = y;
					}
				}
			}
			static const char * vertical[] = { "top", "middle", "bottom" };
			static const char * horizontal[] = { "left", "center", "right" };
			const std::string where = (by == 1 && bx == 1) ? "center" : (by == 1 ? horizontal[bx] : bx == 1 ? vertical[by] : std::string(vertical[by]) + "-" + horizontal[bx]);
			status = best > 0 ? "same view as before: move the board to the " + where + " (it may stick out of the image)"
							  : "same view as before: change distance or tilt";
		}
		return false;
	}
	samples.push_back(std::move(s));
	lastSampleTime = now;
	stillSince = -1; // the next view needs its own still period
	status = "view " + ofToString(samples.size()) + " added, move the board";
	return true;
}

bool ofxArucoCalibrator::captureNow() {
	Sample s;
	if (int(detection.ids.size()) < settings.minCorners || !makeSample(detection, s)) {
		status = "can't capture: not enough corners";
		return false;
	}
	samples.push_back(std::move(s));
	lastSampleTime = ofGetElapsedTimef();
	status = "view " + ofToString(samples.size()) + " added";
	return true;
}

void ofxArucoCalibrator::removeLastSample() {
	if (!samples.empty()) samples.pop_back();
	status = "removed the last view (" + ofToString(samples.size()) + " left)";
}

bool ofxArucoCalibrator::makeSample(const Detection & d, Sample & s) const {
	if (d.ids.size() < 4 || imageSize.width <= 0) return false;
	const auto & charuco = board.getCvCharuco();
	if (charuco.checkCharucoCornersCollinear(d.ids)) return false;
	s.objectPoints.clear();
	s.imagePoints.clear();
	charuco.matchImagePoints(d.corners, d.ids, s.objectPoints, s.imagePoints);
	if (s.objectPoints.size() < 4) return false;

	// signature: where the board is, how big, how tilted
	const float W = float(imageSize.width), H = float(imageSize.height);
	cv::Point2f c(0, 0);
	for (auto & p : s.imagePoints) c += p;
	c *= 1.f / float(s.imagePoints.size());
	std::vector<cv::Point2f> hull;
	cv::convexHull(s.imagePoints, hull);
	const double area = cv::contourArea(hull);
	const glm::vec2 bs = board.getSize();
	std::vector<cv::Point2f> src, dst;
	for (size_t i = 0; i < s.objectPoints.size(); i++) {
		src.emplace_back(s.objectPoints[i].x / bs.x, s.objectPoints[i].y / bs.y);
		dst.emplace_back(s.imagePoints[i].x / W, s.imagePoints[i].y / H);
	}
	float tx = 0, ty = 0;
	cv::Mat h = cv::findHomography(src, dst, 0);
	if (!h.empty() && std::abs(h.at<double>(2, 2)) > 1e-12) {
		h /= h.at<double>(2, 2);
		tx = ofClamp(float(h.at<double>(2, 0)), -1.f, 1.f);
		ty = ofClamp(float(h.at<double>(2, 1)), -1.f, 1.f);
	}
	s.signature = { c.x / W, c.y / H, float(std::sqrt(area / (W * H))), tx, ty };
	return true;
}

float ofxArucoCalibrator::novelty(const Sample & s) const {
	float best = std::numeric_limits<float>::max();
	for (auto & o : samples) {
		const auto & a = s.signature;
		const auto & b = o.signature;
		const float d2 = (a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2])
			+ 0.5f * ((a[3] - b[3]) * (a[3] - b[3]) + (a[4] - b[4]) * (a[4] - b[4]));
		best = std::min(best, std::sqrt(d2));
	}
	return best;
}

float ofxArucoCalibrator::getCoverage() const {
	const int cols = settings.coverageCols, rows = settings.coverageRows;
	if (imageSize.width <= 0 || cols <= 0 || rows <= 0) return 0;
	std::vector<bool> hit(cols * rows, false);
	for (auto & s : samples) {
		for (auto & p : s.imagePoints) {
			const int cx = ofClamp(int(p.x / imageSize.width * cols), 0, cols - 1);
			const int cy = ofClamp(int(p.y / imageSize.height * rows), 0, rows - 1);
			hit[cy * cols + cx] = true;
		}
	}
	return float(std::count(hit.begin(), hit.end(), true)) / float(cols * rows);
}

float ofxArucoCalibrator::getTiltedFraction() const {
	if (samples.empty()) return 0;
	int tilted = 0;
	for (auto & s : samples) {
		if (std::abs(s.signature[3]) > 0.15f || std::abs(s.signature[4]) > 0.15f) tilted++;
	}
	return float(tilted) / float(samples.size());
}

//--------------------------------------------------------------
ofxArucoCalibrator::Result ofxArucoCalibrator::calibrate(int flags, bool removeOutliers) {
	Result r;
	if (samples.size() < 4) {
		r.message = "need at least 4 views";
		return r;
	}
	auto solve = [&](cv::Mat & K, cv::Mat & D) {
		std::vector<std::vector<cv::Point3f>> obj;
		std::vector<std::vector<cv::Point2f>> img;
		for (auto & s : samples) {
			obj.push_back(s.objectPoints);
			img.push_back(s.imagePoints);
		}
		std::vector<cv::Mat> rvecs, tvecs;
		cv::Mat sdIntrinsics, sdExtrinsics, perView;
		const double rms = cv::calibrateCamera(obj, img, imageSize, K, D, rvecs, tvecs, sdIntrinsics, sdExtrinsics, perView, flags);
		for (size_t i = 0; i < samples.size() && i < perView.total(); i++) samples[i].error = perView.at<double>(int(i));
		return rms;
	};
	try {
		cv::Mat K, D;
		double rms = solve(K, D);
		if (removeOutliers && samples.size() >= 10) {
			std::vector<double> errors;
			for (auto & s : samples) errors.push_back(s.error);
			std::vector<double> sorted = errors;
			std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
			const double threshold = std::max(3.0 * sorted[sorted.size() / 2], 1.0);
			const size_t maxRemove = samples.size() / 5;
			// worst first
			std::vector<size_t> order(samples.size());
			std::iota(order.begin(), order.end(), 0);
			std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return errors[a] > errors[b]; });
			std::vector<bool> drop(samples.size(), false);
			for (size_t k = 0; k < maxRemove && errors[order[k]] > threshold; k++) {
				drop[order[k]] = true;
				r.removedSamples++;
			}
			if (r.removedSamples > 0) {
				std::vector<Sample> kept;
				for (size_t i = 0; i < samples.size(); i++) {
					if (!drop[i]) kept.push_back(std::move(samples[i]));
				}
				samples = std::move(kept);
				K = cv::Mat();
				D = cv::Mat();
				rms = solve(K, D);
			}
		}
		r.intrinsics.setup(K, D, imageSize);
		r.rms = rms;
		r.numSamples = int(samples.size());
		r.ok = true;
		r.message = "rms " + ofToString(rms, 3) + " px, " + ofToString(r.numSamples) + " views"
			+ (r.removedSamples ? ", " + ofToString(r.removedSamples) + " outliers removed" : "");
	} catch (const cv::Exception & e) {
		r.message = std::string("calibration failed: ") + e.what();
	}
	return r;
}

ofxArucoCalibrator::Result ofxArucoCalibrator::refineFocal(const ofxArucoIntrinsics & guess) const {
	Result r;
	if (samples.size() < 3) {
		r.message = "need at least 3 views";
		return r;
	}
	if (!guess.isValid() || guess.getImageSize() != imageSize) {
		r.message = "the guess must be for " + ofToString(imageSize.width) + "x" + ofToString(imageSize.height);
		return r;
	}
	std::vector<std::vector<cv::Point3f>> obj;
	std::vector<std::vector<cv::Point2f>> img;
	for (auto & s : samples) {
		obj.push_back(s.objectPoints);
		img.push_back(s.imagePoints);
	}
	cv::Mat K = guess.getCameraMatrix().clone();
	cv::Mat D = guess.getDistCoeffs().empty() ? cv::Mat::zeros(1, 5, CV_64F) : guess.getDistCoeffs().clone();
	int flags = cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4
		| cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6 | cv::CALIB_FIX_TANGENT_DIST;
	if (D.total() >= 8) flags |= cv::CALIB_RATIONAL_MODEL;
	try {
		std::vector<cv::Mat> rvecs, tvecs;
		r.rms = cv::calibrateCamera(obj, img, imageSize, K, D, rvecs, tvecs, flags);
		r.intrinsics.setup(K, D, imageSize);
		r.numSamples = int(samples.size());
		r.ok = true;
		r.message = "rms " + ofToString(r.rms, 3) + " px";
	} catch (const cv::Exception & e) {
		r.message = std::string("refine failed: ") + e.what();
	}
	return r;
}

//--------------------------------------------------------------
void ofxArucoCalibrator::draw(const ofRectangle & vp) const {
	if (imageSize.width <= 0) return;
	const float sx = vp.width / imageSize.width, sy = vp.height / imageSize.height;
	ofPushStyle();

	// coverage
	const int cols = settings.coverageCols, rows = settings.coverageRows;
	std::vector<bool> hit(cols * rows, false);
	for (auto & s : samples) {
		for (auto & p : s.imagePoints) {
			const int cx = ofClamp(int(p.x / imageSize.width * cols), 0, cols - 1);
			const int cy = ofClamp(int(p.y / imageSize.height * rows), 0, rows - 1);
			hit[cy * cols + cx] = true;
		}
	}
	const float cw = vp.width / cols, ch = vp.height / rows;
	for (int y = 0; y < rows; y++) {
		for (int x = 0; x < cols; x++) {
			const ofRectangle cell(vp.x + x * cw, vp.y + y * ch, cw, ch);
			ofFill();
			ofSetColor(hit[y * cols + x] ? ofColor(0, 255, 0, 40) : ofColor(255, 0, 0, 25));
			ofDrawRectangle(cell);
			ofNoFill();
			ofSetColor(255, 255, 255, 40);
			ofDrawRectangle(cell);
		}
	}

	// accepted views
	ofFill();
	ofSetColor(0, 200, 255, 120);
	for (auto & s : samples) {
		for (auto & p : s.imagePoints) ofDrawCircle(vp.x + p.x * sx, vp.y + p.y * sy, 1.5f);
	}

	// current detection
	ofSetColor(isStill() ? ofColor(0, 255, 0) : ofColor(255, 200, 0));
	for (auto & p : detection.corners) ofDrawCircle(vp.x + p.x * sx, vp.y + p.y * sy, 3.f);
	ofPopStyle();
}

bool ofxArucoCalibrator::save(const std::string & path, const Result & result, const std::map<std::string, std::string> & info) {
	if (!result.intrinsics.isValid()) return false;
	const std::string fullPath = ofToDataPath(path, true);
	ofDirectory::createDirectory(ofFilePath::getEnclosingDirectory(fullPath), false, true);
	try {
		cv::FileStorage fs(fullPath, cv::FileStorage::WRITE);
		if (!fs.isOpened()) return false;
		fs << "calibration_time" << ofGetTimestampString("%Y-%m-%d %H:%M:%S");
		for (auto & kv : info) fs << kv.first << kv.second;
		fs << "image_width" << result.intrinsics.getWidth();
		fs << "image_height" << result.intrinsics.getHeight();
		fs << "camera_matrix" << result.intrinsics.getCameraMatrix();
		fs << "distortion_coefficients" << result.intrinsics.getDistCoeffs();
		fs << "avg_reprojection_error" << result.rms;
		fs << "nframes" << result.numSamples;
		return true;
	} catch (const cv::Exception & e) {
		ofLogError("ofxArucoCalibrator") << "could not save " << fullPath << ": " << e.what();
		return false;
	}
}

//--------------------------------------------------------------
std::string ofxArucoCalibrationPlan::toString(Status s) {
	switch (s) {
	case Status::Pending: return "pending";
	case Status::Calibrated: return "calibrated";
	case Status::Scaled: return "scaled";
	case Status::Failed: return "failed";
	case Status::Skipped: return "skipped";
	}
	return "";
}

std::string ofxArucoCalibrationPlan::aspectName(int w, int h) {
	if (w <= 0 || h <= 0) return "?";
	const float r = float(w) / h;
	static const std::pair<const char *, float> known[] = { { "4:3", 4.f / 3 }, { "16:9", 16.f / 9 }, { "16:10", 16.f / 10 },
		{ "3:2", 3.f / 2 }, { "5:4", 5.f / 4 }, { "1:1", 1.f }, { "21:9", 64.f / 27 }, { "11:9", 11.f / 9 } };
	for (auto & k : known) {
		if (std::abs(r - k.second) / k.second < 0.01f) return k.first;
	}
	return ofToString(r, 3) + ":1";
}

void ofxArucoCalibrationPlan::build(const std::vector<ofxArucoCameraMode> & input, const std::string & filePrefix) {
	tasks.clear();
	// unique sizes, highest fps kept
	std::vector<ofxArucoCameraMode> modes;
	for (auto & m : input) {
		if (m.width <= 0 || m.height <= 0) continue;
		auto it = std::find_if(modes.begin(), modes.end(), [&](const ofxArucoCameraMode & o) { return o.width == m.width && o.height == m.height; });
		if (it == modes.end()) {
			modes.push_back(m);
		} else {
			it->fps = std::max(it->fps, m.fps);
			it->fov = std::max(it->fov, m.fov);
		}
	}
	std::sort(modes.begin(), modes.end(), [](const ofxArucoCameraMode & a, const ofxArucoCameraMode & b) {
		return a.width * a.height > b.width * b.height;
	});
	// groups by aspect ratio (and field of view when known), in order of their largest mode
	std::vector<float> groupRatios, groupFovs;
	std::vector<std::vector<ofxArucoCameraMode>> groups;
	for (auto & m : modes) {
		const float r = float(m.width) / m.height;
		size_t g = 0;
		for (; g < groupRatios.size(); g++) {
			const bool sameRatio = std::abs(r - groupRatios[g]) / groupRatios[g] < 0.01f;
			const bool sameFov = m.fov <= 0 || groupFovs[g] <= 0 || std::abs(m.fov - groupFovs[g]) / groupFovs[g] < 0.01f;
			if (sameRatio && sameFov) break;
		}
		if (g == groupRatios.size()) {
			groupRatios.push_back(r);
			groupFovs.push_back(m.fov);
			groups.emplace_back();
		}
		groups[g].push_back(m);
	}
	for (size_t g = 0; g < groups.size(); g++) {
		for (size_t i = 0; i < groups[g].size(); i++) {
			Task t;
			t.mode = groups[g][i];
			t.group = int(g);
			t.kind = i == 0 ? Kind::Full : Kind::Verify;
			t.fileName = filePrefix + t.mode.getName() + ".yml";
			tasks.push_back(t);
		}
	}
}

void ofxArucoCalibrationPlan::resumeFrom(const std::string & folder) {
	for (auto & t : tasks) {
		const std::string path = ofFilePath::join(ofToDataPath(folder, true), t.getFileName());
		if (!ofFile::doesFileExist(path)) continue;
		try {
			cv::FileStorage fs(path, cv::FileStorage::READ);
			if (!fs.isOpened()) continue;
			std::string method;
			if (!fs["calibration_method"].empty()) fs["calibration_method"] >> method;
			if (!fs["avg_reprojection_error"].empty()) t.rms = (double)fs["avg_reprojection_error"];
			if (!fs["nframes"].empty()) t.samples = (int)fs["nframes"];
			t.status = method == "scaled" ? Status::Scaled : Status::Calibrated;
			if (t.status == Status::Calibrated) t.kind = Kind::Full;
			t.note = "from a previous session";
		} catch (const cv::Exception &) {
		}
	}
}

const ofxArucoCalibrationPlan::Task * ofxArucoCalibrationPlan::getReference(const Task & task) const {
	for (auto & t : tasks) {
		if (t.group == task.group && t.status == Status::Calibrated && &t != &task) return &t;
	}
	return nullptr;
}

bool ofxArucoCalibrationPlan::getScaledReference(const Task & task, const std::string & folder, ofxArucoIntrinsics & out) const {
	const Task * ref = getReference(task);
	if (!ref) return false;
	ofxArucoIntrinsics intrinsics;
	if (!intrinsics.load(ofFilePath::join(folder, ref->getFileName()))) return false;
	out = intrinsics.getScaled(task.mode.width, task.mode.height);
	return true;
}

bool ofxArucoCalibrationPlan::saveIndex(const std::string & folder, const std::map<std::string, std::string> & info) const {
	const std::string dir = ofToDataPath(folder, true);
	ofDirectory::createDirectory(dir, false, true);
	try {
		cv::FileStorage fs(ofFilePath::join(dir, "index.yml"), cv::FileStorage::WRITE);
		if (!fs.isOpened()) return false;
		fs << "updated" << ofGetTimestampString("%Y-%m-%d %H:%M:%S");
		for (auto & kv : info) fs << kv.first << kv.second;
		fs << "modes" << "[";
		for (auto & t : tasks) {
			fs << "{";
			fs << "file" << t.getFileName();
			fs << "width" << t.mode.width << "height" << t.mode.height << "fps" << t.mode.fps;
			if (t.mode.fov > 0) fs << "fov" << t.mode.fov;
			fs << "aspect" << aspectName(t.mode.width, t.mode.height);
			fs << "kind" << toString(t.kind) << "status" << toString(t.status);
			fs << "rms" << t.rms << "views" << t.samples;
			fs << "note" << t.note;
			fs << "}";
		}
		fs << "]";
		return true;
	} catch (const cv::Exception & e) {
		ofLogError("ofxArucoCalibrationPlan") << "could not save the index: " << e.what();
		return false;
	}
}
