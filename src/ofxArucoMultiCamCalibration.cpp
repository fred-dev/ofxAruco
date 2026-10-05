#include "ofxArucoMultiCamCalibration.h"
#include "ofxAruco.h"
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <numeric>
#include <queue>

using namespace ofxArucoUtils;

namespace {
	float translationDistanceMm(const glm::mat4 & a, const glm::mat4 & b) {
		return glm::distance(glm::vec3(a[3]), glm::vec3(b[3])) * 1000.f;
	}

	float median(std::vector<float> v) {
		if (v.empty()) return 0;
		std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
		return v[v.size() / 2];
	}

	// Average of rigid transforms: quaternion mean (sign aligned) + translation mean.
	glm::mat4 averageTransforms(const std::vector<glm::mat4> & ts) {
		glm::quat q0 = glm::quat_cast(glm::mat3(ts[0]));
		glm::vec4 qsum(0);
		glm::vec3 tsum(0);
		for (auto & t : ts) {
			glm::quat q = glm::quat_cast(glm::mat3(t));
			if (glm::dot(q, q0) < 0) q = -q;
			qsum += glm::vec4(q.x, q.y, q.z, q.w);
			tsum += glm::vec3(t[3]);
		}
		glm::quat q = glm::normalize(glm::quat(qsum.w, qsum.x, qsum.y, qsum.z));
		glm::mat4 m = glm::mat4_cast(q);
		m[3] = glm::vec4(tsum / float(ts.size()), 1.f);
		return m;
	}
}

ofxArucoMultiCamCalibration::ofxArucoMultiCamCalibration() {
	parameters.setName("multicam calibration");
	parameters.add(autoCapture.set("auto capture", true));
	parameters.add(stillTime.set("still time (s)", 0.6f, 0.f, 5.f));
	parameters.add(stillTranslationMm.set("still max move (mm)", 3.f, 0.1f, 50.f));
	parameters.add(stillRotationDeg.set("still max rot (deg)", 0.5f, 0.05f, 10.f));
	parameters.add(minMoveMm.set("min move between (mm)", 50.f, 0.f, 1000.f));
	parameters.add(minMoveDeg.set("min rot between (deg)", 10.f, 0.f, 90.f));
	parameters.add(maxReprojectionError.set("max reproj error (px)", 2.f, 0.1f, 20.f));
	parameters.add(minPoints.set("min board points", 8, 4, 200));
	parameters.add(bundleAdjust.set("bundle adjustment", true));
}

void ofxArucoMultiCamCalibration::setup(int numCameras, int reference) {
	cameras.assign(numCameras, CameraResult());
	for (int i = 0; i < numCameras; i++) cameras[i].name = "camera " + ofToString(i);
	referenceCamera = ofClamp(reference, 0, numCameras - 1);
	tracks.assign(numCameras, CamTrack());
	samples.clear();
	lastCaptured.clear();
	referenceToWorld = glm::mat4(1.f);
	solve();
}

void ofxArucoMultiCamCalibration::setCameraName(int camera, const std::string & name) {
	if (camera >= 0 && camera < (int)cameras.size()) cameras[camera].name = name;
}

std::vector<ofxArucoMultiCamCalibration::Observation> ofxArucoMultiCamCalibration::collect(
	const std::vector<const ofxAruco *> & detectors, int boardIndex) const {
	std::vector<Observation> obs(cameras.size());
	for (size_t i = 0; i < detectors.size() && i < obs.size(); i++) {
		if (!detectors[i]) continue;
		const auto & bp = detectors[i]->getBoardPose(boardIndex);
		obs[i].valid = bp.found;
		obs[i].boardToCamera = bp.pose;
		obs[i].reprojectionError = bp.reprojectionError;
		obs[i].numPoints = bp.numPoints;
		if (bp.found) {
			obs[i].objectPoints = bp.objectPoints;
			obs[i].imagePoints = bp.imagePoints;
			const auto & size = detectors[i]->getResult().imageSize;
			const auto & intr = detectors[i]->getIntrinsics();
			obs[i].intrinsics = (intr.isValid() && intr.getImageSize() != cv::Size(size.x, size.y))
				? intr.getScaled(size.x, size.y)
				: intr;
		}
	}
	return obs;
}

bool ofxArucoMultiCamCalibration::addSample(const std::vector<Observation> & in) {
	if (in.size() != cameras.size()) {
		ofLogError("ofxArucoMultiCamCalibration") << "addSample: expected " << cameras.size() << " observations, got " << in.size();
		return false;
	}
	std::vector<Observation> obs = in;
	int valid = 0;
	for (auto & o : obs) {
		if (o.valid && (o.reprojectionError > maxReprojectionError || o.numPoints < minPoints)) o.valid = false;
		if (o.valid) valid++;
	}
	if (valid < 2) {
		ofLogNotice("ofxArucoMultiCamCalibration") << "sample skipped: the board must be seen (with good quality) by at least 2 cameras";
		return false;
	}
	samples.push_back({ obs });
	lastCaptured = obs;
	solve();
	ofLogNotice("ofxArucoMultiCamCalibration") << "sample " << samples.size() << " added (" << valid << " cameras)";
	return true;
}

bool ofxArucoMultiCamCalibration::addSample(const std::vector<const ofxAruco *> & detectors, int boardIndex) {
	return addSample(collect(detectors, boardIndex));
}

bool ofxArucoMultiCamCalibration::update(const std::vector<const ofxAruco *> & detectors, int boardIndex) {
	if (tracks.size() != cameras.size()) tracks.assign(cameras.size(), CamTrack());
	const float now = ofGetElapsedTimef();
	auto obs = collect(detectors, boardIndex);

	int seen = 0;
	bool allStill = true;
	float minStill = std::numeric_limits<float>::max();
	for (size_t i = 0; i < obs.size(); i++) {
		auto & t = tracks[i];
		const auto & o = obs[i];
		const bool good = o.valid && o.reprojectionError <= maxReprojectionError && o.numPoints >= minPoints;
		if (!good) {
			obs[i].valid = false;
			t.seen = false;
			t.stillSince = -1;
			continue;
		}
		seen++;
		const uint64_t frame = detectors[i] ? detectors[i]->getResult().frameNumber : 0;
		if (frame != t.lastFrame) {
			const bool moved = !t.seen || translationDistanceMm(t.last, o.boardToCamera) > stillTranslationMm
				|| rotationAngleDeg(t.last, o.boardToCamera) > stillRotationDeg;
			if (moved || t.stillSince < 0) t.stillSince = now;
			t.last = o.boardToCamera;
			t.lastFrame = frame;
			t.seen = true;
		}
		minStill = std::min(minStill, now - t.stillSince);
	}
	if (seen < 2) allStill = false;
	stillProgress = (seen >= 2 && stillTime > 0) ? ofClamp(minStill / stillTime, 0, 1) : 0;
	if (seen >= 2 && minStill < stillTime) allStill = false;

	if (!autoCapture || !allStill) return false;

	// require motion since the last sample, so samples are diverse
	if (!lastCaptured.empty()) {
		bool movedEnough = false;
		bool comparable = false;
		for (size_t i = 0; i < obs.size(); i++) {
			if (!obs[i].valid || !lastCaptured[i].valid) continue;
			comparable = true;
			if (translationDistanceMm(lastCaptured[i].boardToCamera, obs[i].boardToCamera) > minMoveMm
				|| rotationAngleDeg(lastCaptured[i].boardToCamera, obs[i].boardToCamera) > minMoveDeg) {
				movedEnough = true;
			}
		}
		if (comparable && !movedEnough) return false;
	}
	return addSample(obs);
}

void ofxArucoMultiCamCalibration::clear() {
	samples.clear();
	lastCaptured.clear();
	solve();
}

void ofxArucoMultiCamCalibration::removeLastSample() {
	if (samples.empty()) return;
	samples.pop_back();
	lastCaptured = samples.empty() ? std::vector<Observation>() : samples.back().observations;
	solve();
}

ofxArucoMultiCamCalibration::PairEstimate ofxArucoMultiCamCalibration::estimatePair(int i, int j) const {
	PairEstimate e;
	std::vector<glm::mat4> rel;
	for (auto & s : samples) {
		const auto & oi = s.observations[i];
		const auto & oj = s.observations[j];
		if (oi.valid && oj.valid) {
			// p_i = B_i * B_j^-1 * p_j
			rel.push_back(oi.boardToCamera * rigidInverse(oj.boardToCamera));
		}
	}
	if (rel.empty()) return e;

	glm::mat4 mean = averageTransforms(rel);

	// outlier rejection: drop samples far from the mean (relative to the median spread)
	if (rel.size() >= 4) {
		std::vector<float> dr, dt;
		for (auto & r : rel) {
			dr.push_back(rotationAngleDeg(r, mean));
			dt.push_back(translationDistanceMm(r, mean));
		}
		const float rThresh = std::max(3.f * median(dr), 0.5f);
		const float tThresh = std::max(3.f * median(dt), 5.f);
		std::vector<glm::mat4> inliers;
		for (size_t k = 0; k < rel.size(); k++) {
			if (dr[k] <= rThresh && dt[k] <= tThresh) inliers.push_back(rel[k]);
		}
		if (inliers.size() >= 2 && inliers.size() < rel.size()) {
			rel = inliers;
			mean = averageTransforms(rel);
		}
	}

	double sr = 0, st = 0;
	for (auto & r : rel) {
		sr += std::pow(rotationAngleDeg(r, mean), 2);
		st += std::pow(translationDistanceMm(r, mean), 2);
	}
	e.valid = true;
	e.count = int(rel.size());
	e.jToI = mean;
	e.rotationStdDeg = float(std::sqrt(sr / rel.size()));
	e.translationStdMm = float(std::sqrt(st / rel.size()));
	return e;
}

bool ofxArucoMultiCamCalibration::solve() {
	const int n = (int)cameras.size();
	for (auto & c : cameras) {
		c.solved = false;
		c.samples = 0;
		c.linkedTo = -1;
		c.rotationStdDeg = c.translationStdMm = 0;
		c.refined = false;
		c.reprojectionError = 0;
		c.cameraToReference = glm::mat4(1.f);
	}
	if (n == 0) return false;
	cameras[referenceCamera].solved = true;

	// pairwise estimates
	std::vector<std::vector<PairEstimate>> pairs(n, std::vector<PairEstimate>(n));
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			if (i != j) pairs[i][j] = estimatePair(i, j);
		}
	}

	// Dijkstra from the reference, preferring links with many samples
	std::vector<float> cost(n, std::numeric_limits<float>::max());
	using Item = std::pair<float, int>;
	std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
	cost[referenceCamera] = 0;
	queue.push({ 0.f, referenceCamera });
	while (!queue.empty()) {
		auto [c, p] = queue.top();
		queue.pop();
		if (c > cost[p]) continue;
		for (int k = 0; k < n; k++) {
			const auto & e = pairs[p][k]; // k -> p
			if (k == p || !e.valid) continue;
			const float nc = c + 1.f / std::sqrt(float(e.count));
			if (nc < cost[k]) {
				cost[k] = nc;
				auto & cam = cameras[k];
				cam.solved = true;
				cam.linkedTo = p;
				cam.samples = e.count;
				cam.rotationStdDeg = e.rotationStdDeg;
				cam.translationStdMm = e.translationStdMm;
				queue.push({ nc, k });
			}
		}
	}
	// compose along the tree (parents always have a lower cost)
	std::vector<int> order(n);
	std::iota(order.begin(), order.end(), 0);
	std::sort(order.begin(), order.end(), [&](int a, int b) { return cost[a] < cost[b]; });
	for (int k : order) {
		auto & cam = cameras[k];
		if (k == referenceCamera || !cam.solved) continue;
		cam.cameraToReference = cameras[cam.linkedTo].cameraToReference * pairs[cam.linkedTo][k].jToI;
	}
	if (bundleAdjust) refine();
	return isSolved();
}

// ---------------------------------------------------------------------------
// Bundle adjustment: unknowns are referenceToCamera for every solved camera
// (except the reference, fixed at identity) and boardToReference for every
// sample. Residuals are the reprojection errors of every board corner in every
// camera. Solved with Levenberg-Marquardt; the normal equations are accumulated
// per observation (each one only touches 1 camera + 1 board pose), so it stays
// fast with many samples.
namespace {
	struct Pose6 {
		cv::Vec3d r, t;
	};

	void compose(const Pose6 & a, const Pose6 & b, cv::Vec3d & r, cv::Vec3d & t) {
		// a * b
		cv::Matx33d Ra, Rb;
		cv::Rodrigues(a.r, Ra);
		cv::Rodrigues(b.r, Rb);
		cv::Rodrigues(Ra * Rb, r);
		t = Ra * b.t + a.t;
	}

	struct BAObservation {
		int cam, sample;
		std::vector<cv::Point3d> obj;
		std::vector<cv::Point2d> img;
		cv::Mat K, D;
	};
}

void ofxArucoMultiCamCalibration::refine() {
	const int n = (int)cameras.size();
	if (samples.empty()) return;

	// --- gather observations of solved cameras that have points + intrinsics
	std::vector<BAObservation> obs;
	std::vector<int> sampleUsed(samples.size(), 0);
	for (size_t s = 0; s < samples.size(); s++) {
		for (int k = 0; k < n; k++) {
			const auto & o = samples[s].observations[k];
			if (!o.valid || !cameras[k].solved) continue;
			if (o.objectPoints.empty() || o.objectPoints.size() != o.imagePoints.size() || !o.intrinsics.isValid()) {
				return; // not enough data for refinement: keep the averaged result
			}
			BAObservation b;
			b.cam = k;
			b.sample = int(s);
			for (auto & p : o.objectPoints) b.obj.emplace_back(p.x, p.y, p.z);
			for (auto & p : o.imagePoints) b.img.emplace_back(p.x, p.y);
			b.K = o.intrinsics.getCameraMatrix();
			b.D = o.intrinsics.getDistCoeffs();
			obs.push_back(std::move(b));
			sampleUsed[s]++;
		}
	}

	// --- parameter layout
	std::vector<int> camOffset(n, -1), sampleOffset(samples.size(), -1);
	int N = 0;
	for (int k = 0; k < n; k++) {
		if (k != referenceCamera && cameras[k].solved) {
			camOffset[k] = N;
			N += 6;
		}
	}
	for (size_t s = 0; s < samples.size(); s++) {
		if (sampleUsed[s] > 0) {
			sampleOffset[s] = N;
			N += 6;
		}
	}
	if (N == 0) return;

	std::vector<double> p(N, 0.0);
	auto setPose = [&](int off, const glm::mat4 & m) {
		cv::Vec3d r, t;
		fromMat4(m, r, t);
		for (int i = 0; i < 3; i++) {
			p[off + i] = r[i];
			p[off + 3 + i] = t[i];
		}
	};
	auto getPose = [](const std::vector<double> & v, int off) {
		Pose6 q;
		if (off < 0) return q; // identity
		for (int i = 0; i < 3; i++) {
			q.r[i] = v[off + i];
			q.t[i] = v[off + 3 + i];
		}
		return q;
	};
	for (int k = 0; k < n; k++) {
		if (camOffset[k] >= 0) setPose(camOffset[k], rigidInverse(cameras[k].cameraToReference));
	}
	for (size_t s = 0; s < samples.size(); s++) {
		if (sampleOffset[s] < 0) continue;
		for (int k = 0; k < n; k++) { // board in the reference frame, from the first camera that saw it
			const auto & o = samples[s].observations[k];
			if (o.valid && cameras[k].solved) {
				setPose(sampleOffset[s], cameras[k].cameraToReference * o.boardToCamera);
				break;
			}
		}
	}

	auto residual = [&](const BAObservation & o, const std::vector<double> & v, std::vector<double> & out) {
		cv::Vec3d r, t;
		compose(getPose(v, camOffset[o.cam]), getPose(v, sampleOffset[o.sample]), r, t);
		std::vector<cv::Point2d> proj;
		cv::projectPoints(o.obj, r, t, o.K, o.D, proj);
		out.resize(proj.size() * 2);
		for (size_t i = 0; i < proj.size(); i++) {
			out[2 * i] = proj[i].x - o.img[i].x;
			out[2 * i + 1] = proj[i].y - o.img[i].y;
		}
	};
	std::vector<char> active(obs.size(), 1);
	auto cost = [&](const std::vector<double> & v) {
		double c = 0;
		std::vector<double> r;
		for (size_t i = 0; i < obs.size(); i++) {
			if (!active[i]) continue;
			residual(obs[i], v, r);
			for (double e : r) c += e * e;
		}
		return c;
	};

	auto runLM = [&](int maxIters) {
		double lambda = 1e-3;
		double c = cost(p);
		std::vector<double> r0, r1;
		for (int iter = 0; iter < maxIters; iter++) {
			cv::Mat A = cv::Mat::zeros(N, N, CV_64F);
			cv::Mat g = cv::Mat::zeros(N, 1, CV_64F);
			for (size_t i = 0; i < obs.size(); i++) {
				if (!active[i]) continue;
				const auto & o = obs[i];
				residual(o, p, r0);
				std::vector<int> idx;
				for (int off : { camOffset[o.cam], sampleOffset[o.sample] }) {
					if (off >= 0) {
						for (int j = 0; j < 6; j++) idx.push_back(off + j);
					}
				}
				const int m = (int)idx.size();
				cv::Mat J((int)r0.size(), m, CV_64F);
				for (int j = 0; j < m; j++) {
					const double h = 1e-6;
					const double keep = p[idx[j]];
					p[idx[j]] = keep + h;
					residual(o, p, r1);
					p[idx[j]] = keep;
					for (int e = 0; e < J.rows; e++) J.at<double>(e, j) = (r1[e] - r0[e]) / h;
				}
				cv::Mat JtJ = J.t() * J;
				cv::Mat Jtr = J.t() * cv::Mat(r0);
				for (int a = 0; a < m; a++) {
					g.at<double>(idx[a]) += Jtr.at<double>(a);
					for (int b = 0; b < m; b++) A.at<double>(idx[a], idx[b]) += JtJ.at<double>(a, b);
				}
			}
			bool improved = false;
			for (int tries = 0; tries < 10 && !improved; tries++) {
				cv::Mat Ad = A.clone();
				for (int d = 0; d < N; d++) Ad.at<double>(d, d) += lambda * (A.at<double>(d, d) + 1e-9);
				cv::Mat dp;
				if (!cv::solve(Ad, -g, dp, cv::DECOMP_CHOLESKY)) cv::solve(Ad, -g, dp, cv::DECOMP_SVD);
				std::vector<double> pn = p;
				for (int d = 0; d < N; d++) pn[d] += dp.at<double>(d);
				const double cn = cost(pn);
				if (cn < c) {
					const double gain = (c - cn) / std::max(c, 1e-12);
					p = pn;
					c = cn;
					lambda = std::max(lambda * 0.3, 1e-9);
					improved = true;
					if (gain < 1e-10) return;
				} else {
					lambda *= 10;
				}
			}
			if (!improved) return;
		}
	};

	runLM(50);

	// drop gross outliers (bad detections) and run again
	std::vector<double> r;
	std::vector<float> rms(obs.size(), 0.f);
	for (size_t i = 0; i < obs.size(); i++) {
		residual(obs[i], p, r);
		double s = 0;
		for (double e : r) s += e * e;
		rms[i] = float(std::sqrt(s / (r.size() / 2)));
	}
	const float limit = std::max(3.f * median(rms), 1.f);
	bool dropped = false;
	for (size_t i = 0; i < obs.size(); i++) {
		if (rms[i] > limit) {
			active[i] = 0;
			dropped = true;
		}
	}
	if (dropped) runLM(50);

	// --- results
	std::vector<double> sum(n, 0.0);
	std::vector<int> count(n, 0);
	for (size_t i = 0; i < obs.size(); i++) {
		if (!active[i]) continue;
		residual(obs[i], p, r);
		for (double e : r) sum[obs[i].cam] += e * e;
		count[obs[i].cam] += int(r.size() / 2);
	}
	for (int k = 0; k < n; k++) {
		if (!cameras[k].solved) continue;
		if (camOffset[k] >= 0) {
			const Pose6 q = getPose(p, camOffset[k]);
			cameras[k].cameraToReference = rigidInverse(toMat4(q.r, q.t));
		}
		cameras[k].refined = count[k] > 0;
		cameras[k].reprojectionError = count[k] > 0 ? float(std::sqrt(sum[k] / count[k])) : 0.f;
	}
}

bool ofxArucoMultiCamCalibration::isSolved() const {
	if (cameras.size() < 2) return false;
	for (auto & c : cameras) {
		if (!c.solved) return false;
	}
	return true;
}

bool ofxArucoMultiCamCalibration::isSolved(int camera) const {
	return camera >= 0 && camera < (int)cameras.size() && cameras[camera].solved;
}

glm::mat4 ofxArucoMultiCamCalibration::getCameraToReference(int camera) const {
	return isSolved(camera) ? cameras[camera].cameraToReference : glm::mat4(1.f);
}

glm::mat4 ofxArucoMultiCamCalibration::getCameraToWorld(int camera) const {
	return referenceToWorld * getCameraToReference(camera);
}

glm::mat4 ofxArucoMultiCamCalibration::getCameraToCamera(int from, int to) const {
	return rigidInverse(getCameraToReference(to)) * getCameraToReference(from);
}

void ofxArucoMultiCamCalibration::setWorldToReferenceCamera() {
	referenceToWorld = glm::mat4(1.f);
}

bool ofxArucoMultiCamCalibration::setWorldFromBoard(int camera, const glm::mat4 & boardToCamera) {
	if (!isSolved(camera)) {
		ofLogError("ofxArucoMultiCamCalibration") << "setWorldFromBoard: camera " << camera << " is not calibrated yet";
		return false;
	}
	const glm::mat4 boardToReference = getCameraToReference(camera) * boardToCamera;
	referenceToWorld = rigidInverse(boardToReference);
	return true;
}

ofJson ofxArucoMultiCamCalibration::toJson() const {
	ofJson j;
	j["about"] = "ofxAruco multi camera calibration. p_world = cameraToWorld * p_camera. "
				 "Camera frame = OpenCV (x right, y down, z forward). Units: meters (the board units). "
				 "Matrices are row-major (4 rows of 4).";
	j["referenceCamera"] = referenceCamera;
	j["numSamples"] = samples.size();
	j["referenceToWorld"] = ofxArucoUtils::toJson(referenceToWorld);
	ofJson cams = ofJson::array();
	for (int i = 0; i < (int)cameras.size(); i++) {
		const auto & c = cameras[i];
		cams.push_back({
			{ "index", i },
			{ "name", c.name },
			{ "solved", c.solved },
			{ "linkedTo", c.linkedTo },
			{ "samples", c.samples },
			{ "rotationStdDeg", c.rotationStdDeg },
			{ "translationStdMm", c.translationStdMm },
			{ "refined", c.refined },
			{ "reprojectionErrorPx", c.reprojectionError },
			{ "cameraToReference", ofxArucoUtils::toJson(c.cameraToReference) },
			{ "cameraToWorld", ofxArucoUtils::toJson(getCameraToWorld(i)) },
		});
	}
	j["cameras"] = cams;
	return j;
}

bool ofxArucoMultiCamCalibration::fromJson(const ofJson & j) {
	try {
		const auto & cams = j.at("cameras");
		cameras.assign(cams.size(), CameraResult());
		for (size_t i = 0; i < cams.size(); i++) {
			auto & c = cameras[i];
			const auto & cj = cams[i];
			c.name = cj.value("name", "camera " + ofToString(i));
			c.solved = cj.value("solved", false);
			c.linkedTo = cj.value("linkedTo", -1);
			c.samples = cj.value("samples", 0);
			c.rotationStdDeg = cj.value("rotationStdDeg", 0.f);
			c.translationStdMm = cj.value("translationStdMm", 0.f);
			c.refined = cj.value("refined", false);
			c.reprojectionError = cj.value("reprojectionErrorPx", 0.f);
			c.cameraToReference = mat4FromJson(cj.at("cameraToReference"));
		}
		referenceCamera = j.value("referenceCamera", 0);
		referenceToWorld = j.contains("referenceToWorld") ? mat4FromJson(j["referenceToWorld"]) : glm::mat4(1.f);
		tracks.assign(cameras.size(), CamTrack());
		samples.clear(); // the raw samples are not stored, only the result
		lastCaptured.clear();
		return true;
	} catch (const std::exception & e) {
		ofLogError("ofxArucoMultiCamCalibration") << "invalid json: " << e.what();
		return false;
	}
}

bool ofxArucoMultiCamCalibration::save(const std::string & path) const {
	return ofxArucoUtils::saveJson(path, toJson());
}

bool ofxArucoMultiCamCalibration::load(const std::string & path) {
	const std::string fullPath = ofToDataPath(path, true);
	if (!ofFile::doesFileExist(fullPath)) return false;
	return fromJson(ofLoadJson(fullPath));
}
