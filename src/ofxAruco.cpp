#include "ofxAruco.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

#include <algorithm>
#include <functional>
#include <unordered_map>

using namespace ofxArucoUtils;

// Immutable snapshot of everything the worker needs. A new one is created on the
// main thread whenever a parameter, the intrinsics or the boards change, so the
// worker never reads ofParameters directly.
struct ofxAruco::Config {
	int dictionary = cv::aruco::DICT_5X5_100;
	cv::aruco::DetectorParameters detectorParams;
	cv::aruco::RefineParameters refineParams;
	ofxArucoIntrinsics intrinsics;
	std::vector<ofxArucoBoard> boards;
	std::vector<int> boardPass;        // per board: -1 found by the main detector, k by extraDictionaries[k]
	std::vector<int> extraDictionaries; // boards in other dictionaries: one more OpenCV pass each
	std::vector<int> anyFamilies;       // "any dictionary": families looked up in the extra pass
	std::map<std::pair<int, int>, float> markerLengths; // (family, id) -> meters: set by the app, then from boards
	float markerLength = 0;
	bool estimateMarkerPoses = true;
	bool estimateBoardPoses = true;
	bool refineWithBoards = true;
};

namespace {
	float rmsError(const std::vector<cv::Point3f> & obj, const std::vector<cv::Point2f> & img,
		const cv::Vec3d & rvec, const cv::Vec3d & tvec, const cv::Mat & K, const cv::Mat & D) {
		if (obj.empty()) return 0;
		std::vector<cv::Point2f> proj;
		cv::projectPoints(obj, rvec, tvec, K, D, proj);
		double sum = 0;
		for (size_t i = 0; i < img.size(); i++) {
			const cv::Point2f d = proj[i] - img[i];
			sum += d.dot(d);
		}
		return float(std::sqrt(sum / img.size()));
	}

	// A square seen from the front has two possible poses (mirror tilts). IPPE gives
	// both: keep the one that faces the camera with the lowest reprojection error.
	// Falls back to the iterative solver for degenerate cases (e.g. a perfectly
	// frontal marker at the principal point).
	void estimateMarkerPose(const std::vector<cv::Point3f> & obj, const std::vector<cv::Point2f> & img,
		const cv::Mat & K, const cv::Mat & D, ofxArucoMarker & m) {
		std::vector<cv::Vec3d> rvecs, tvecs;
		cv::Mat errs;
		try {
			cv::solvePnPGeneric(obj, img, K, D, rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE,
				cv::noArray(), cv::noArray(), errs);
		} catch (const cv::Exception &) {
			rvecs.clear();
		}
		float best = std::numeric_limits<float>::max();
		for (size_t k = 0; k < rvecs.size(); k++) {
			const glm::mat4 pose = ofxArucoUtils::toMat4(rvecs[k], tvecs[k]);
			// marker z must point back towards the camera
			if (glm::dot(glm::vec3(pose[2]), glm::vec3(pose[3])) >= 0) continue;
			const float e = rmsError(obj, img, rvecs[k], tvecs[k], K, D);
			if (e < best) {
				best = e;
				m.rvec = rvecs[k];
				m.tvec = tvecs[k];
			}
		}
		if (best > 1.f) {
			cv::Vec3d r = m.rvec, t = m.tvec;
			if (best == std::numeric_limits<float>::max()) {
				r = cv::Vec3d(CV_PI, 0, 0); // facing the camera
				t = cv::Vec3d(0, 0, 1);
			}
			if (cv::solvePnP(obj, img, K, D, r, t, true, cv::SOLVEPNP_ITERATIVE)) {
				const float e = rmsError(obj, img, r, t, K, D);
				if (e < best) {
					best = e;
					m.rvec = r;
					m.tvec = t;
				}
			}
		}
		if (best == std::numeric_limits<float>::max()) return;
		m.hasPose = true;
		m.pose = ofxArucoUtils::toMat4(m.rvec, m.tvec);
		m.reprojectionError = best;
	}

	// --- "any dictionary" -------------------------------------------------------
	// Finds marker shapes once (like OpenCV: adaptive thresholds, contours, convex
	// quads), reads their bits for every marker size and looks the codes up in hash
	// tables of every dictionary: one pass for all dictionaries, exact codes only.

	uint64_t toCode(const cv::Mat & bits) {
		uint64_t code = 0;
		for (int y = 0; y < bits.rows; y++) {
			for (int x = 0; x < bits.cols; x++) code = (code << 1) | (bits.at<uchar>(y, x) ? 1 : 0);
		}
		return code;
	}

	struct CodeTable {
		int family = -1;
		int markerSize = 0;
		std::unordered_map<uint64_t, std::pair<int, int>> codes; // code -> id, clockwise quarter turns
	};

	// Built once per family and kept (a few thousand codes x 4 rotations).
	const CodeTable & getCodeTable(int family) {
		static std::mutex mutex;
		static std::map<int, CodeTable> tables;
		std::lock_guard<std::mutex> lock(mutex);
		auto it = tables.find(family);
		if (it != tables.end()) return it->second;
		CodeTable & t = tables[family];
		const cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(family);
		t.family = family;
		t.markerSize = dict.markerSize;
		cv::Mat bits, turned;
		for (int id = 0; id < dict.bytesList.rows; id++) {
			bits = cv::aruco::Dictionary::getBitsFromByteList(dict.bytesList.rowRange(id, id + 1), dict.markerSize);
			for (int r = 0; r < 4; r++) {
				t.codes.emplace(toCode(bits), std::make_pair(id, r));
				cv::rotate(bits, turned, cv::ROTATE_90_CLOCKWISE);
				bits = turned.clone();
			}
		}
		return t;
	}

	cv::Point2f centreOf(const std::vector<cv::Point2f> & q) {
		return (q[0] + q[1] + q[2] + q[3]) * 0.25f;
	}

	bool insideAny(const std::vector<std::vector<cv::Point2f>> & quads, const cv::Point2f & p) {
		for (auto & q : quads) {
			if (cv::pointPolygonTest(q, p, false) >= 0) return true;
		}
		return false;
	}

	struct AnyMarker {
		int dictionary = -1;
		int id = -1;
		int markerSize = 0;
		std::vector<cv::Point2f> corners;
	};

	std::vector<AnyMarker> scanAnyDictionary(const cv::Mat & gray, const std::vector<int> & families,
		const cv::aruco::DetectorParameters & p) {
		std::vector<AnyMarker> found;
		if (families.empty() || gray.empty()) return found;

		std::vector<const CodeTable *> tables;
		std::vector<int> sizes;
		for (int f : families) {
			tables.push_back(&getCodeTable(f));
			sizes.push_back(tables.back()->markerSize);
		}
		// largest grids first: reading a marker with fewer cells than it has gives
		// noise, not a code, while the other way round can match by chance
		std::sort(sizes.begin(), sizes.end(), std::greater<int>());
		sizes.erase(std::unique(sizes.begin(), sizes.end()), sizes.end());

		const double maxSide = std::max(gray.cols, gray.rows);
		const double minPerimeter = p.minMarkerPerimeterRate * maxSide, maxPerimeter = p.maxMarkerPerimeterRate * maxSide;
		const int cell = std::max(2, p.perspectiveRemovePixelPerCell);
		const int cellMargin = int(p.perspectiveRemoveIgnoredMarginPerCell * cell);
		const int inner = std::max(1, cell - 2 * cellMargin);
		const int B = std::max(1, p.markerBorderBits);
		const float border = float(p.minDistanceToBorder);

		cv::Mat thresh, warped;
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Point> approx;
		const int step = std::max(1, p.adaptiveThreshWinSizeStep);
		for (int win = std::max(3, p.adaptiveThreshWinSizeMin); win <= std::max(3, p.adaptiveThreshWinSizeMax); win += step) {
			cv::adaptiveThreshold(gray, thresh, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, win % 2 ? win : win + 1,
				p.adaptiveThreshConstant);
			cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
			for (auto & contour : contours) {
				const double perimeter = double(contour.size());
				if (perimeter < minPerimeter || perimeter > maxPerimeter) continue;
				cv::approxPolyDP(contour, approx, perimeter * p.polygonalApproxAccuracyRate, true);
				if (approx.size() != 4 || !cv::isContourConvex(approx)) continue;
				double minSide2 = std::numeric_limits<double>::max();
				bool nearBorder = false;
				for (int k = 0; k < 4; k++) {
					const cv::Point d = approx[k] - approx[(k + 1) % 4];
					minSide2 = std::min(minSide2, double(d.dot(d)));
					nearBorder |= approx[k].x < border || approx[k].y < border || approx[k].x > gray.cols - 1 - border
						|| approx[k].y > gray.rows - 1 - border;
				}
				if (nearBorder || minSide2 < std::pow(p.minCornerDistanceRate * perimeter, 2)) continue;

				std::vector<cv::Point2f> q(approx.begin(), approx.end());
				const cv::Point2f d1 = q[1] - q[0], d2 = q[2] - q[0];
				if (d1.x * d2.y - d1.y * d2.x < 0) std::swap(q[1], q[3]); // clockwise in the image (y down)

				// the same marker again (another threshold window, the other edge of its border)
				const cv::Point2f centre = centreOf(q);
				const float side = std::sqrt(float(std::abs(cv::contourArea(q))));
				bool known = false;
				for (auto & f : found) known |= cv::norm(centreOf(f.corners) - centre) < 0.3f * side;
				if (known) continue;

				bool matched = false;
				for (int n : sizes) {
					const int s = n + 2 * B, S = s * cell;
					const cv::Point2f dst[4] = { { 0, 0 }, { S - 1.f, 0 }, { S - 1.f, S - 1.f }, { 0, S - 1.f } };
					cv::warpPerspective(gray, warped, cv::getPerspectiveTransform(q.data(), dst), cv::Size(S, S), cv::INTER_NEAREST);
					cv::Scalar mean, stddev;
					cv::meanStdDev(warped(cv::Rect(cell / 2, cell / 2, S - cell, S - cell)), mean, stddev);
					if (stddev[0] < p.minOtsuStdDev) break; // flat: not a marker at any size
					cv::threshold(warped, warped, 125, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
					cv::Mat bits(s, s, CV_8UC1);
					for (int y = 0; y < s; y++) {
						for (int x = 0; x < s; x++) {
							const cv::Mat square = warped(cv::Rect(x * cell + cellMargin, y * cell + cellMargin, inner, inner));
							bits.at<uchar>(y, x) = cv::countNonZero(square) * 2 > int(square.total()) ? 1 : 0;
						}
					}
					// the border must be black (white for inverted markers)
					auto borderErrors = [&](const cv::Mat & b) {
						int e = 0;
						for (int y = 0; y < s; y++) {
							for (int x = 0; x < s; x++) {
								if ((y < B || y >= s - B || x < B || x >= s - B) && b.at<uchar>(y, x)) e++;
							}
						}
						return e;
					};
					const int maxErrors = int(n * n * p.maxErroneousBitsInBorderRate);
					if (borderErrors(bits) > maxErrors) {
						if (!p.detectInvertedMarker) continue;
						bits = 1 - bits;
						if (borderErrors(bits) > maxErrors) continue;
					}
					const uint64_t code = toCode(bits(cv::Rect(B, B, n, n)));
					for (auto * t : tables) {
						if (t->markerSize != n) continue;
						auto it = t->codes.find(code);
						if (it == t->codes.end()) continue;
						AnyMarker m;
						m.id = it->second.first;
						m.dictionary = ofxArucoUtils::getSmallestDictionary(t->family, m.id);
						m.markerSize = n;
						const int r = it->second.second; // the printed top-left corner is r corners further
						for (int k = 0; k < 4; k++) m.corners.push_back(q[(k + r) % 4]);
						found.push_back(std::move(m));
						matched = true;
						break;
					}
					if (matched) break;
				}
			}
		}

		if (p.cornerRefinementMethod != cv::aruco::CORNER_REFINE_NONE) {
			const cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
				std::max(1, p.cornerRefinementMaxIterations), p.cornerRefinementMinAccuracy);
			for (auto & m : found) {
				const float cellPx = std::sqrt(float(std::abs(cv::contourArea(m.corners)))) / float(m.markerSize + 2 * B);
				const int half = std::max(2, std::min(p.cornerRefinementWinSize, int(cellPx * 0.5f)));
				cv::cornerSubPix(gray, m.corners, cv::Size(half, half), cv::Size(-1, -1), criteria);
			}
		}
		return found;
	}

	bool isPlanar(const std::vector<cv::Point3f> & obj) {
		for (auto & p : obj) {
			if (std::abs(p.z) > 1e-6f) return false;
		}
		return true;
	}
}

// Detector objects, only touched by whichever thread runs detection.
struct ofxAruco::Worker {
	std::shared_ptr<const Config> config;
	cv::aruco::ArucoDetector detector;
	std::vector<cv::aruco::ArucoDetector> extraDetectors; // one per Config::extraDictionaries
	std::vector<std::unique_ptr<cv::aruco::CharucoDetector>> charucoDetectors;
	ofxArucoIntrinsics scaledIntrinsics; // intrinsics at the current image size
	cv::Size scaledFor;

	void configure(const std::shared_ptr<const Config> & c) {
		if (c == config) return;
		config = c;
		detector = cv::aruco::ArucoDetector(getDictionary(c->dictionary), c->detectorParams, c->refineParams);
		extraDetectors.clear();
		for (int d : c->extraDictionaries) {
			extraDetectors.emplace_back(getDictionary(d), c->detectorParams, c->refineParams);
		}
		for (int f : c->anyFamilies) getCodeTable(f); // build the tables now, not on the first frame
		scaledFor = cv::Size();
		charucoDetectors.clear();
		for (auto & b : c->boards) {
			if (b.getType() == ofxArucoBoard::Type::Charuco) {
				cv::aruco::CharucoParameters cp;
				cp.tryRefineMarkers = false; // done once for all boards below
				charucoDetectors.emplace_back(std::make_unique<cv::aruco::CharucoDetector>(
					b.getCvCharuco(), cp, c->detectorParams, c->refineParams));
			} else {
				charucoDetectors.emplace_back(nullptr);
			}
		}
	}

	const ofxArucoIntrinsics & intrinsicsFor(cv::Size size) {
		if (size != scaledFor) {
			scaledFor = size;
			scaledIntrinsics = config->intrinsics;
			if (scaledIntrinsics.isValid() && scaledIntrinsics.getImageSize() != size) {
				ofLogNotice("ofxAruco") << "image is " << size.width << "x" << size.height << ", intrinsics are for "
										<< scaledIntrinsics.getWidth() << "x" << scaledIntrinsics.getHeight() << ": scaling them";
				scaledIntrinsics = scaledIntrinsics.getScaled(size.width, size.height);
			}
			// ChArUco corner interpolation is more accurate with intrinsics
			if (scaledIntrinsics.isValid()) {
				for (auto & d : charucoDetectors) {
					if (!d) continue;
					auto cp = d->getCharucoParameters();
					cp.cameraMatrix = scaledIntrinsics.getCameraMatrix();
					cp.distCoeffs = scaledIntrinsics.getDistCoeffs();
					d->setCharucoParameters(cp);
				}
			}
		}
		return scaledIntrinsics;
	}

	ofxArucoResult process(const cv::Mat & gray) {
		const uint64_t t0 = ofGetElapsedTimeMicros();
		const Config & c = *config;
		ofxArucoResult r;
		r.imageSize = { gray.cols, gray.rows };

		const auto & intr = intrinsicsFor(gray.size());
		const bool hasK = intr.isValid();
		const cv::Mat & K = intr.getCameraMatrix();
		const cv::Mat & D = intr.getDistCoeffs();

		std::vector<std::vector<cv::Point2f>> corners, rejected;
		std::vector<int> ids;
		detector.detectMarkers(gray, corners, ids, rejected);

		auto refine = [&](cv::aruco::ArucoDetector & d, int pass, std::vector<std::vector<cv::Point2f>> & mc, std::vector<int> & mi,
						  std::vector<std::vector<cv::Point2f>> & mr) {
			if (!c.refineWithBoards) return;
			for (size_t b = 0; b < c.boards.size(); b++) {
				if (c.boardPass[b] != pass) continue;
				d.refineDetectedMarkers(gray, c.boards[b].getCvBoard(), mc, mi, mr,
					hasK ? cv::InputArray(K) : cv::noArray(), hasK ? cv::InputArray(D) : cv::noArray());
			}
		};
		refine(detector, -1, corners, ids, rejected);
		std::vector<int> dicts(ids.size(), c.dictionary);

		// boards in other dictionaries
		for (size_t k = 0; k < extraDetectors.size(); k++) {
			std::vector<std::vector<cv::Point2f>> ec, er;
			std::vector<int> ei;
			extraDetectors[k].detectMarkers(gray, ec, ei, er);
			refine(extraDetectors[k], int(k), ec, ei, er);
			for (size_t i = 0; i < ei.size(); i++) {
				if (insideAny(corners, centreOf(ec[i]))) continue;
				corners.push_back(ec[i]);
				ids.push_back(ei[i]);
				dicts.push_back(c.extraDictionaries[k]);
			}
		}

		// every other dictionary
		if (!c.anyFamilies.empty()) {
			for (auto & a : scanAnyDictionary(gray, c.anyFamilies, c.detectorParams)) {
				if (insideAny(corners, centreOf(a.corners))) continue;
				corners.push_back(a.corners);
				ids.push_back(a.id);
				dicts.push_back(a.dictionary);
			}
		}

		// --- single markers
		std::vector<int> families(ids.size());
		r.markers.resize(ids.size());
		for (size_t i = 0; i < ids.size(); i++) {
			auto & m = r.markers[i];
			m.id = ids[i];
			m.dictionary = dicts[i];
			families[i] = ofxArucoUtils::getDictionaryFamily(dicts[i]);
			for (int k = 0; k < 4; k++) m.corners[k] = { corners[i][k].x, corners[i][k].y };
			auto it = c.markerLengths.find({ families[i], m.id });
			const float L = it != c.markerLengths.end() ? it->second : c.markerLength;
			m.length = L;
			if (hasK && c.estimateMarkerPoses && L > 0) {
				const std::vector<cv::Point3f> markerObj = {
					{ -L / 2, L / 2, 0 }, { L / 2, L / 2, 0 }, { L / 2, -L / 2, 0 }, { -L / 2, -L / 2, 0 }
				};
				estimateMarkerPose(markerObj, corners[i], K, D, m);
			}
		}

		// --- boards
		r.boards.resize(c.boards.size());
		for (size_t b = 0; b < c.boards.size(); b++) {
			const auto & board = c.boards[b];
			auto & bp = r.boards[b];
			bp.boardIndex = int(b);
			bp.name = board.getName();

			// the markers of the board's dictionary
			const int family = ofxArucoUtils::getDictionaryFamily(board.getDictionary());
			std::vector<std::vector<cv::Point2f>> mc;
			std::vector<int> mi;
			for (size_t i = 0; i < ids.size(); i++) {
				if (families[i] != family) continue;
				mc.push_back(corners[i]);
				mi.push_back(ids[i]);
			}
			if (mi.empty()) continue;

			std::vector<cv::Point3f> obj;
			std::vector<cv::Point2f> img;
			if (charucoDetectors[b]) {
				std::vector<cv::Point2f> chCorners;
				std::vector<int> chIds;
				charucoDetectors[b]->detectBoard(gray, chCorners, chIds, mc, mi);
				if (chIds.size() >= 4 && !board.getCvCharuco().checkCharucoCornersCollinear(chIds)) {
					board.getCvCharuco().matchImagePoints(chCorners, chIds, obj, img);
					bp.charucoIds = chIds;
				}
			} else {
				board.getCvBoard().matchImagePoints(mc, mi, obj, img);
			}

			bp.numPoints = int(obj.size());
			bp.imagePoints.reserve(img.size());
			for (auto & p : img) bp.imagePoints.emplace_back(p.x, p.y);
			bp.objectPoints.reserve(obj.size());
			for (auto & p : obj) bp.objectPoints.emplace_back(p.x, p.y, p.z);

			if (hasK && c.estimateBoardPoses && obj.size() >= 4) {
				try {
					const int method = isPlanar(obj) ? cv::SOLVEPNP_IPPE : cv::SOLVEPNP_SQPNP;
					if (cv::solvePnP(obj, img, K, D, bp.rvec, bp.tvec, false, method)) {
						cv::solvePnPRefineLM(obj, img, K, D, bp.rvec, bp.tvec);
						bp.found = true;
						bp.pose = toMat4(bp.rvec, bp.tvec);
						bp.reprojectionError = rmsError(obj, img, bp.rvec, bp.tvec, K, D);
					}
				} catch (const cv::Exception & e) {
					ofLogVerbose("ofxAruco") << "board pose failed: " << e.what();
				}
			}
		}

		r.rejected.resize(rejected.size());
		for (size_t i = 0; i < rejected.size(); i++) {
			for (int k = 0; k < 4 && k < (int)rejected[i].size(); k++) r.rejected[i][k] = { rejected[i][k].x, rejected[i][k].y };
		}

		r.detectMillis = (ofGetElapsedTimeMicros() - t0) / 1000.f;
		return r;
	}
};

//--------------------------------------------------------------
ofxAruco::ofxAruco() {
	parameters.setName("aruco");
	parameters.add(dictionary.set("dictionary", cv::aruco::DICT_5X5_100, 0, cv::aruco::DICT_ARUCO_MIP_36h12));
	parameters.add(dictionaryName.set("dictionary name", dictionaryToName(dictionary)));
	dictionaryName.setSerializable(false);
	parameters.add(anyDictionary.set("any dictionary", false));
	parameters.add(markerLength.set("marker length (m)", 0.05f, 0.001f, 1.f));
	parameters.add(estimateMarkerPoses.set("marker poses", true));
	parameters.add(estimateBoardPoses.set("board poses", true));
	parameters.add(refineWithBoards.set("refine with boards", true));
	parameters.add(useAruco3Detection.set("fast (aruco3)", false));
	parameters.add(aruco3MinSide.set("aruco3 min side (px)", 32, 8, 128));
	parameters.add(aruco3MinMarkerRatio.set("aruco3 min ratio", 0.f, 0.f, 0.2f));
	parameters.add(cornerRefinement.set("corner refinement", cv::aruco::CORNER_REFINE_SUBPIX, 0, 3));
	parameters.add(adaptiveThreshWinSizeMin.set("thresh win min", 3, 3, 99));
	parameters.add(adaptiveThreshWinSizeMax.set("thresh win max", 23, 3, 99));
	parameters.add(adaptiveThreshWinSizeStep.set("thresh win step", 10, 1, 50));
	parameters.add(adaptiveThreshConstant.set("thresh constant", 7, 0, 30));
	parameters.add(minMarkerPerimeterRate.set("min perimeter", 0.03f, 0.001f, 1.f));
	parameters.add(maxMarkerPerimeterRate.set("max perimeter", 4.f, 0.1f, 4.f));
	parameters.add(polygonalApproxAccuracyRate.set("polygon accuracy", 0.03f, 0.001f, 0.2f));
	parameters.add(errorCorrectionRate.set("error correction", 0.6f, 0.f, 1.f));
	parameters.add(detectInvertedMarker.set("inverted markers", false));
	parameters.add(shortPixelsScale.set("16bit scale (0 auto)", 0.f, 0.f, 1.f));

	listeners.push(parameters.parameterChangedE().newListener([this](ofAbstractParameter & p) {
		if (p.getName() == dictionary.getName()) {
			dictionaryName.set(dictionaryToName(dictionary));
		}
		markConfigDirty();
	}));
}

ofxAruco::~ofxAruco() {
	stopThread();
}

void ofxAruco::setup(int dict, bool _threaded) {
	dictionary = dict;
	threaded = _threaded;
	if (!worker) worker = std::make_unique<Worker>();
	if (!isSetup) {
		listeners.push(ofEvents().update.newListener(this, &ofxAruco::onUpdate, OF_EVENT_ORDER_BEFORE_APP));
		isSetup = true;
	}
	markConfigDirty();
	if (threaded) startThread();
}

void ofxAruco::setThreaded(bool t) {
	if (t == threaded) return;
	threaded = t;
	if (!isSetup) return;
	if (threaded) {
		startThread();
	} else {
		stopThread();
	}
}

void ofxAruco::setIntrinsics(const ofxArucoIntrinsics & i) {
	intrinsics = i;
	markConfigDirty();
}

bool ofxAruco::loadIntrinsics(const std::string & path) {
	ofxArucoIntrinsics i;
	if (!i.load(path)) return false;
	setIntrinsics(i);
	ofLogNotice("ofxAruco") << "intrinsics " << i.getWidth() << "x" << i.getHeight() << " fx " << i.getFx() << " fy " << i.getFy();
	return true;
}

int ofxAruco::addBoard(const ofxArucoBoard & board) {
	if (!board.isValid()) {
		ofLogError("ofxAruco") << "addBoard: invalid board";
		return -1;
	}
	if (getDictionaryFamily(board.getDictionary()) != getDictionaryFamily(dictionary)) {
		ofLogNotice("ofxAruco") << "board '" << board.getName() << "' uses " << dictionaryToName(board.getDictionary())
								<< ", the detector " << dictionaryToName(dictionary) << ": its markers are searched in an extra pass";
	}
	boards.push_back(board);
	markConfigDirty();
	return int(boards.size()) - 1;
}

int ofxAruco::loadBoard(const std::string & path) {
	ofxArucoBoard b;
	if (!b.load(path)) return -1;
	return addBoard(b);
}

void ofxAruco::clearBoards() {
	boards.clear();
	markConfigDirty();
}

void ofxAruco::setMarkerLength(int dict, int id, float length) {
	const int family = getDictionaryFamily(dict);
	if (family < 0) {
		ofLogError("ofxAruco") << "setMarkerLength: unknown dictionary " << dict;
		return;
	}
	markerLengths[{ family, id }] = length;
	markConfigDirty();
}

void ofxAruco::clearMarkerLengths() {
	markerLengths.clear();
	markConfigDirty();
}

float ofxAruco::getMarkerLength(int dict, int id) const {
	const int family = getDictionaryFamily(dict);
	auto it = markerLengths.find({ family, id });
	if (it != markerLengths.end()) return it->second;
	for (auto & b : boards) {
		if (b.getMarkerLength() <= 0 || getDictionaryFamily(b.getDictionary()) != family) continue;
		const auto & ids = b.getCvBoard().getIds();
		if (std::find(ids.begin(), ids.end(), id) != ids.end()) return b.getMarkerLength();
	}
	return markerLength;
}

bool ofxAruco::saveSettings(const std::string & path) const {
	ofJson j;
	ofSerialize(j, parameters);
	return ofxArucoUtils::saveJson(path, j);
}

bool ofxAruco::loadSettings(const std::string & path) {
	const std::string fullPath = ofToDataPath(path, true);
	if (!ofFile::doesFileExist(fullPath)) {
		ofLogNotice("ofxAruco") << "no settings file " << fullPath << ", using defaults";
		return false;
	}
	ofDeserialize(ofLoadJson(fullPath), parameters);
	return true;
}

void ofxAruco::markConfigDirty() {
	configDirty = true;
}

std::shared_ptr<const ofxAruco::Config> ofxAruco::makeConfig() const {
	auto c = std::make_shared<Config>();
	c->dictionary = dictionary;
	auto & p = c->detectorParams;
	p.cornerRefinementMethod = ofClamp(cornerRefinement, 0, 3);
	p.adaptiveThreshWinSizeMin = std::max(3, adaptiveThreshWinSizeMin.get());
	p.adaptiveThreshWinSizeMax = std::max(p.adaptiveThreshWinSizeMin, adaptiveThreshWinSizeMax.get());
	p.adaptiveThreshWinSizeStep = std::max(1, adaptiveThreshWinSizeStep.get());
	p.adaptiveThreshConstant = adaptiveThreshConstant;
	p.minMarkerPerimeterRate = minMarkerPerimeterRate;
	p.maxMarkerPerimeterRate = std::max(maxMarkerPerimeterRate.get(), minMarkerPerimeterRate.get());
	p.polygonalApproxAccuracyRate = polygonalApproxAccuracyRate;
	p.errorCorrectionRate = errorCorrectionRate;
	p.detectInvertedMarker = detectInvertedMarker;
	p.useAruco3Detection = useAruco3Detection;
	p.minSideLengthCanonicalImg = aruco3MinSide;
	p.minMarkerLengthRatioOriginalImg = aruco3MinMarkerRatio;
	c->intrinsics = intrinsics;
	c->boards = boards;

	// boards the main detector doesn't cover (another family, or ids past its size) get their own pass
	const int primaryFamily = getDictionaryFamily(dictionary);
	for (auto & b : boards) {
		const int d = b.getDictionary(), f = getDictionaryFamily(d);
		int pass = -1;
		if (f != primaryFamily || getDictionarySize(d) > getDictionarySize(dictionary)) {
			pass = -2;
			for (size_t k = 0; k < c->extraDictionaries.size(); k++) {
				const int e = c->extraDictionaries[k];
				if (getDictionaryFamily(e) != f) continue;
				if (getDictionarySize(d) > getDictionarySize(e)) c->extraDictionaries[k] = d; // the larger one holds both
				pass = int(k);
			}
			if (pass == -2) {
				c->extraDictionaries.push_back(d);
				pass = int(c->extraDictionaries.size()) - 1;
			}
		}
		c->boardPass.push_back(pass);
	}
	if (anyDictionary) {
		for (int f : getAllDictionaryFamilies()) {
			bool covered = f == primaryFamily;
			for (int e : c->extraDictionaries) covered |= getDictionaryFamily(e) == f;
			if (!covered) c->anyFamilies.push_back(f);
		}
	}
	// marker sizes: the ones set by the app, then the boards' (first board wins)
	c->markerLengths = markerLengths;
	for (auto & b : boards) {
		if (b.getMarkerLength() <= 0) continue;
		const int f = getDictionaryFamily(b.getDictionary());
		for (int id : b.getCvBoard().getIds()) c->markerLengths.emplace(std::make_pair(f, id), b.getMarkerLength());
	}
	c->markerLength = markerLength;
	c->estimateMarkerPoses = estimateMarkerPoses;
	c->estimateBoardPoses = estimateBoardPoses;
	c->refineWithBoards = refineWithBoards;
	return c;
}

//--------------------------------------------------------------
void ofxAruco::detect(const ofPixels & pixels, uint64_t userTag) {
	if (!pixels.isAllocated()) return;
	const int ch = int(pixels.getNumChannels());
	cv::Mat src(int(pixels.getHeight()), int(pixels.getWidth()), CV_8UC(ch), const_cast<unsigned char *>(pixels.getData()));
	cv::Mat gray;
	switch (ch) {
	case 1:
		gray = src.clone();
		break;
	case 3:
		cv::cvtColor(src, gray, pixels.getPixelFormat() == OF_PIXELS_BGR ? cv::COLOR_BGR2GRAY : cv::COLOR_RGB2GRAY);
		break;
	case 4:
		cv::cvtColor(src, gray, pixels.getPixelFormat() == OF_PIXELS_BGRA ? cv::COLOR_BGRA2GRAY : cv::COLOR_RGBA2GRAY);
		break;
	default:
		cv::extractChannel(src, gray, 0);
		break;
	}
	submit(std::move(gray), userTag);
}

void ofxAruco::detect(const ofShortPixels & pixels, uint64_t userTag) {
	if (!pixels.isAllocated()) return;
	cv::Mat src(int(pixels.getHeight()), int(pixels.getWidth()), CV_16UC(int(pixels.getNumChannels())),
		const_cast<unsigned short *>(pixels.getData()));
	if (src.channels() > 1) {
		cv::extractChannel(src, shortScratch, 0);
		src = shortScratch;
	}
	double scale = shortPixelsScale;
	if (scale <= 0) {
		// automatic: put 3x the mean brightness at white (robust to a few very bright pixels)
		const double mean = cv::mean(src)[0];
		scale = mean > 0 ? 255.0 / (3.0 * mean) : 1.0;
	}
	cv::Mat gray;
	src.convertTo(gray, CV_8U, scale);
	submit(std::move(gray), userTag);
}

void ofxAruco::detect(const cv::Mat & image, uint64_t userTag) {
	if (image.empty()) return;
	cv::Mat gray;
	if (image.channels() == 3) {
		cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	} else if (image.channels() == 4) {
		cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
	} else {
		gray = image.clone();
	}
	if (gray.depth() != CV_8U) gray.convertTo(gray, CV_8U);
	submit(std::move(gray), userTag);
}

void ofxAruco::submit(cv::Mat && gray, uint64_t userTag) {
	if (!isSetup) {
		ofLogWarning("ofxAruco") << "detect() called before setup(), using defaults";
		setup(dictionary, threaded);
	}
	if (configDirty) {
		config = makeConfig();
		configDirty = false;
	}
	const uint64_t now = ofGetElapsedTimeMicros();
	const uint64_t frameNumber = ++submittedFrames;

	if (!threaded) {
		worker->configure(config);
		ofxArucoResult r = worker->process(gray);
		r.userTag = userTag;
		r.timeMicros = now;
		r.frameNumber = frameNumber;
		result = std::move(r);
		frameNew = true;
		const uint64_t t = ofGetElapsedTimeMicros();
		if (lastResultTime) detectionFps = ofLerp(detectionFps, 1e6f / std::max<uint64_t>(1, t - lastResultTime), 0.1f);
		lastResultTime = t;
		return;
	}

	{
		std::lock_guard<std::mutex> lock(mutex);
		pendingImage = std::move(gray); // replaces a frame the worker didn't get to yet
		pendingTag = userTag;
		pendingTime = now;
		pendingFrameNumber = frameNumber;
		pendingConfig = config;
		hasPending = true;
	}
	condition.notify_one();
}

bool ofxAruco::update() {
	if (!threaded) return frameNew;
	std::lock_guard<std::mutex> lock(mutex);
	if (!hasNewResult) return false;
	result = std::move(latestResult);
	latestResult = ofxArucoResult();
	hasNewResult = false;
	frameNew = true;
	const uint64_t t = ofGetElapsedTimeMicros();
	if (lastResultTime) detectionFps = ofLerp(detectionFps, 1e6f / std::max<uint64_t>(1, t - lastResultTime), 0.1f);
	lastResultTime = t;
	return true;
}

void ofxAruco::waitForResult(uint64_t timeoutMillis) {
	if (threaded) {
		std::unique_lock<std::mutex> lock(mutex);
		resultCondition.wait_for(lock, std::chrono::milliseconds(timeoutMillis), [this] {
			return !hasPending && !workerBusy;
		});
	}
	update();
}

void ofxAruco::onUpdate(ofEventArgs &) {
	frameNew = false;
	update();
}

void ofxAruco::startThread() {
	if (thread.joinable()) return;
	if (!worker) worker = std::make_unique<Worker>();
	{
		std::lock_guard<std::mutex> lock(mutex);
		running = true;
	}
	thread = std::thread(&ofxAruco::threadedFunction, this);
}

void ofxAruco::stopThread() {
	{
		std::lock_guard<std::mutex> lock(mutex);
		running = false;
	}
	condition.notify_all();
	if (thread.joinable()) thread.join();
}

void ofxAruco::threadedFunction() {
	while (true) {
		cv::Mat image;
		std::shared_ptr<const Config> cfg;
		uint64_t tag, time, frameNumber;
		{
			std::unique_lock<std::mutex> lock(mutex);
			condition.wait(lock, [this] { return hasPending || !running; });
			if (!running) break;
			image = std::move(pendingImage);
			pendingImage = cv::Mat();
			cfg = pendingConfig;
			tag = pendingTag;
			time = pendingTime;
			frameNumber = pendingFrameNumber;
			hasPending = false;
			workerBusy = true;
		}

		ofxArucoResult r;
		try {
			worker->configure(cfg);
			r = worker->process(image);
		} catch (const std::exception & e) {
			ofLogError("ofxAruco") << "detection failed: " << e.what();
		}
		r.userTag = tag;
		r.timeMicros = time;
		r.frameNumber = frameNumber;

		{
			std::lock_guard<std::mutex> lock(mutex);
			latestResult = std::move(r);
			hasNewResult = true;
			workerBusy = false;
		}
		resultCondition.notify_all();
	}
}

//--------------------------------------------------------------
const ofxArucoBoardPose & ofxAruco::getBoardPose(int boardIndex) const {
	static const ofxArucoBoardPose empty;
	if (boardIndex < 0 || boardIndex >= (int)result.boards.size()) return empty;
	return result.boards[boardIndex];
}

ofRectangle ofxAruco::resolveViewport(const ofRectangle & viewport) const {
	if (viewport.width > 0 && viewport.height > 0) return viewport;
	if (result.imageSize.x > 0) return ofRectangle(0, 0, result.imageSize.x, result.imageSize.y);
	return ofRectangle(0, 0, intrinsics.getWidth(), intrinsics.getHeight());
}

void ofxAruco::draw(const ofRectangle & viewport) const {
	drawMarkers(viewport);
	drawBoards(viewport);
	drawAxes(viewport);
}

void ofxAruco::drawMarkers(const ofRectangle & viewport, bool drawIds) const {
	if (result.imageSize.x <= 0) return;
	const ofRectangle vp = resolveViewport(viewport);
	const glm::vec2 s(vp.width / result.imageSize.x, vp.height / result.imageSize.y);
	const glm::vec2 o(vp.x, vp.y);
	ofPushStyle();
	ofNoFill();
	ofSetLineWidth(2);
	for (auto & m : result.markers) {
		ofSetColor(0, 255, 0);
		ofBeginShape();
		for (auto & c : m.corners) ofVertex(o + c * s);
		ofEndShape(true);
		ofSetColor(255, 0, 0);
		ofDrawCircle(o + m.corners[0] * s, 3); // first (top-left) corner
		if (drawIds) {
			std::string label = ofToString(m.id);
			if (getDictionaryFamily(m.dictionary) != getDictionaryFamily(dictionary)) label += " " + dictionaryToShortName(m.dictionary);
			ofDrawBitmapStringHighlight(label, o + m.getCenter() * s, ofColor(0, 0, 0, 160), ofColor::yellow);
		}
	}
	ofPopStyle();
}

void ofxAruco::drawBoards(const ofRectangle & viewport) const {
	if (result.imageSize.x <= 0) return;
	const ofRectangle vp = resolveViewport(viewport);
	const glm::vec2 s(vp.width / result.imageSize.x, vp.height / result.imageSize.y);
	const glm::vec2 o(vp.x, vp.y);
	ofPushStyle();
	ofFill();
	for (auto & b : result.boards) {
		ofSetColor(b.found ? ofColor(0, 200, 255) : ofColor(255, 120, 0));
		const float r = b.charucoIds.empty() ? 1.5f : 3.f;
		for (auto & p : b.imagePoints) ofDrawCircle(o + p * s, r);
	}
	ofPopStyle();
}

void ofxAruco::drawRejected(const ofRectangle & viewport) const {
	if (result.imageSize.x <= 0) return;
	const ofRectangle vp = resolveViewport(viewport);
	const glm::vec2 s(vp.width / result.imageSize.x, vp.height / result.imageSize.y);
	const glm::vec2 o(vp.x, vp.y);
	ofPushStyle();
	ofNoFill();
	ofSetColor(255, 0, 255, 160);
	for (auto & q : result.rejected) {
		ofBeginShape();
		for (auto & c : q) ofVertex(o + c * s);
		ofEndShape(true);
	}
	ofPopStyle();
}

void ofxAruco::drawAxes(const ofRectangle & viewport) const {
	if (!intrinsics.isValid() || result.imageSize.x <= 0) return;
	beginCamera(viewport);
	ofPushStyle();
	ofSetLineWidth(3);
	for (auto & m : result.markers) {
		if (!m.hasPose) continue;
		ofPushMatrix();
		ofMultMatrix(m.pose);
		ofDrawAxis((m.length > 0 ? m.length : markerLength.get()) * 0.5f);
		ofPopMatrix();
	}
	for (size_t i = 0; i < result.boards.size() && i < boards.size(); i++) {
		if (!result.boards[i].found) continue;
		ofPushMatrix();
		ofMultMatrix(result.boards[i].pose);
		const glm::vec2 size = boards[i].getSize();
		ofDrawAxis(std::min(size.x, size.y) * 0.5f);
		ofPopMatrix();
	}
	ofPopStyle();
	end();
}

void ofxAruco::beginCamera(const ofRectangle & viewport) const {
	const ofRectangle vp = resolveViewport(viewport);
	ofxArucoIntrinsics intr = intrinsics;
	if (result.imageSize.x > 0 && intr.isValid() && intr.getImageSize() != cv::Size(result.imageSize.x, result.imageSize.y)) {
		intr = intr.getScaled(result.imageSize.x, result.imageSize.y);
	}
	intr.begin(vp);
}

void ofxAruco::begin(int markerIndex, const ofRectangle & viewport) const {
	beginCamera(viewport);
	if (markerIndex >= 0 && markerIndex < (int)result.markers.size()) {
		ofMultMatrix(result.markers[markerIndex].pose);
	}
}

void ofxAruco::beginBoard(int boardIndex, const ofRectangle & viewport) const {
	beginCamera(viewport);
	ofMultMatrix(getBoardPose(boardIndex).pose);
}

void ofxAruco::end() const {
	ofGetCurrentRenderer()->popView();
}

glm::mat4 ofxAruco::getProjectionMatrix(float nearDist, float farDist) const {
	ofxArucoIntrinsics intr = intrinsics;
	if (result.imageSize.x > 0 && intr.isValid() && intr.getImageSize() != cv::Size(result.imageSize.x, result.imageSize.y)) {
		intr = intr.getScaled(result.imageSize.x, result.imageSize.y);
	}
	return intr.getProjectionMatrix(nearDist, farDist);
}

glm::mat4 ofxAruco::getModelViewMatrix(int markerIndex) const {
	if (markerIndex < 0 || markerIndex >= (int)result.markers.size()) return cvToGl();
	return cvToGl() * result.markers[markerIndex].pose;
}

glm::mat4 ofxAruco::getModelViewMatrixBoard(int boardIndex) const {
	return cvToGl() * getBoardPose(boardIndex).pose;
}

void ofxAruco::getMarkerImage(int dict, int markerId, int sizePixels, ofPixels & pixels, int borderBits) {
	const auto d = getDictionary(dict);
	cv::Mat img;
	d.generateImageMarker(markerId, sizePixels, img, borderBits);
	// one cell of white quiet zone around the marker
	const int quiet = sizePixels / (d.markerSize + 2 * borderBits);
	cv::copyMakeBorder(img, img, quiet, quiet, quiet, quiet, cv::BORDER_CONSTANT, cv::Scalar(255));
	pixels.setFromPixels(img.data, img.cols, img.rows, OF_PIXELS_GRAY);
}
