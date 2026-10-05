#include "ofxArucoIntrinsics.h"
#include "ofxArucoTypes.h"
#include <opencv2/calib3d.hpp>

void ofxArucoIntrinsics::setup(double fx, double fy, double cx, double cy, int width, int height,
	const std::vector<double> & distortion) {
	cameraMatrix = cv::Mat(cv::Matx33d(fx, 0, cx, 0, fy, cy, 0, 0, 1), true);
	distCoeffs = distortion.empty() ? cv::Mat() : cv::Mat(distortion, true).reshape(1, 1);
	imageSize = cv::Size(width, height);
}

void ofxArucoIntrinsics::setup(const cv::Mat & K, const cv::Mat & dist, cv::Size size) {
	K.convertTo(cameraMatrix, CV_64F);
	if (dist.empty()) {
		distCoeffs = cv::Mat();
	} else {
		dist.reshape(1, 1).convertTo(distCoeffs, CV_64F);
	}
	imageSize = size;
}

bool ofxArucoIntrinsics::load(const std::string & path) {
	const std::string fullPath = ofToDataPath(path, true);
	if (!ofFile::doesFileExist(fullPath)) {
		ofLogError("ofxArucoIntrinsics") << "file not found: " << fullPath;
		return false;
	}
	const std::string ext = ofToLower(ofFilePath::getFileExt(fullPath));

	if (ext == "json") {
		return fromJson(ofLoadJson(fullPath));
	}

	if (ext == "int") {
		// Legacy ArUco 1.x format: "fx = ...", "fy = ...", "cx = ...", "cy = ...", "width = ...", "height = ..."
		std::map<std::string, double> values;
		for (auto & line : ofBuffer(ofBufferFromFile(fullPath)).getLines()) {
			auto parts = ofSplitString(line, "=", true, true);
			if (parts.size() == 2) values[parts[0]] = ofToDouble(parts[1]);
		}
		if (!values.count("fx") || !values.count("fy") || !values.count("cx") || !values.count("cy")) {
			ofLogError("ofxArucoIntrinsics") << "missing fx/fy/cx/cy in " << fullPath;
			return false;
		}
		setup(values["fx"], values["fy"], values["cx"], values["cy"], int(values["width"]), int(values["height"]));
		return isValid();
	}

	// OpenCV FileStorage (yml / yaml / xml). Accept the common key names.
	try {
		cv::FileStorage fs(fullPath, cv::FileStorage::READ);
		if (!fs.isOpened()) {
			ofLogError("ofxArucoIntrinsics") << "could not open " << fullPath;
			return false;
		}
		auto firstMat = [&](std::initializer_list<const char *> keys) {
			cv::Mat m;
			for (auto k : keys) {
				if (!fs[k].empty()) {
					fs[k] >> m;
					if (!m.empty()) break;
				}
			}
			return m;
		};
		cv::Mat K = firstMat({ "camera_matrix", "cameraMatrix", "K", "Camera_Matrix" });
		cv::Mat D = firstMat({ "distortion_coefficients", "distCoeffs", "dist_coeffs", "D", "Distortion_Coefficients" });
		cv::Size size;
		if (!fs["image_width"].empty()) {
			size.width = (int)fs["image_width"];
			size.height = (int)fs["image_height"];
		} else if (!fs["imageSize"].empty()) {
			std::vector<int> s;
			fs["imageSize"] >> s;
			if (s.size() == 2) size = cv::Size(s[0], s[1]);
		}
		if (K.empty() || K.rows != 3 || K.cols != 3) {
			ofLogError("ofxArucoIntrinsics") << "no 3x3 camera_matrix in " << fullPath;
			return false;
		}
		if (size.width <= 0) {
			// best effort: assume the principal point is near the center
			size = cv::Size(int(std::round(K.at<double>(0, 2) * 2)), int(std::round(K.at<double>(1, 2) * 2)));
			ofLogWarning("ofxArucoIntrinsics") << "no image size in " << fullPath << ", guessing " << size;
		}
		setup(K, D, size);
		return true;
	} catch (const cv::Exception & e) {
		ofLogError("ofxArucoIntrinsics") << "error reading " << fullPath << ": " << e.what();
		return false;
	}
}

bool ofxArucoIntrinsics::save(const std::string & path) const {
	const std::string fullPath = ofToDataPath(path, true);
	const std::string ext = ofToLower(ofFilePath::getFileExt(fullPath));
	if (ext == "json") {
		return ofxArucoUtils::saveJson(fullPath, toJson());
	}
	try {
		cv::FileStorage fs(fullPath, cv::FileStorage::WRITE);
		fs << "image_width" << imageSize.width;
		fs << "image_height" << imageSize.height;
		fs << "camera_matrix" << cameraMatrix;
		fs << "distortion_coefficients" << distCoeffs;
		return true;
	} catch (const cv::Exception & e) {
		ofLogError("ofxArucoIntrinsics") << "error writing " << fullPath << ": " << e.what();
		return false;
	}
}

ofJson ofxArucoIntrinsics::toJson() const {
	ofJson j;
	j["width"] = imageSize.width;
	j["height"] = imageSize.height;
	j["fx"] = getFx();
	j["fy"] = getFy();
	j["cx"] = getCx();
	j["cy"] = getCy();
	std::vector<double> d;
	for (int i = 0; i < (int)distCoeffs.total(); i++) d.push_back(distCoeffs.at<double>(i));
	j["distortion"] = d;
	j["distortionOrder"] = "OpenCV: k1 k2 p1 p2 [k3 [k4 k5 k6 [s1 s2 s3 s4 [tx ty]]]]";
	return j;
}

bool ofxArucoIntrinsics::fromJson(const ofJson & j) {
	try {
		std::vector<double> d;
		if (j.contains("distortion")) d = j["distortion"].get<std::vector<double>>();
		setup(j.at("fx").get<double>(), j.at("fy").get<double>(), j.at("cx").get<double>(), j.at("cy").get<double>(),
			j.at("width").get<int>(), j.at("height").get<int>(), d);
		return isValid();
	} catch (const std::exception & e) {
		ofLogError("ofxArucoIntrinsics") << "invalid json: " << e.what();
		return false;
	}
}

ofxArucoIntrinsics ofxArucoIntrinsics::getScaled(int width, int height) const {
	ofxArucoIntrinsics out = *this;
	if (!isValid()) return out;
	const double sx = double(width) / imageSize.width;
	const double sy = double(height) / imageSize.height;
	if (std::abs(sx - sy) > 0.01) {
		ofLogWarning("ofxArucoIntrinsics") << "scaling to a different aspect ratio (" << imageSize << " -> "
										   << width << "x" << height << "), results will be approximate";
	}
	out.cameraMatrix = cameraMatrix.clone();
	// scale about pixel centers: (c + 0.5) * s - 0.5
	out.cameraMatrix.at<double>(0, 0) *= sx;
	out.cameraMatrix.at<double>(1, 1) *= sy;
	out.cameraMatrix.at<double>(0, 2) = (getCx() + 0.5) * sx - 0.5;
	out.cameraMatrix.at<double>(1, 2) = (getCy() + 0.5) * sy - 0.5;
	out.imageSize = cv::Size(width, height);
	return out;
}

glm::mat4 ofxArucoIntrinsics::getProjectionMatrix(float n, float f) const {
	if (!isValid()) return glm::mat4(1.f);
	const float W = float(imageSize.width);
	const float H = float(imageSize.height);
	glm::mat4 P(0.f);
	// Maps the OpenGL camera frame (x right, y up, looking down -z) to clip space so
	// that a point lands on the same pixel OpenCV would project it to.
	// (+0.5: OpenCV pixel centers are at integer coordinates)
	P[0][0] = 2.f * float(getFx()) / W;
	P[1][1] = 2.f * float(getFy()) / H;
	P[2][0] = 1.f - 2.f * (float(getCx()) + 0.5f) / W;
	P[2][1] = 2.f * (float(getCy()) + 0.5f) / H - 1.f;
	P[2][2] = -(f + n) / (f - n);
	P[2][3] = -1.f;
	P[3][2] = -2.f * f * n / (f - n);
	return P;
}

void ofxArucoIntrinsics::begin(const ofRectangle & viewport, float nearDist, float farDist) const {
	auto renderer = ofGetCurrentRenderer();
	renderer->pushView();
	renderer->viewport(viewport);
	// same as ofCamera::begin(): no vertical flip, the projection takes care of it
	renderer->setOrientation(ofGetOrientation(), false);
	renderer->matrixMode(OF_MATRIX_PROJECTION);
	renderer->loadMatrix(getProjectionMatrix(nearDist, farDist));
	renderer->matrixMode(OF_MATRIX_MODELVIEW);
	// OpenCV camera frame -> OpenGL camera frame (flip y and z)
	renderer->loadViewMatrix(glm::scale(glm::mat4(1.f), glm::vec3(1.f, -1.f, -1.f)));
}

void ofxArucoIntrinsics::end() const {
	ofGetCurrentRenderer()->popView();
}

double ofxArucoIntrinsics::getFx() const { return cameraMatrix.empty() ? 0 : cameraMatrix.at<double>(0, 0); }
double ofxArucoIntrinsics::getFy() const { return cameraMatrix.empty() ? 0 : cameraMatrix.at<double>(1, 1); }
double ofxArucoIntrinsics::getCx() const { return cameraMatrix.empty() ? 0 : cameraMatrix.at<double>(0, 2); }
double ofxArucoIntrinsics::getCy() const { return cameraMatrix.empty() ? 0 : cameraMatrix.at<double>(1, 2); }

glm::vec2 ofxArucoIntrinsics::getFov() const {
	if (!isValid()) return { 0, 0 };
	return { float(glm::degrees(2 * std::atan(imageSize.width / (2 * getFx())))),
		float(glm::degrees(2 * std::atan(imageSize.height / (2 * getFy())))) };
}

glm::vec2 ofxArucoIntrinsics::project(const glm::vec3 & p) const {
	if (!isValid()) return { 0, 0 };
	std::vector<cv::Point3f> in { { p.x, p.y, p.z } };
	std::vector<cv::Point2f> out;
	cv::projectPoints(in, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cameraMatrix, distCoeffs, out);
	return { out[0].x, out[0].y };
}
