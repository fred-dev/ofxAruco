#include "ofxArucoBoard.h"
#include "ofxArucoTypes.h"
#include <opencv2/core/persistence.hpp>
#include <numeric>

ofxArucoBoard ofxArucoBoard::makeCharuco(int squaresX, int squaresY, float squareLen, float markerLen,
	int dict, int firstId, bool legacy) {
	ofxArucoBoard b;
	b.type = Type::Charuco;
	b.name = "charuco";
	b.countX = squaresX;
	b.countY = squaresY;
	b.squareLength = squareLen;
	b.markerLength = markerLen;
	b.dictionary = dict;
	b.firstMarkerId = firstId;
	b.legacyPattern = legacy;
	b.rebuild();
	return b;
}

ofxArucoBoard ofxArucoBoard::makeGrid(int markersX, int markersY, float markerLen, float separation,
	int dict, int firstId) {
	ofxArucoBoard b;
	b.type = Type::Grid;
	b.name = "grid";
	b.countX = markersX;
	b.countY = markersY;
	b.markerLength = markerLen;
	b.markerSeparation = separation;
	b.dictionary = dict;
	b.firstMarkerId = firstId;
	b.rebuild();
	return b;
}

ofxArucoBoard ofxArucoBoard::makeCustom(int dict, const std::vector<int> & ids,
	const std::vector<std::array<glm::vec3, 4>> & corners) {
	ofxArucoBoard b;
	if (ids.size() != corners.size() || ids.empty()) {
		ofLogError("ofxArucoBoard") << "makeCustom: ids and corners must have the same, non zero, size";
		return b;
	}
	b.type = Type::Custom;
	b.name = "custom";
	b.dictionary = dict;
	b.customIds = ids;
	b.customCorners = corners;
	b.rebuild();
	return b;
}

void ofxArucoBoard::rebuild() {
	const auto dict = ofxArucoUtils::getDictionary(dictionary);
	auto makeIds = [&](int n) {
		std::vector<int> ids(n);
		std::iota(ids.begin(), ids.end(), firstMarkerId);
		return ids;
	};
	try {
		switch (type) {
		case Type::Charuco: {
			const int n = (countX * countY) / 2;
			charuco = cv::aruco::CharucoBoard(cv::Size(countX, countY), squareLength, markerLength, dict, makeIds(n));
			charuco.setLegacyPattern(legacyPattern);
			board = charuco;
			break;
		}
		case Type::Grid:
			board = cv::aruco::GridBoard(cv::Size(countX, countY), markerLength, markerSeparation, dict, makeIds(countX * countY));
			break;
		case Type::Custom: {
			std::vector<std::vector<cv::Point3f>> obj;
			for (auto & c : customCorners) {
				obj.push_back({ { c[0].x, c[0].y, c[0].z }, { c[1].x, c[1].y, c[1].z },
					{ c[2].x, c[2].y, c[2].z }, { c[3].x, c[3].y, c[3].z } });
			}
			board = cv::aruco::Board(obj, dict, customIds);
			break;
		}
		case Type::None:
			break;
		}
	} catch (const cv::Exception & e) {
		ofLogError("ofxArucoBoard") << "could not create board: " << e.what();
		type = Type::None;
	}
}

std::string ofxArucoBoard::getTypeName() const {
	switch (type) {
	case Type::Charuco: return "charuco";
	case Type::Grid: return "grid";
	case Type::Custom: return "custom";
	default: return "none";
	}
}

// lengths are floats: round to micrometers so the json stays readable (0.026, not 0.026000000536)
static double um(float meters) {
	return std::round(double(meters) * 1e6) / 1e6;
}

ofJson ofxArucoBoard::toJson() const {
	ofJson j;
	j["type"] = getTypeName();
	j["name"] = name;
	j["dictionary"] = ofxArucoUtils::dictionaryToName(dictionary);
	switch (type) {
	case Type::Charuco:
		j["squaresX"] = countX;
		j["squaresY"] = countY;
		j["squareLength"] = um(squareLength);
		j["markerLength"] = um(markerLength);
		j["firstMarkerId"] = firstMarkerId;
		j["legacyPattern"] = legacyPattern;
		break;
	case Type::Grid:
		j["markersX"] = countX;
		j["markersY"] = countY;
		j["markerLength"] = um(markerLength);
		j["markerSeparation"] = um(markerSeparation);
		j["firstMarkerId"] = firstMarkerId;
		break;
	case Type::Custom: {
		ofJson markers = ofJson::array();
		for (size_t i = 0; i < customIds.size(); i++) {
			ofJson corners = ofJson::array();
			for (auto & c : customCorners[i]) corners.push_back({ um(c.x), um(c.y), um(c.z) });
			markers.push_back({ { "id", customIds[i] }, { "corners", corners } });
		}
		j["markers"] = markers;
		break;
	}
	default:
		break;
	}
	return j;
}

bool ofxArucoBoard::fromJson(const ofJson & j) {
	try {
		const std::string t = j.at("type").get<std::string>();
		const std::string dictName = j.value("dictionary", std::string("DICT_5X5_100"));
		const int dict = ofxArucoUtils::dictionaryFromName(dictName);
		if (dict < 0) {
			ofLogError("ofxArucoBoard") << "unknown dictionary " << dictName;
			return false;
		}
		if (t == "charuco") {
			*this = makeCharuco(j.at("squaresX"), j.at("squaresY"), j.at("squareLength"), j.at("markerLength"),
				dict, j.value("firstMarkerId", 0), j.value("legacyPattern", false));
		} else if (t == "grid") {
			*this = makeGrid(j.at("markersX"), j.at("markersY"), j.at("markerLength"), j.at("markerSeparation"),
				dict, j.value("firstMarkerId", 0));
		} else if (t == "custom") {
			std::vector<int> ids;
			std::vector<std::array<glm::vec3, 4>> corners;
			for (auto & m : j.at("markers")) {
				ids.push_back(m.at("id"));
				std::array<glm::vec3, 4> c;
				for (int k = 0; k < 4; k++) {
					auto & p = m.at("corners")[k];
					c[k] = { p[0].get<float>(), p[1].get<float>(), p[2].get<float>() };
				}
				corners.push_back(c);
			}
			*this = makeCustom(dict, ids, corners);
		} else {
			ofLogError("ofxArucoBoard") << "unknown board type " << t << " (use charuco, grid or custom)";
			return false;
		}
		name = j.value("name", getTypeName());
		return isValid();
	} catch (const std::exception & e) {
		ofLogError("ofxArucoBoard") << "invalid board json: " << e.what();
		return false;
	}
}

bool ofxArucoBoard::load(const std::string & path) {
	const std::string fullPath = ofToDataPath(path, true);
	if (!ofFile::doesFileExist(fullPath)) {
		ofLogError("ofxArucoBoard") << "file not found: " << fullPath;
		return false;
	}
	return fromJson(ofLoadJson(fullPath));
}

bool ofxArucoBoard::save(const std::string & path) const {
	return ofxArucoUtils::saveJson(path, toJson());
}

bool ofxArucoBoard::loadLegacyAruco(const std::string & path, float markerLen, int dict) {
	const std::string fullPath = ofToDataPath(path, true);
	try {
		cv::FileStorage fs(fullPath, cv::FileStorage::READ);
		if (!fs.isOpened()) {
			ofLogError("ofxArucoBoard") << "could not open " << fullPath;
			return false;
		}
		std::vector<int> ids;
		std::vector<std::array<glm::vec3, 4>> corners;
		float sidePixels = 0;
		for (auto it = fs["aruco_bc_markers"].begin(); it != fs["aruco_bc_markers"].end(); ++it) {
			cv::FileNode m = *it;
			ids.push_back((int)m["id"]);
			std::array<glm::vec3, 4> c;
			int k = 0;
			for (auto cit = m["corners"].begin(); cit != m["corners"].end() && k < 4; ++cit, ++k) {
				std::vector<float> p;
				(*cit) >> p;
				c[k] = { p[0], p[1], p.size() > 2 ? p[2] : 0.f };
			}
			sidePixels = glm::distance(c[0], c[1]);
			corners.push_back(c);
		}
		if (ids.empty() || sidePixels <= 0) {
			ofLogError("ofxArucoBoard") << "no markers in " << fullPath;
			return false;
		}
		const float scale = markerLen / sidePixels;
		for (auto & c : corners) {
			for (auto & p : c) p *= scale;
		}
		*this = makeCustom(dict, ids, corners);
		name = ofFilePath::getBaseName(fullPath);
		return isValid();
	} catch (const cv::Exception & e) {
		ofLogError("ofxArucoBoard") << "error reading " << fullPath << ": " << e.what();
		return false;
	}
}

glm::vec2 ofxArucoBoard::getSize() const {
	switch (type) {
	case Type::Charuco:
		return { countX * squareLength, countY * squareLength };
	case Type::Grid:
		return { countX * markerLength + (countX - 1) * markerSeparation,
			countY * markerLength + (countY - 1) * markerSeparation };
	case Type::Custom: {
		glm::vec2 mn(std::numeric_limits<float>::max()), mx(std::numeric_limits<float>::lowest());
		for (auto & c : customCorners) {
			for (auto & p : c) {
				mn = glm::min(mn, glm::vec2(p));
				mx = glm::max(mx, glm::vec2(p));
			}
		}
		return mx - mn;
	}
	default:
		return { 0, 0 };
	}
}

glm::vec3 ofxArucoBoard::getCenter() const {
	if (type == Type::Custom) {
		glm::vec3 sum(0);
		int n = 0;
		for (auto & c : customCorners) {
			for (auto & p : c) {
				sum += p;
				n++;
			}
		}
		return n ? sum / float(n) : sum;
	}
	return glm::vec3(getSize() * 0.5f, 0.f);
}

int ofxArucoBoard::getNumMarkers() const {
	return isValid() ? (int)board.getIds().size() : 0;
}

void ofxArucoBoard::getImage(ofPixels & pixels, int widthPixels, int marginPixels) const {
	if (!isValid()) return;
	const glm::vec2 s = getSize();
	const int w = widthPixels + 2 * marginPixels;
	const int h = int(std::round(widthPixels * s.y / s.x)) + 2 * marginPixels;
	cv::Mat img;
	board.generateImage(cv::Size(w, h), img, marginPixels, 1);
	pixels.setFromPixels(img.data, img.cols, img.rows, OF_PIXELS_GRAY);
}

void ofxArucoBoard::getImageForPrint(ofPixels & pixels, float dpi, float marginMeters) const {
	const float pixelsPerMeter = dpi / 0.0254f;
	getImage(pixels, int(std::round(getSize().x * pixelsPerMeter)), int(std::round(marginMeters * pixelsPerMeter)));
}
