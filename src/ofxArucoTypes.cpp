#include "ofxArucoTypes.h"
#include <opencv2/calib3d.hpp>
#include <regex>

namespace {
	struct DictEntry { const char * name; int id; };
	const DictEntry kDictionaries[] = {
		{ "DICT_4X4_50", cv::aruco::DICT_4X4_50 },
		{ "DICT_4X4_100", cv::aruco::DICT_4X4_100 },
		{ "DICT_4X4_250", cv::aruco::DICT_4X4_250 },
		{ "DICT_4X4_1000", cv::aruco::DICT_4X4_1000 },
		{ "DICT_5X5_50", cv::aruco::DICT_5X5_50 },
		{ "DICT_5X5_100", cv::aruco::DICT_5X5_100 },
		{ "DICT_5X5_250", cv::aruco::DICT_5X5_250 },
		{ "DICT_5X5_1000", cv::aruco::DICT_5X5_1000 },
		{ "DICT_6X6_50", cv::aruco::DICT_6X6_50 },
		{ "DICT_6X6_100", cv::aruco::DICT_6X6_100 },
		{ "DICT_6X6_250", cv::aruco::DICT_6X6_250 },
		{ "DICT_6X6_1000", cv::aruco::DICT_6X6_1000 },
		{ "DICT_7X7_50", cv::aruco::DICT_7X7_50 },
		{ "DICT_7X7_100", cv::aruco::DICT_7X7_100 },
		{ "DICT_7X7_250", cv::aruco::DICT_7X7_250 },
		{ "DICT_7X7_1000", cv::aruco::DICT_7X7_1000 },
		{ "DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL },
		{ "DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5 },
		{ "DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9 },
		{ "DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10 },
		{ "DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11 },
		{ "DICT_ARUCO_MIP_36h12", cv::aruco::DICT_ARUCO_MIP_36h12 },
	};
}

namespace ofxArucoUtils {

const std::vector<std::string> & getDictionaryNames() {
	static std::vector<std::string> names = [] {
		std::vector<std::string> n;
		for (auto & d : kDictionaries) n.push_back(d.name);
		return n;
	}();
	return names;
}

int dictionaryFromName(const std::string & name) {
	for (auto & d : kDictionaries) {
		if (name == d.name) return d.id;
	}
	return -1;
}

std::string dictionaryToName(int dictionary) {
	for (auto & d : kDictionaries) {
		if (dictionary == d.id) return d.name;
	}
	return "";
}

cv::aruco::Dictionary getDictionary(int dictionary) {
	if (dictionaryToName(dictionary).empty()) {
		ofLogWarning("ofxAruco") << "unknown dictionary " << dictionary << ", using DICT_4X4_50";
		dictionary = cv::aruco::DICT_4X4_50;
	}
	return cv::aruco::getPredefinedDictionary(dictionary);
}

glm::mat4 toMat4(const cv::Vec3d & rvec, const cv::Vec3d & tvec) {
	cv::Matx33d R;
	cv::Rodrigues(rvec, R);
	glm::mat4 m(1.f);
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			m[c][r] = float(R(r, c)); // glm is column-major: m[col][row]
		}
		m[3][r] = float(tvec[r]);
	}
	return m;
}

void fromMat4(const glm::mat4 & m, cv::Vec3d & rvec, cv::Vec3d & tvec) {
	cv::Matx33d R;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			R(r, c) = m[c][r];
		}
		tvec[r] = m[3][r];
	}
	cv::Rodrigues(R, rvec);
}

glm::mat4 cvToGl() {
	return glm::scale(glm::mat4(1.f), glm::vec3(1.f, -1.f, -1.f));
}

glm::mat4 rigidInverse(const glm::mat4 & m) {
	glm::mat3 Rt = glm::transpose(glm::mat3(m));
	glm::mat4 inv(Rt);
	inv[3] = glm::vec4(-(Rt * glm::vec3(m[3])), 1.f);
	return inv;
}

float rotationAngleDeg(const glm::mat4 & a, const glm::mat4 & b) {
	// relative rotation, in double; atan2 stays precise for tiny angles (acos doesn't)
	const glm::dmat3 d = glm::transpose(glm::dmat3(glm::mat3(a))) * glm::dmat3(glm::mat3(b));
	const double s = glm::length(glm::dvec3(d[1][2] - d[2][1], d[2][0] - d[0][2], d[0][1] - d[1][0])); // 2 sin
	const double c = d[0][0] + d[1][1] + d[2][2] - 1.0;                                                // 2 cos
	return float(glm::degrees(std::atan2(s, c)));
}

ofJson toJson(const glm::mat4 & m) {
	ofJson rows = ofJson::array();
	for (int r = 0; r < 4; r++) {
		rows.push_back({ m[0][r], m[1][r], m[2][r], m[3][r] });
	}
	return rows;
}

bool saveJson(const std::string & path, const ofJson & json) {
	const std::string pretty = json.dump(4);
	// collapse every array that only contains numbers: [\n 1.0,\n 2.0\n] -> [1.0, 2.0]
	static const std::regex numberArray(R"(\[\s*(-?[0-9][0-9.eE+-]*(\s*,\s*-?[0-9][0-9.eE+-]*)*)\s*\])");
	static const std::regex separators(R"(\s*,\s*)");
	std::string out;
	auto begin = std::sregex_iterator(pretty.begin(), pretty.end(), numberArray);
	size_t last = 0;
	for (auto it = begin; it != std::sregex_iterator(); ++it) {
		out += pretty.substr(last, it->position() - last);
		out += "[" + std::regex_replace((*it)[1].str(), separators, ", ") + "]";
		last = it->position() + it->length();
	}
	out += pretty.substr(last);
	return ofBufferToFile(ofToDataPath(path, true), ofBuffer(out.data(), out.size()));
}

bool savePngWithDpi(const ofPixels & pixels, const std::string & path, float dpi) {
	ofBuffer png;
	if (!ofSaveImage(pixels, png, OF_IMAGE_FORMAT_PNG)) return false;
	const std::string in(png.getData(), png.size());
	auto be32 = [](uint32_t v) {
		return std::string { char(v >> 24), char((v >> 16) & 0xff), char((v >> 8) & 0xff), char(v & 0xff) };
	};
	auto readBe32 = [&](size_t pos) {
		return (uint32_t((unsigned char)in[pos]) << 24) | (uint32_t((unsigned char)in[pos + 1]) << 16)
			| (uint32_t((unsigned char)in[pos + 2]) << 8) | uint32_t((unsigned char)in[pos + 3]);
	};
	// our pHYs chunk (physical pixel size), CRC over type + data
	const uint32_t pixelsPerMeter = uint32_t(std::round(dpi / 0.0254f));
	const std::string phys = std::string("pHYs") + be32(pixelsPerMeter) + be32(pixelsPerMeter) + char(1); // unit: meter
	uint32_t crc = 0xffffffffu;
	for (unsigned char c : phys) {
		crc ^= c;
		for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
	}
	crc ^= 0xffffffffu;

	// PNG = 8 byte signature + chunks (length, type, data, crc). Copy every chunk,
	// dropping any existing pHYs and inserting ours right after IHDR.
	if (in.size() < 8) return false;
	std::string out = in.substr(0, 8);
	size_t pos = 8;
	while (pos + 12 <= in.size()) {
		const uint32_t len = readBe32(pos);
		const std::string type = in.substr(pos + 4, 4);
		const size_t total = 12 + size_t(len);
		if (pos + total > in.size()) return false;
		if (type != "pHYs") out += in.substr(pos, total);
		if (type == "IHDR") out += be32(9) + phys + be32(crc);
		pos += total;
	}
	return ofBufferToFile(ofToDataPath(path, true), ofBuffer(out.data(), out.size()), true);
}

glm::mat4 mat4FromJson(const ofJson & j) {
	glm::mat4 m(1.f);
	if (!j.is_array() || j.size() != 4) return m;
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			m[c][r] = j[r][c].get<float>();
		}
	}
	return m;
}

}
