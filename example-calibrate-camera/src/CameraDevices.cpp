#include "CameraDevices.h"

std::string shortCameraId(const CameraDevice & camera) {
	uint32_t h = 2166136261u;
	for (unsigned char c : camera.uniqueId) {
		h ^= c;
		h *= 16777619u;
	}
	char buf[16];
	std::snprintf(buf, sizeof(buf), "%08x", h);
	return buf;
}

static std::string safeName(const std::string & name) {
	std::string safe;
	for (char c : ofTrim(name)) safe += (std::isalnum((unsigned char)c) || c == '-') ? c : '_';
	return safe.empty() ? "camera" : safe;
}

void assignLabels(std::vector<CameraDevice> & cameras) {
	for (auto & c : cameras) {
		c.label = safeName(c.name);
		const int sameName = int(std::count_if(cameras.begin(), cameras.end(), [&](const CameraDevice & o) { return o.name == c.name; }));
		if (sameName > 1) c.label += "_" + shortCameraId(c);
	}
}

std::string calibrationFolder(const CameraDevice & camera) {
	return "calibrations/" + (camera.label.empty() ? safeName(camera.name) : camera.label);
}

#if !defined(TARGET_OSX) && !defined(TARGET_WIN32)
// other platforms: device names only, no mode list
std::vector<CameraDevice> listCameraDevices() {
	std::vector<CameraDevice> out;
	ofVideoGrabber grabber;
	for (auto & d : grabber.listDevices()) {
		CameraDevice c;
		c.name = d.deviceName;
		c.uniqueId = d.serialID.empty() ? d.deviceName : d.serialID;
		c.grabberIndex = d.id;
		for (auto & f : d.formats) c.modes.push_back({ f.width, f.height, f.framerates.empty() ? 0.f : f.framerates.back() });
		out.push_back(c);
	}
	return out;
}
#endif
