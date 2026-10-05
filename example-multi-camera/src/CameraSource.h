#pragma once

// A camera for the calibration: anything that gives images + intrinsics.
// To calibrate other devices (e.g. Azure Kinects), implement this interface:
// return the color (or IR) pixels and the factory intrinsics of that stream.

#include "ofMain.h"
#include "ofxAruco.h"

class CameraSource {
public:
	virtual ~CameraSource() = default;
	virtual void update() = 0;
	virtual bool isFrameNew() const = 0;
	virtual const ofPixels & getPixels() const = 0;
	virtual void draw(const ofRectangle & rect) const = 0;
	virtual const ofxArucoIntrinsics & getIntrinsics() const = 0;
	virtual std::string getName() const = 0;
	// true when the intrinsics are a guess (no calibration file): poses will be approximate
	virtual bool hasEstimatedIntrinsics() const { return false; }
};

// A webcam through ofVideoGrabber.
class WebcamSource : public CameraSource {
public:
	// intrinsicsFile: OpenCV .yml / .json calibration of this webcam at any resolution.
	// If it doesn't exist the intrinsics are estimated from the horizontal field of view.
	bool setup(int deviceId, int width, int height, const std::string & intrinsicsFile, float hfovDeg) {
		grabber.setDeviceID(deviceId);
		if (!grabber.setup(width, height)) return false;
		name = "webcam " + ofToString(deviceId);
		const int w = int(grabber.getWidth()), h = int(grabber.getHeight());
		if (!intrinsicsFile.empty() && ofFile::doesFileExist(intrinsicsFile) && intrinsics.load(intrinsicsFile)) {
			if (intrinsics.getWidth() != w) intrinsics = intrinsics.getScaled(w, h);
		} else {
			const double f = (w * 0.5) / std::tan(glm::radians(hfovDeg) * 0.5);
			intrinsics.setup(f, f, w * 0.5 - 0.5, h * 0.5 - 0.5, w, h);
			estimated = true;
			ofLogWarning("WebcamSource") << name << ": no intrinsics file '" << intrinsicsFile << "', estimating from a "
										 << hfovDeg << " deg field of view. Poses will be approximate.";
		}
		return true;
	}
	void update() override { grabber.update(); }
	bool isFrameNew() const override { return grabber.isFrameNew(); }
	const ofPixels & getPixels() const override { return grabber.getPixels(); }
	void draw(const ofRectangle & r) const override { grabber.draw(r); }
	const ofxArucoIntrinsics & getIntrinsics() const override { return intrinsics; }
	std::string getName() const override { return name; }
	bool hasEstimatedIntrinsics() const override { return estimated; }

private:
	mutable ofVideoGrabber grabber;
	ofxArucoIntrinsics intrinsics;
	std::string name;
	bool estimated = false;
};
