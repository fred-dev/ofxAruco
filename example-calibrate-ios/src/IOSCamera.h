#pragma once

// iOS camera capture for calibration (AVFoundation), with the control calibration needs
// and ofVideoGrabber doesn't give:
//   - every physical back lens (wide, ultra wide, telephoto), never the "virtual"
//     multi-lens cameras that switch lenses by themselves
//   - every native format of a lens (size, frame rate, field of view)
//   - focus locked at a lens position (the intrinsics change with focus)
//   - video stabilisation off, geometric distortion correction on or off
//   - the intrinsic matrix iOS itself reports per frame, for comparison
// Frames are delivered as 8 bit luma (the Y plane), which is all calibration needs.
// The image is in the sensor's native (landscape) orientation, as openFrameworks' iOS
// video grabber delivers it: calibrations are valid for frames in that orientation.

#include "ofMain.h"

struct IOSCameraFormat {
	int width = 0;
	int height = 0;
	float maxFps = 0;
	float fov = 0; // horizontal field of view (degrees) as reported by iOS
};

struct IOSCameraDevice {
	std::string uniqueId;
	std::string name;   // "Back Camera", "Back Ultra Wide Camera"...
	std::string lens;   // file-safe: "BackWide", "BackUltraWide", "BackTelephoto"
	bool distortionCorrectionSupported = false;
	std::vector<IOSCameraFormat> formats; // unique sizes, largest first
};

class IOSCamera {
public:
	IOSCamera();
	~IOSCamera();

	static std::vector<IOSCameraDevice> listDevices();
	// "iPhone16,1" (the hardware model; the device name is private since iOS 16)
	static std::string getModelIdentifier();

	// Opens a lens in its native format of this size (the highest frame rate).
	bool open(const std::string & uniqueId, int width, int height);
	void close();
	bool isOpen() const;

	bool update(); // true when a new frame arrived
	const ofPixels & getGray() const { return gray; }
	int getWidth() const { return gray.getWidth(); }
	int getHeight() const { return gray.getHeight(); }

	// Focus: auto until locked. lockFocus() locks at the current lens position;
	// setLensPosition() locks at a given one (0 near .. 1 far, as AVFoundation).
	// A locked position is kept when another format is opened.
	bool lockFocus();
	bool setLensPosition(float position);
	void unlockFocus();
	bool isFocusLocked() const { return focusLocked; }
	float getLensPosition() const;

	// Geometric distortion correction (iOS straightens ultra wide images by default).
	// Calibrate with the setting your app will use. Applied when a format is opened.
	void setDistortionCorrection(bool enabled) { distortionCorrection = enabled; }
	bool getDistortionCorrection() const { return distortionCorrection; }
	bool isDistortionCorrectionSupported() const;

	// The intrinsic matrix iOS reports for the current frames (fx, fy, cx, cy in pixels).
	bool hasReportedIntrinsics() const;
	glm::vec4 getReportedIntrinsics() const;

private:
	void * impl; // IOSCameraImpl (Objective-C)
	ofPixels gray;
	bool focusLocked = false;
	float lockedLensPosition = -1;
	bool distortionCorrection = false;
};
