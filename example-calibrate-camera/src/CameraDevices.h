#pragma once

// Lists the connected cameras and their native capture modes (resolutions),
// in the same order ofVideoGrabber uses, so grabberIndex can go straight to
// ofVideoGrabber::setDeviceID().
//
//   macOS:   AVFoundation (CameraDevices_mac.mm), same device order as ofAVFoundationGrabber
//   Windows: DirectShow (CameraDevices_win.cpp), same device order as ofDirectShowGrabber

#include "ofMain.h"
#include "ofxAruco.h"

struct CameraDevice {
	std::string name;
	std::string uniqueId;  // stable id: macOS uniqueID, Windows device path
	std::string model;
	int grabberIndex = -1; // for ofVideoGrabber::setDeviceID()
	std::string label;     // file-safe name, see assignLabels()
	std::vector<ofxArucoCameraMode> modes; // unique sizes, largest first
};

std::vector<CameraDevice> listCameraDevices();

// File-safe labels: the camera name ("Logitech BRIO" -> "Logitech_BRIO"). Only when two
// connected cameras have the same name, a short id is added to tell them apart
// ("Logitech_BRIO_1a2b3c4d"). Note: on macOS and Windows a USB camera's unique id
// usually depends on the USB port, so it is not a reliable identity anyway.
void assignLabels(std::vector<CameraDevice> & cameras);
// Short id (FNV-1a of uniqueId, 8 hex digits).
std::string shortCameraId(const CameraDevice & camera);
// calibrations/<label>  (files inside: <label>_<width>x<height>.yml)
std::string calibrationFolder(const CameraDevice & camera);
