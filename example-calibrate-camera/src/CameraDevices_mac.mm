#include "CameraDevices.h"

#ifdef TARGET_OSX
#import <AVFoundation/AVFoundation.h>

// Same device list and order as ofAVFoundationGrabber (keep in sync with
// openFrameworks/video/ofAVFoundationGrabber.mm).
static NSArray * listAVDevices() {
	NSArray * devices;
	if (@available(macOS 10.15, *)) {
		NSMutableArray * deviceTypes = [NSMutableArray arrayWithObject:AVCaptureDeviceTypeBuiltInWideAngleCamera];
#if __MAC_OS_X_VERSION_MAX_ALLOWED >= 140000
		if (@available(macOS 14.0, *)) {
			if (&AVCaptureDeviceTypeExternal != nil) {
				[deviceTypes addObject:AVCaptureDeviceTypeExternal];
				[deviceTypes addObject:AVCaptureDeviceTypeContinuityCamera];
			}
		}
#endif
		AVCaptureDeviceDiscoverySession * session = [AVCaptureDeviceDiscoverySession
			discoverySessionWithDeviceTypes:deviceTypes
								  mediaType:AVMediaTypeVideo
								   position:AVCaptureDevicePositionUnspecified];
		devices = [session devices];
	} else {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
		devices = [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo];
#pragma clang diagnostic pop
	}
	if ([devices count] > 1) {
		devices = [devices sortedArrayUsingComparator:^NSComparisonResult(AVCaptureDevice * d1, AVCaptureDevice * d2) {
			NSString * name1 = d1.localizedName;
			NSString * name2 = d2.localizedName;
			BOOL isFaceTime1 = [name1 hasPrefix:@"FaceTime"];
			BOOL isFaceTime2 = [name2 hasPrefix:@"FaceTime"];
			if (isFaceTime1 && !isFaceTime2) return NSOrderedAscending;
			if (!isFaceTime1 && isFaceTime2) return NSOrderedDescending;
			return [name1 compare:name2];
		}];
	}
	return devices;
}

std::vector<CameraDevice> listCameraDevices() {
	std::vector<CameraDevice> out;
	@autoreleasepool {
		NSArray * devices = listAVDevices();
		int index = 0;
		for (AVCaptureDevice * device in devices) {
			CameraDevice c;
			c.grabberIndex = index++;
			c.name = device.localizedName ? [device.localizedName UTF8String] : "camera";
			c.uniqueId = device.uniqueID ? [device.uniqueID UTF8String] : c.name;
			c.model = device.modelID ? [device.modelID UTF8String] : "";
			for (AVCaptureDeviceFormat * format in device.formats) {
				const CMVideoDimensions d = CMVideoFormatDescriptionGetDimensions(format.formatDescription);
				float fps = 0;
				for (AVFrameRateRange * range in format.videoSupportedFrameRateRanges) fps = std::max(fps, float(range.maxFrameRate));
				auto it = std::find_if(c.modes.begin(), c.modes.end(), [&](const ofxArucoCameraMode & m) { return m.width == d.width && m.height == d.height; });
				if (it == c.modes.end()) {
					c.modes.push_back({ int(d.width), int(d.height), fps });
				} else {
					it->fps = std::max(it->fps, fps);
				}
			}
			std::sort(c.modes.begin(), c.modes.end(), [](const ofxArucoCameraMode & a, const ofxArucoCameraMode & b) { return a.width * a.height > b.width * b.height; });
			out.push_back(c);
		}
	}
	return out;
}
#endif
