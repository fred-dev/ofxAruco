#include "IOSCamera.h"

#import <AVFoundation/AVFoundation.h>
#include <simd/simd.h>
#include <sys/sysctl.h>
#include <mutex>

// Works with and without ARC
#if __has_feature(objc_arc)
#define IOSCAM_RELEASE(x)
#else
#define IOSCAM_RELEASE(x) [(x) release]
#endif

@interface IOSCameraImpl : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate> {
@public
	AVCaptureSession * session;
	AVCaptureDevice * device;
	AVCaptureVideoDataOutput * output;
	dispatch_queue_t queue;

	std::mutex mutex;
	std::vector<unsigned char> back;
	int backWidth;
	int backHeight;
	bool hasNew;
	bool hasIntrinsics;
	float fx, fy, cx, cy;
}
- (BOOL)openDevice:(NSString *)uniqueId width:(int)width height:(int)height lensPosition:(float)lensPosition distortionCorrection:(BOOL)distortionCorrection;
- (void)close;
@end

@implementation IOSCameraImpl

- (instancetype)init {
	self = [super init];
	if (self) {
		session = nil;
		device = nil;
		output = nil;
		queue = nil;
		backWidth = backHeight = 0;
		hasNew = false;
		hasIntrinsics = false;
		fx = fy = cx = cy = 0;
	}
	return self;
}

- (void)dealloc {
	[self close];
#if !__has_feature(objc_arc)
	[super dealloc];
#endif
}

- (BOOL)openDevice:(NSString *)uniqueId width:(int)width height:(int)height lensPosition:(float)lensPosition distortionCorrection:(BOOL)distortionCorrection {
	[self close];
	AVCaptureDevice * dev = [AVCaptureDevice deviceWithUniqueID:uniqueId];
	if (dev == nil) return NO;

	// the native format of this size with the highest frame rate (full range 4:2:0 preferred)
	AVCaptureDeviceFormat * best = nil;
	double bestFps = 0;
	bool bestFull = false;
	for (AVCaptureDeviceFormat * f in dev.formats) {
		const CMVideoDimensions d = CMVideoFormatDescriptionGetDimensions(f.formatDescription);
		if (d.width != width || d.height != height) continue;
		const FourCharCode sub = CMFormatDescriptionGetMediaSubType(f.formatDescription);
		if (sub != kCVPixelFormatType_420YpCbCr8BiPlanarFullRange && sub != kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange) continue;
		double fps = 0;
		for (AVFrameRateRange * r in f.videoSupportedFrameRateRanges) fps = MAX(fps, r.maxFrameRate);
		const bool full = sub == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange;
		if (best == nil || fps > bestFps || (fps == bestFps && full && !bestFull)) {
			best = f;
			bestFps = fps;
			bestFull = full;
		}
	}
	if (best == nil) return NO;

	session = [[AVCaptureSession alloc] init];
	[session beginConfiguration];
	session.sessionPreset = AVCaptureSessionPresetInputPriority; // the device's activeFormat decides

	NSError * error = nil;
	AVCaptureDeviceInput * input = [AVCaptureDeviceInput deviceInputWithDevice:dev error:&error];
	if (input == nil || ![session canAddInput:input]) {
		[session commitConfiguration];
		[self close];
		return NO;
	}
	[session addInput:input];

	output = [[AVCaptureVideoDataOutput alloc] init];
	output.alwaysDiscardsLateVideoFrames = YES;
	output.videoSettings = @{ (id)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_420YpCbCr8BiPlanarFullRange) };
	queue = dispatch_queue_create("IOSCamera", DISPATCH_QUEUE_SERIAL);
	[output setSampleBufferDelegate:self queue:queue];
	if (![session canAddOutput:output]) {
		[session commitConfiguration];
		[self close];
		return NO;
	}
	[session addOutput:output];

	AVCaptureConnection * connection = [output connectionWithMediaType:AVMediaTypeVideo];
	if (connection != nil) {
		// stabilisation crops and moves the image: off
		if (connection.isVideoStabilizationSupported) {
			connection.preferredVideoStabilizationMode = AVCaptureVideoStabilizationModeOff;
		}
		if (@available(iOS 11.0, *)) {
			if (connection.isCameraIntrinsicMatrixDeliverySupported) {
				connection.cameraIntrinsicMatrixDeliveryEnabled = YES;
			}
		}
	}

	if ([dev lockForConfiguration:&error]) {
		dev.activeFormat = best;
		if (@available(iOS 13.0, *)) {
			if (dev.isGeometricDistortionCorrectionSupported) {
				dev.geometricDistortionCorrectionEnabled = distortionCorrection;
			}
		}
		if (lensPosition >= 0 && [dev isFocusModeSupported:AVCaptureFocusModeLocked]) {
			[dev setFocusModeLockedWithLensPosition:lensPosition completionHandler:nil];
		} else if ([dev isFocusModeSupported:AVCaptureFocusModeContinuousAutoFocus]) {
			dev.focusMode = AVCaptureFocusModeContinuousAutoFocus;
		}
		[dev unlockForConfiguration];
	}
	[session commitConfiguration];

#if __has_feature(objc_arc)
	device = dev;
#else
	device = [dev retain];
#endif
	{
		std::lock_guard<std::mutex> guard(mutex);
		hasNew = false;
		hasIntrinsics = false;
	}
	[session startRunning];
	return YES;
}

- (void)close {
	if (session != nil) {
		[session stopRunning];
		IOSCAM_RELEASE(session);
		session = nil;
	}
	if (output != nil) {
		[output setSampleBufferDelegate:nil queue:NULL];
		IOSCAM_RELEASE(output);
		output = nil;
	}
#if !__has_feature(objc_arc) && !OS_OBJECT_USE_OBJC
	if (queue != nil) dispatch_release(queue);
#elif !__has_feature(objc_arc)
	if (queue != nil) [queue release];
#endif
	queue = nil;
	if (device != nil) {
		IOSCAM_RELEASE(device);
		device = nil;
	}
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection {
	CVImageBufferRef image = CMSampleBufferGetImageBuffer(sampleBuffer);
	if (image == NULL) return;
	CVPixelBufferLockBaseAddress(image, kCVPixelBufferLock_ReadOnly);
	const size_t w = CVPixelBufferGetWidthOfPlane(image, 0);
	const size_t h = CVPixelBufferGetHeightOfPlane(image, 0);
	const size_t stride = CVPixelBufferGetBytesPerRowOfPlane(image, 0);
	const unsigned char * src = (const unsigned char *)CVPixelBufferGetBaseAddressOfPlane(image, 0);
	if (src != NULL) {
		std::lock_guard<std::mutex> guard(mutex);
		back.resize(w * h);
		for (size_t y = 0; y < h; y++) memcpy(&back[y * w], src + y * stride, w);
		backWidth = int(w);
		backHeight = int(h);
		hasNew = true;
		CFTypeRef attachment = CMGetAttachment(sampleBuffer, kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix, NULL);
		if (attachment != NULL) {
			NSData * data = (__bridge NSData *)attachment;
			if (data.length >= sizeof(matrix_float3x3)) {
				matrix_float3x3 m;
				[data getBytes:&m length:sizeof(m)];
				fx = m.columns[0][0];
				fy = m.columns[1][1];
				cx = m.columns[2][0];
				cy = m.columns[2][1];
				hasIntrinsics = true;
			}
		}
	}
	CVPixelBufferUnlockBaseAddress(image, kCVPixelBufferLock_ReadOnly);
}

@end

//--------------------------------------------------------------
namespace {
IOSCameraImpl * getImpl(void * p) {
	return (__bridge IOSCameraImpl *)p;
}
}

IOSCamera::IOSCamera() {
#if __has_feature(objc_arc)
	impl = (__bridge_retained void *)[[IOSCameraImpl alloc] init];
#else
	impl = [[IOSCameraImpl alloc] init];
#endif
}

IOSCamera::~IOSCamera() {
	close();
#if __has_feature(objc_arc)
	CFBridgingRelease(impl);
#else
	[getImpl(impl) release];
#endif
}

std::string IOSCamera::getModelIdentifier() {
	size_t size = 0;
	sysctlbyname("hw.machine", NULL, &size, NULL, 0);
	std::string model(size, '\0');
	if (size > 0) sysctlbyname("hw.machine", &model[0], &size, NULL, 0);
	while (!model.empty() && model.back() == '\0') model.pop_back();
	return model.empty() ? "iOS" : model;
}

std::vector<IOSCameraDevice> IOSCamera::listDevices() {
	std::vector<IOSCameraDevice> out;
	@autoreleasepool {
		NSMutableArray * types = [NSMutableArray arrayWithObjects:AVCaptureDeviceTypeBuiltInWideAngleCamera, AVCaptureDeviceTypeBuiltInTelephotoCamera, nil];
		if (@available(iOS 13.0, *)) {
			[types addObject:AVCaptureDeviceTypeBuiltInUltraWideCamera];
		}
		AVCaptureDeviceDiscoverySession * discovery = [AVCaptureDeviceDiscoverySession discoverySessionWithDeviceTypes:types
																							   mediaType:AVMediaTypeVideo
																								position:AVCaptureDevicePositionBack];
		for (AVCaptureDevice * dev in discovery.devices) {
			IOSCameraDevice d;
			d.uniqueId = [dev.uniqueID UTF8String];
			d.name = dev.localizedName ? [dev.localizedName UTF8String] : "camera";
			std::string lens = "Wide";
			if ([dev.deviceType isEqualToString:AVCaptureDeviceTypeBuiltInTelephotoCamera]) lens = "Telephoto";
			if (@available(iOS 13.0, *)) {
				if ([dev.deviceType isEqualToString:AVCaptureDeviceTypeBuiltInUltraWideCamera]) lens = "UltraWide";
			}
			d.lens = "Back" + lens;
			if (@available(iOS 13.0, *)) {
				d.distortionCorrectionSupported = dev.isGeometricDistortionCorrectionSupported;
			}
			for (AVCaptureDeviceFormat * f in dev.formats) {
				const FourCharCode sub = CMFormatDescriptionGetMediaSubType(f.formatDescription);
				if (sub != kCVPixelFormatType_420YpCbCr8BiPlanarFullRange && sub != kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange) continue;
				const CMVideoDimensions dim = CMVideoFormatDescriptionGetDimensions(f.formatDescription);
				float fps = 0;
				for (AVFrameRateRange * r in f.videoSupportedFrameRateRanges) fps = std::max(fps, float(r.maxFrameRate));
				auto it = std::find_if(d.formats.begin(), d.formats.end(), [&](const IOSCameraFormat & m) { return m.width == dim.width && m.height == dim.height; });
				if (it == d.formats.end()) {
					IOSCameraFormat m;
					m.width = dim.width;
					m.height = dim.height;
					m.maxFps = fps;
					m.fov = f.videoFieldOfView;
					d.formats.push_back(m);
				} else {
					it->maxFps = std::max(it->maxFps, fps);
				}
			}
			std::sort(d.formats.begin(), d.formats.end(), [](const IOSCameraFormat & a, const IOSCameraFormat & b) { return a.width * a.height > b.width * b.height; });
			out.push_back(d);
		}
	}
	return out;
}

bool IOSCamera::open(const std::string & uniqueId, int width, int height) {
	gray.clear();
	return [getImpl(impl) openDevice:[NSString stringWithUTF8String:uniqueId.c_str()]
							   width:width
							  height:height
						lensPosition:(focusLocked ? lockedLensPosition : -1.f)
				distortionCorrection:distortionCorrection];
}

void IOSCamera::close() {
	[getImpl(impl) close];
}

bool IOSCamera::isOpen() const {
	return getImpl(impl)->session != nil;
}

bool IOSCamera::update() {
	IOSCameraImpl * c = getImpl(impl);
	std::lock_guard<std::mutex> guard(c->mutex);
	if (!c->hasNew) return false;
	gray.setFromPixels(c->back.data(), c->backWidth, c->backHeight, OF_PIXELS_GRAY);
	c->hasNew = false;
	return true;
}

bool IOSCamera::lockFocus() {
	AVCaptureDevice * dev = getImpl(impl)->device;
	if (dev == nil || ![dev isFocusModeSupported:AVCaptureFocusModeLocked]) return false;
	return setLensPosition(dev.lensPosition);
}

bool IOSCamera::setLensPosition(float position) {
	AVCaptureDevice * dev = getImpl(impl)->device;
	if (dev == nil || ![dev isFocusModeSupported:AVCaptureFocusModeLocked]) return false;
	NSError * error = nil;
	if (![dev lockForConfiguration:&error]) return false;
	[dev setFocusModeLockedWithLensPosition:position completionHandler:nil];
	[dev unlockForConfiguration];
	focusLocked = true;
	lockedLensPosition = position;
	return true;
}

void IOSCamera::unlockFocus() {
	focusLocked = false;
	lockedLensPosition = -1;
	AVCaptureDevice * dev = getImpl(impl)->device;
	NSError * error = nil;
	if (dev != nil && [dev isFocusModeSupported:AVCaptureFocusModeContinuousAutoFocus] && [dev lockForConfiguration:&error]) {
		dev.focusMode = AVCaptureFocusModeContinuousAutoFocus;
		[dev unlockForConfiguration];
	}
}

float IOSCamera::getLensPosition() const {
	AVCaptureDevice * dev = getImpl(impl)->device;
	return dev != nil ? dev.lensPosition : lockedLensPosition;
}

bool IOSCamera::isDistortionCorrectionSupported() const {
	AVCaptureDevice * dev = getImpl(impl)->device;
	if (dev == nil) return false;
	if (@available(iOS 13.0, *)) {
		return dev.isGeometricDistortionCorrectionSupported;
	}
	return false;
}

bool IOSCamera::hasReportedIntrinsics() const {
	IOSCameraImpl * c = getImpl(impl);
	std::lock_guard<std::mutex> guard(c->mutex);
	return c->hasIntrinsics;
}

glm::vec4 IOSCamera::getReportedIntrinsics() const {
	IOSCameraImpl * c = getImpl(impl);
	std::lock_guard<std::mutex> guard(c->mutex);
	return glm::vec4(c->fx, c->fy, c->cx, c->cy);
}
