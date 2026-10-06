#include "CameraDevices.h"

#ifdef TARGET_WIN32
// DirectShow enumeration: the same device category and order that
// ofDirectShowGrabber (videoInput) uses, so the index matches setDeviceID().
#include <windows.h>
#include <dshow.h>
#include <dvdmedia.h> // VIDEOINFOHEADER2

#pragma comment(lib, "strmiids.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "oleaut32.lib")

namespace {
std::string narrow(const wchar_t * w) {
	if (!w) return "";
	const int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
	if (n <= 1) return "";
	std::string s(n - 1, '\0');
	WideCharToMultiByte(CP_UTF8, 0, w, -1, &s[0], n, nullptr, nullptr);
	return s;
}

void freeMediaType(AM_MEDIA_TYPE * mt) {
	if (!mt) return;
	if (mt->cbFormat != 0) CoTaskMemFree(mt->pbFormat);
	if (mt->pUnk) mt->pUnk->Release();
	CoTaskMemFree(mt);
}

bool isCapturePin(IPin * pin) {
	PIN_DIRECTION dir;
	if (FAILED(pin->QueryDirection(&dir)) || dir != PINDIR_OUTPUT) return false;
	IKsPropertySet * ks = nullptr;
	if (FAILED(pin->QueryInterface(IID_PPV_ARGS(&ks)))) return false;
	GUID category = GUID_NULL;
	DWORD returned = 0;
	const bool ok = SUCCEEDED(ks->Get(AMPROPSETID_Pin, AMPROPERTY_PIN_CATEGORY, nullptr, 0, &category, sizeof(category), &returned));
	ks->Release();
	return ok && category == PIN_CATEGORY_CAPTURE;
}

void addMode(CameraDevice & c, int w, int h, float fps) {
	if (w <= 0 || h <= 0) return;
	auto it = std::find_if(c.modes.begin(), c.modes.end(), [&](const ofxArucoCameraMode & m) { return m.width == w && m.height == h; });
	if (it == c.modes.end()) {
		c.modes.push_back({ w, h, fps });
	} else {
		it->fps = std::max(it->fps, fps);
	}
}

void readModes(IBaseFilter * filter, CameraDevice & c) {
	IEnumPins * pins = nullptr;
	if (FAILED(filter->EnumPins(&pins))) return;
	IPin * pin = nullptr;
	while (pins->Next(1, &pin, nullptr) == S_OK) {
		if (isCapturePin(pin)) {
			IAMStreamConfig * config = nullptr;
			if (SUCCEEDED(pin->QueryInterface(IID_PPV_ARGS(&config)))) {
				int count = 0, size = 0;
				if (SUCCEEDED(config->GetNumberOfCapabilities(&count, &size)) && size == sizeof(VIDEO_STREAM_CONFIG_CAPS)) {
					for (int i = 0; i < count; i++) {
						AM_MEDIA_TYPE * mt = nullptr;
						VIDEO_STREAM_CONFIG_CAPS caps;
						if (FAILED(config->GetStreamCaps(i, &mt, reinterpret_cast<BYTE *>(&caps))) || !mt) continue;
						if (mt->formattype == FORMAT_VideoInfo && mt->cbFormat >= sizeof(VIDEOINFOHEADER) && mt->pbFormat) {
							auto * vih = reinterpret_cast<VIDEOINFOHEADER *>(mt->pbFormat);
							addMode(c, vih->bmiHeader.biWidth, std::abs(vih->bmiHeader.biHeight),
								vih->AvgTimePerFrame > 0 ? float(1e7 / double(vih->AvgTimePerFrame)) : 0.f);
						} else if (mt->formattype == FORMAT_VideoInfo2 && mt->cbFormat >= sizeof(VIDEOINFOHEADER2) && mt->pbFormat) {
							auto * vih = reinterpret_cast<VIDEOINFOHEADER2 *>(mt->pbFormat);
							addMode(c, vih->bmiHeader.biWidth, std::abs(vih->bmiHeader.biHeight),
								vih->AvgTimePerFrame > 0 ? float(1e7 / double(vih->AvgTimePerFrame)) : 0.f);
						}
						freeMediaType(mt);
					}
				}
				config->Release();
			}
		}
		pin->Release();
	}
	pins->Release();
}
}

std::vector<CameraDevice> listCameraDevices() {
	std::vector<CameraDevice> out;
	// COM may already be initialized by openFrameworks (any mode is fine for enumeration)
	const HRESULT init = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
	const bool mustUninit = SUCCEEDED(init);

	ICreateDevEnum * devEnum = nullptr;
	if (SUCCEEDED(CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&devEnum)))) {
		IEnumMoniker * monikers = nullptr;
		if (devEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &monikers, 0) == S_OK) {
			IMoniker * moniker = nullptr;
			int index = 0;
			while (monikers->Next(1, &moniker, nullptr) == S_OK) {
				CameraDevice c;
				c.grabberIndex = index++;
				IPropertyBag * bag = nullptr;
				if (SUCCEEDED(moniker->BindToStorage(nullptr, nullptr, IID_PPV_ARGS(&bag)))) {
					VARIANT v;
					VariantInit(&v);
					if (SUCCEEDED(bag->Read(L"FriendlyName", &v, nullptr)) && v.vt == VT_BSTR) c.name = narrow(v.bstrVal);
					VariantClear(&v);
					if (SUCCEEDED(bag->Read(L"DevicePath", &v, nullptr)) && v.vt == VT_BSTR) c.uniqueId = narrow(v.bstrVal);
					VariantClear(&v);
					bag->Release();
				}
				if (c.name.empty()) c.name = "camera " + ofToString(c.grabberIndex);
				if (c.uniqueId.empty()) c.uniqueId = c.name;
				IBaseFilter * filter = nullptr;
				if (SUCCEEDED(moniker->BindToObject(nullptr, nullptr, IID_PPV_ARGS(&filter)))) {
					readModes(filter, c);
					filter->Release();
				}
				std::sort(c.modes.begin(), c.modes.end(), [](const ofxArucoCameraMode & a, const ofxArucoCameraMode & b) { return a.width * a.height > b.width * b.height; });
				out.push_back(c);
				moniker->Release();
			}
			monikers->Release();
		}
		devEnum->Release();
	}
	if (mustUninit) CoUninitialize();
	return out;
}
#endif
