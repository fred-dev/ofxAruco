#include "ofMain.h"
#include "ofApp.h"

int main() {
	// Workaround: with some openFrameworks nightlies + Visual Studio 2026 the default
	// logger channel is null and the first ofLog() crashes. Setting it explicitly is harmless.
	ofLogToConsole();

	ofGLWindowSettings settings;
	settings.setSize(1200, 800);
	settings.setGLVersion(3, 2);
	auto window = ofCreateWindow(settings);
	ofRunApp(window, std::make_shared<ofApp>());
	ofRunMainLoop();
}
