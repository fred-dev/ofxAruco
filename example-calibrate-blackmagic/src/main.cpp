#include "ofMain.h"
#include "ofApp.h"

int main() {
	ofLogToConsole();
	ofGLWindowSettings settings;
	settings.setSize(1600, 900);
	settings.setGLVersion(3, 2);
	auto window = ofCreateWindow(settings);
	ofRunApp(window, std::make_shared<ofApp>());
	ofRunMainLoop();
}
