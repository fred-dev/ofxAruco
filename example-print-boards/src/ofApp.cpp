#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowTitle("ofxAruco - print boards");
	ofBackground(40);

	params.add(type, typeName, dictionary, dictionaryName, countX, countY, squareMm, markerMm, separationMm, firstId, dpi, marginMm);
	typeName.setSerializable(false);
	dictionaryName.setSerializable(false);
	gui.setDefaultWidth(300);
	gui.setup(params, "settings.json", 10, 10);
	gui.add(saveButton.setup("save png + json  (s)"));
	if (ofFile::doesFileExist("settings.json")) gui.loadFromFile("settings.json");

	// regenerate when anything changes
	listeners.push(params.parameterChangedE().newListener([this](ofAbstractParameter & p) {
		if (p.getName() != typeName.getName() && p.getName() != dictionaryName.getName()) dirty = true;
	}));
	saveButton.addListener(this, &ofApp::save);

	// the self-check detector: main thread, results immediately
	checker.setup(dictionary, false);
}

void ofApp::rebuild() {
	dirty = false;
	typeName.set(type == 0 ? "ChArUco board" : type == 1 ? "grid board" : "single marker");
	dictionaryName.set(ofxArucoUtils::dictionaryToName(dictionary));

	const float sq = squareMm / 1000.f, mk = markerMm / 1000.f, sep = separationMm / 1000.f;
	if (type == 0 && mk >= sq) {
		status = "the marker must be smaller than the square";
		pixels.clear();
		return;
	}
	if (type == 0) {
		board = ofxArucoBoard::makeCharuco(countX, countY, sq, mk, dictionary, firstId);
	} else if (type == 1) {
		board = ofxArucoBoard::makeGrid(countX, countY, mk, sep, dictionary, firstId);
	} else {
		board = ofxArucoBoard::makeGrid(1, 1, mk, sep, dictionary, firstId);
	}
	board.setName(getFileName());
	if (!board.isValid()) {
		status = "invalid board (too many markers for this dictionary?)";
		pixels.clear();
		return;
	}
	board.getImageForPrint(pixels, dpi, marginMm / 1000.f);
	preview.loadData(pixels);

	// self-check: detect the generated image
	checker.dictionary = dictionary;
	checker.detect(pixels);
	const glm::vec2 size = board.getSize() * 100.f; // cm
	const float paperW = size.x + 2 * marginMm / 10.f, paperH = size.y + 2 * marginMm / 10.f;
	auto fits = [&](float w, float h) { return (paperW <= w && paperH <= h) || (paperW <= h && paperH <= w); };
	std::stringstream ss;
	ss << typeName.get() << ", " << board.getNumMarkers() << " markers (ids " << firstId << " - " << firstId + board.getNumMarkers() - 1 << ")\n"
	   << "printed size: " << ofToString(size.x, 1) << " x " << ofToString(size.y, 1) << " cm (+ margin)\n"
	   << "image: " << pixels.getWidth() << " x " << pixels.getHeight() << " px at " << dpi << " dpi\n"
	   << "fits on: " << (fits(21.0f, 29.7f) ? "A4 " : "") << (fits(21.6f, 27.9f) ? "Letter " : "") << (fits(29.7f, 42.0f) ? "A3 " : "")
	   << (fits(42.0f, 59.4f) ? "A2" : "") << "\n"
	   << "self-check: " << checker.getNumMarkers() << " / " << board.getNumMarkers() << " markers detected"
	   << (checker.getNumMarkers() == board.getNumMarkers() ? "  OK" : "  (check the sizes)");
	status = ss.str();
}

std::string ofApp::getFileName() const {
	const std::string dict = ofxArucoUtils::dictionaryToName(dictionary);
	if (type == 0) {
		return "charuco_" + ofToString(countX.get()) + "x" + ofToString(countY.get()) + "_" + ofToString(squareMm.get(), 0) + "mm_"
			+ ofToString(markerMm.get(), 0) + "mm_" + dict + "_id" + ofToString(firstId.get());
	}
	if (type == 1) {
		return "grid_" + ofToString(countX.get()) + "x" + ofToString(countY.get()) + "_" + ofToString(markerMm.get(), 0) + "mm_"
			+ ofToString(separationMm.get(), 0) + "mm_" + dict + "_id" + ofToString(firstId.get());
	}
	return "marker_" + ofToString(firstId.get()) + "_" + ofToString(markerMm.get(), 0) + "mm_" + dict;
}

void ofApp::save() {
	if (!pixels.isAllocated()) return;
	const std::string name = getFileName();
	ofxArucoUtils::savePngWithDpi(pixels, name + ".png", dpi); // prints at the right size at 100%
	board.save(name + ".json");
	gui.saveToFile("settings.json");
	ofLogNotice() << "saved " << ofToDataPath(name + ".png", true) << " and " << name << ".json";
	status += "\nsaved bin/data/" + name + ".png + .json";
}

//--------------------------------------------------------------
void ofApp::update() {
	if (dirty) rebuild();
}

void ofApp::draw() {
	// preview, fitted to the free area
	ofRectangle area(330, 10, ofGetWidth() - 340, ofGetHeight() - 120);
	if (preview.isAllocated() && pixels.isAllocated()) {
		ofRectangle r(0, 0, preview.getWidth(), preview.getHeight());
		r.scaleTo(area);
		ofSetColor(255);
		preview.draw(r);
		checker.drawMarkers(r); // the self-check detections on top
	}
	ofDrawBitmapStringHighlight(status, 330, ofGetHeight() - 95);
	ofDrawBitmapStringHighlight("Print at 100% scale, then measure and fix the sizes in the .json if needed.", 330, ofGetHeight() - 15);
	gui.draw();
}

void ofApp::keyPressed(int key) {
	if (key == 's') save();
}
