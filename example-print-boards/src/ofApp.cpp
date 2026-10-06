#include "ofApp.h"

namespace {
struct Paper {
	const char * name;
	float w, h; // mm, portrait
};
const Paper papers[] = {
	{ "A4", 210, 297 },
	{ "A3", 297, 420 },
	{ "A2", 420, 594 },
	{ "A1", 594, 841 },
	{ "A0", 841, 1189 },
	{ "Letter", 215.9f, 279.4f },
};
const int numPapers = sizeof(papers) / sizeof(papers[0]);
const int A4 = 0, A1 = 3;

const char * typeNames[] = { "ChArUco board", "grid board", "single marker", "marker cards (cut out)" };
}

//--------------------------------------------------------------
// The presets. Sizes are chosen to fit the paper with a 10 mm margin.
// Cards use the same counts and marker sizes as the boards above them, with
// gaps wide enough to leave a white border around every cut-out marker.
// They start at id 20 so they don't clash with the 4x3 and 7x5 boards.
const std::vector<ofApp::Values> & ofApp::getPresets() {
	//                     name                         type          paper cx  cy  square marker sep  id   dpi  margin
	static const std::vector<Values> presets = {
		{ "A4 ChArUco 4x3 (60 mm squares)",  Charuco,      A4,   4,  3,  60,   45,    10,  0,   300, 10 },
		{ "A4 ChArUco 7x5 (36 mm squares)",  Charuco,      A4,   7,  5,  36,   27,    10,  0,   300, 10 },
		{ "A4 ChArUco 10x8 (23 mm squares)", Charuco,      A4,   10, 8,  23,   17,    10,  0,   300, 10 },
		{ "A4 cards 4x3 (45 mm markers)",    Cards,        A4,   4,  3,  60,   45,    20,  20,  300, 10 },
		{ "A4 cards 7x5 (27 mm markers)",    Cards,        A4,   7,  5,  36,   27,    12,  20,  300, 10 },
		{ "A4 cards 10x8 (17 mm markers)",   Cards,        A4,   10, 8,  23,   17,    7,   20,  300, 10 },
		{ "A1 single marker (500 mm)",       SingleMarker, A1,   1,  1,  600,  500,   10,  0,   150, 40 },
	};
	return presets;
}

ofApp::Values ofApp::capture() const {
	return { "user", type, paper, countX, countY, squareMm, markerMm, separationMm, firstId, dpi, marginMm };
}

void ofApp::apply(const Values & v) {
	type = v.type;
	paper = v.paper;
	countX = v.countX;
	countY = v.countY;
	squareMm = v.squareMm;
	markerMm = v.markerMm;
	separationMm = v.separationMm;
	firstId = v.firstId;
	dpi = v.dpi;
	marginMm = v.marginMm;
}

void ofApp::onPresetChanged() {
	const int user = userPreset();
	if (lastPreset == user) userValues = capture(); // leaving "user": remember it
	applying = true;
	apply(preset == user ? userValues : getPresets()[preset]);
	applying = false;
	lastPreset = preset;
	presetName.set(preset == user ? "user" : getPresets()[preset].name);
	dirty = true;
}

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowTitle("ofxAruco - print boards");
	ofBackground(40);

	preset.setMax(userPreset());
	params.add(preset, presetName, type, typeName, paper, paperName, dictionary, dictionaryName, countX, countY, squareMm, markerMm,
		separationMm, firstId, dpi, marginMm);
	presetName.setSerializable(false);
	typeName.setSerializable(false);
	paperName.setSerializable(false);
	dictionaryName.setSerializable(false);
	gui.setDefaultWidth(330);
	gui.setup(params, "settings.json", 10, 10);
	gui.add(saveButton.setup("save png + yml  (s)"));

	listeners.push(params.parameterChangedE().newListener([this](ofAbstractParameter & p) {
		const std::string & n = p.getName();
		if (n == presetName.getName() || n == typeName.getName() || n == paperName.getName() || n == dictionaryName.getName()) return;
		dirty = true;
		if (n == preset.getName()) {
			if (!applying) onPresetChanged();
			return;
		}
		// edited by hand while a preset is selected: switch to "user", keep the edit
		if (!applying && preset != userPreset()) {
			applying = true;
			preset = userPreset();
			applying = false;
			lastPreset = userPreset();
			presetName.set("user");
		}
	}));
	saveButton.addListener(this, &ofApp::save);

	loadSettings();

	// the self-check detector: main thread, results immediately
	checker.setup(dictionary, false);
}

void ofApp::loadSettings() {
	ofJson json;
	if (ofFile::doesFileExist("settings.json")) json = ofLoadJson("settings.json");
	applying = true;
	if (json.contains(params.getEscapedName())) ofDeserialize(json, params);
	applying = false;

	userValues = capture();
	if (json.contains("user")) {
		const ofJson & u = json["user"];
		userValues.type = u.value("type", userValues.type);
		userValues.paper = u.value("paper", userValues.paper);
		userValues.countX = u.value("countX", userValues.countX);
		userValues.countY = u.value("countY", userValues.countY);
		userValues.squareMm = u.value("squareMm", userValues.squareMm);
		userValues.markerMm = u.value("markerMm", userValues.markerMm);
		userValues.separationMm = u.value("separationMm", userValues.separationMm);
		userValues.firstId = u.value("firstId", userValues.firstId);
		userValues.dpi = u.value("dpi", userValues.dpi);
		userValues.marginMm = u.value("marginMm", userValues.marginMm);
	}
	lastPreset = -1;
	onPresetChanged(); // applies the selected preset (or the user values)
}

void ofApp::saveSettings() {
	if (preset == userPreset()) userValues = capture();
	ofJson json;
	ofSerialize(json, params);
	json["user"] = {
		{ "type", userValues.type }, { "paper", userValues.paper }, { "countX", userValues.countX }, { "countY", userValues.countY },
		{ "squareMm", userValues.squareMm }, { "markerMm", userValues.markerMm }, { "separationMm", userValues.separationMm },
		{ "firstId", userValues.firstId }, { "dpi", userValues.dpi }, { "marginMm", userValues.marginMm }
	};
	ofSavePrettyJson("settings.json", json);
}

//--------------------------------------------------------------
void ofApp::rebuild() {
	dirty = false;
	typeName.set(typeNames[type]);
	paperName.set(papers[paper].name);
	dictionaryName.set(ofxArucoUtils::dictionaryToName(dictionary));

	const float sq = squareMm / 1000.f, mk = markerMm / 1000.f, sep = separationMm / 1000.f;
	if (type == Charuco && mk >= sq) {
		status = "the marker must be smaller than the square";
		pixels.clear();
		return;
	}
	if (type == Charuco) {
		board = ofxArucoBoard::makeCharuco(countX, countY, sq, mk, dictionary, firstId);
	} else if (type == Grid || type == Cards) {
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
	if (type == Cards) drawCutLines();
	// allocate() (not loadData()) every time: when the size changes loadData()
	// reallocates without the gray -> RGB swizzle and the preview turns red
	preview.allocate(pixels);

	// self-check: detect the generated image
	checker.dictionary = dictionary;
	checker.detect(pixels);

	const glm::vec2 size = board.getSize() * 1000.f; // mm
	const float w = size.x + 2 * marginMm, h = size.y + 2 * marginMm;
	auto fits = [&](const Paper & p) { return (w <= p.w && h <= p.h) || (w <= p.h && h <= p.w); };
	fitsPaper = fits(papers[paper]);
	std::string fitsOn;
	for (auto & p : papers) {
		if (fits(p)) fitsOn += std::string(p.name) + " ";
	}

	std::stringstream ss;
	ss << presetName.get() << ": " << typeName.get() << ", " << board.getNumMarkers() << " markers (ids " << firstId << " - "
	   << firstId + board.getNumMarkers() - 1 << ")\n"
	   << "printed size: " << ofToString(size.x / 10.f, 1) << " x " << ofToString(size.y / 10.f, 1) << " cm (+ " << marginMm
	   << " mm margin)\n"
	   << "image: " << pixels.getWidth() << " x " << pixels.getHeight() << " px at " << dpi << " dpi\n"
	   << "paper: " << papers[paper].name << (fitsPaper ? " OK" : " - DOES NOT FIT") << "   (fits on: " << fitsOn << ")\n"
	   << "self-check: " << checker.getNumMarkers() << " / " << board.getNumMarkers() << " markers detected"
	   << (checker.getNumMarkers() == board.getNumMarkers() ? "  OK" : "  (check the sizes)");
	status = ss.str();
}

// Grey cut lines through the middle of the gaps between cards, and around the
// outside, running edge to edge so they also work as crop marks.
void ofApp::drawCutLines() {
	if (!pixels.isAllocated()) return;
	const float ppm = dpi / 0.0254f; // pixels per meter
	const glm::vec2 size = board.getSize();
	const float scale = std::round(size.x * ppm) / size.x; // what getImageForPrint actually used
	const float margin = std::round(marginMm / 1000.f * ppm);
	const float mk = markerMm / 1000.f, sep = separationMm / 1000.f;
	const int lineWidth = std::max(1, int(std::round(0.0002f * ppm))); // 0.2 mm
	const unsigned char grey = 170;

	const int W = pixels.getWidth(), H = pixels.getHeight(), C = pixels.getNumChannels();
	unsigned char * data = pixels.getData();
	auto set = [&](int x, int y) {
		if (x < 0 || y < 0 || x >= W || y >= H) return;
		for (int c = 0; c < C; c++) data[(y * W + x) * C + c] = grey;
	};
	for (int i = 0; i <= countX; i++) {
		const int x0 = int(std::round(margin + (i * (mk + sep) - sep * 0.5f) * scale - lineWidth * 0.5f));
		for (int x = x0; x < x0 + lineWidth; x++)
			for (int y = 0; y < H; y++) set(x, y);
	}
	for (int j = 0; j <= countY; j++) {
		const int y0 = int(std::round(margin + (j * (mk + sep) - sep * 0.5f) * scale - lineWidth * 0.5f));
		for (int y = y0; y < y0 + lineWidth; y++)
			for (int x = 0; x < W; x++) set(x, y);
	}
}

std::string ofApp::getFileName() const {
	const std::string dict = ofxArucoUtils::dictionaryToName(dictionary);
	const std::string counts = ofToString(countX.get()) + "x" + ofToString(countY.get());
	switch (type) {
	case Charuco:
		return "charuco_" + counts + "_" + ofToString(squareMm.get(), 0) + "mm_" + ofToString(markerMm.get(), 0) + "mm_" + dict + "_id"
			+ ofToString(firstId.get());
	case Grid:
		return "grid_" + counts + "_" + ofToString(markerMm.get(), 0) + "mm_" + ofToString(separationMm.get(), 0) + "mm_" + dict + "_id"
			+ ofToString(firstId.get());
	case Cards:
		return "cards_" + counts + "_" + ofToString(markerMm.get(), 0) + "mm_" + dict + "_id" + ofToString(firstId.get()) + "-"
			+ ofToString(firstId.get() + countX.get() * countY.get() - 1);
	default:
		return "marker_" + ofToString(firstId.get()) + "_" + ofToString(markerMm.get(), 0) + "mm_" + dict;
	}
}

void ofApp::save() {
	if (!pixels.isAllocated()) return;
	const std::string name = getFileName();
	ofxArucoUtils::savePngWithDpi(pixels, name + ".png", dpi); // prints at the right size at 100%
	board.save(name + ".yml");
	saveSettings();
	ofLogNotice() << "saved " << ofToDataPath(name + ".png", true) << " and " << name << ".yml";
	status += "\nsaved bin/data/" + name + ".png + .yml";
}

//--------------------------------------------------------------
void ofApp::update() {
	if (dirty) rebuild();
}

void ofApp::draw() {
	// the paper (turned to match the board), with the image on it at scale
	ofRectangle area(360, 10, ofGetWidth() - 370, ofGetHeight() - 130);
	if (preview.isAllocated() && pixels.isAllocated()) {
		const float bw = board.getSize().x * 1000.f + 2 * marginMm, bh = board.getSize().y * 1000.f + 2 * marginMm;
		float pw = papers[paper].w, ph = papers[paper].h;
		if ((bw > bh) != (pw > ph)) std::swap(pw, ph);

		ofRectangle page(0, 0, std::max(pw, bw), std::max(ph, bh)); // grow to show what sticks out
		page.scaleTo(area);
		const float k = page.width / std::max(pw, bw); // screen px per mm
		ofRectangle sheet(0, 0, pw * k, ph * k);
		sheet.setFromCenter(page.getCenter(), pw * k, ph * k);
		ofRectangle image;
		image.setFromCenter(page.getCenter(), bw * k, bh * k);

		ofSetColor(255);
		ofDrawRectangle(sheet);
		preview.draw(image);
		checker.drawMarkers(image); // the self-check detections on top
		ofNoFill();
		ofSetColor(fitsPaper ? ofColor(120) : ofColor(255, 60, 60));
		ofDrawRectangle(sheet);
		ofFill();
		ofSetColor(255);
		ofDrawBitmapStringHighlight(papers[paper].name, sheet.x + 4, sheet.getBottom() - 6);
	}
	ofDrawBitmapStringHighlight(status, 360, ofGetHeight() - 105);
	ofDrawBitmapStringHighlight("Print at 100% scale, then measure and fix the sizes in the .yml if needed.", 360, ofGetHeight() - 15);
	gui.draw();
}

void ofApp::keyPressed(int key) {
	if (key == 's') save();
}

void ofApp::exit() {
	saveSettings();
}
