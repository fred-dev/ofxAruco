#include "ofApp.h"
#include <cairo/cairo-pdf.h>

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
	gui.add(saveButton.setup("save pdf + yml  (s)"));

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
	ready = false;
	typeName.set(typeNames[type]);
	paperName.set(papers[paper].name);
	dictionaryName.set(ofxArucoUtils::dictionaryToName(dictionary));

	// keep the marker ids inside the dictionary (ids 0 .. size - 1)
	const int numMarkers = type == Charuco ? (countX * countY) / 2 : (type == SingleMarker ? 1 : countX * countY);
	const int dictSize = ofxArucoUtils::getDictionarySize(dictionary);
	const int maxFirstId = dictSize - numMarkers;
	if (maxFirstId < 0) {
		status = ofToString(numMarkers) + " markers but " + dictionaryName.get() + " has only " + ofToString(dictSize)
			+ ": use fewer markers or a bigger dictionary";
		return;
	}
	firstId.setMax(maxFirstId);
	if (firstId > maxFirstId) {
		applying = true; // a correction, not a user edit: keeps the preset
		firstId = maxFirstId;
		applying = false;
	}

	const float sq = squareMm / 1000.f, mk = markerMm / 1000.f, sep = separationMm / 1000.f;
	if (type == Charuco && mk >= sq) {
		status = "the marker must be smaller than the square";
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
		return;
	}

	// the sheet: the paper, turned like the board, grown if the board (+ margin) is bigger
	const glm::vec2 size = board.getSize() * 1000.f; // mm
	boardW = size.x;
	boardH = size.y;
	const float w = boardW + 2 * marginMm, h = boardH + 2 * marginMm;
	sheetW = papers[paper].w;
	sheetH = papers[paper].h;
	if ((w > h) != (sheetW > sheetH)) std::swap(sheetW, sheetH);
	pageW = std::max(sheetW, w);
	pageH = std::max(sheetH, h);
	boardX = (pageW - boardW) / 2;
	boardY = (pageH - boardH) / 2;

	// draw the sheet exactly as the pdf will be, then detect on it (self-check) and show it
	if (!renderPage()) {
		status = "could not draw the board";
		return;
	}
	preview.allocate(pagePixels);
	checker.dictionary = dictionary;
	checker.detect(pagePixels);
	ready = true;

	auto fits = [&](const Paper & p) { return (w <= p.w && h <= p.h) || (w <= p.h && h <= p.w); };
	fitsPaper = fits(papers[paper]);
	// most printers leave 3 - 5 mm unprinted at the edges
	const float edge = std::min((papers[paper].w - std::min(w, h)) / 2, (papers[paper].h - std::max(w, h)) / 2);
	std::string fitsOn;
	for (auto & p : papers) {
		if (fits(p)) fitsOn += std::string(p.name) + " ";
	}

	std::stringstream ss;
	ss << presetName.get() << ": " << typeName.get() << ", " << board.getNumMarkers() << " markers (ids " << firstId << " - "
	   << firstId + board.getNumMarkers() - 1 << ")\n"
	   << "printed size: " << ofToString(size.x / 10.f, 1) << " x " << ofToString(size.y / 10.f, 1) << " cm (+ " << marginMm
	   << " mm margin), pdf page " << ofToString(pageW, 1) << " x " << ofToString(pageH, 1) << " mm\n"
	   << "paper: " << papers[paper].name << " (" << papers[paper].w << " x " << papers[paper].h << " mm)"
	   << (!fitsPaper ? " - DOES NOT FIT" : edge < 4 ? " - fits, but only " + ofToString(edge, 1) + " mm to the edge (printers need ~4)" : " OK")
	   << "   (fits on: " << fitsOn << ")\n"
	   << "self-check on the printed sheet (" << pagePixels.getWidth() << " x " << pagePixels.getHeight() << " px): "
	   << checker.getNumMarkers() << " / " << board.getNumMarkers() << " markers detected"
	   << (checker.getNumMarkers() == board.getNumMarkers() ? "  OK" : "  (check the sizes)");
	status = ss.str();
}

// The whole sheet in millimetres (the caller scales cairo to mm), top left = (0, 0):
// every marker cell, the chessboard squares, the cut lines and a note with a 50 mm
// bar. Everything is vector shapes, so the pdf prints edges exactly where they are.
void ofApp::drawSheet(cairo_t * cr) const {
	const cv::aruco::Board & cvBoard = board.getCvBoard();
	const auto & ids = cvBoard.getIds();
	const auto & objPoints = cvBoard.getObjPoints(); // metres, y down, corners clockwise from top left
	const cv::aruco::Dictionary & dict = cvBoard.getDictionary();
	auto toPage = [&](const cv::Point3f & p) { return glm::vec2(boardX + p.x * 1000.f, boardY + p.y * 1000.f); };
	auto quad = [&](const glm::vec2 & a, const glm::vec2 & b, const glm::vec2 & c, const glm::vec2 & d) {
		cairo_move_to(cr, a.x, a.y);
		cairo_line_to(cr, b.x, b.y);
		cairo_line_to(cr, c.x, c.y);
		cairo_line_to(cr, d.x, d.y);
		cairo_close_path(cr);
	};

	// all the black in one path, filled once: no hairline seams between touching cells
	cairo_new_path(cr);
	std::vector<glm::vec2> centres;
	for (size_t m = 0; m < ids.size() && m < objPoints.size(); m++) {
		cv::Mat bits; // one pixel per cell, with the 1 cell black border
		dict.generateImageMarker(ids[m], dict.markerSize + 2, bits, 1);
		const int n = bits.rows;
		glm::vec2 c[4];
		for (int k = 0; k < 4; k++) c[k] = toPage(objPoints[m][k]);
		centres.push_back((c[0] + c[1] + c[2] + c[3]) * 0.25f);
		auto at = [&](float u, float v) { return (1 - u) * (1 - v) * c[0] + u * (1 - v) * c[1] + u * v * c[2] + (1 - u) * v * c[3]; };
		for (int r = 0; r < n; r++) {
			for (int x = 0; x < n;) {
				if (bits.at<uchar>(r, x) > 127) {
					x++;
					continue;
				}
				int e = x; // a run of black cells on this row
				while (e < n && bits.at<uchar>(r, e) <= 127) e++;
				const float u0 = float(x) / n, u1 = float(e) / n, v0 = float(r) / n, v1 = float(r + 1) / n;
				quad(at(u0, v0), at(u1, v0), at(u1, v1), at(u0, v1));
				x = e;
			}
		}
	}
	if (type == Charuco) {
		// the squares that hold a marker are white, the others black
		const float s = squareMm;
		for (int y = 0; y < countY; y++) {
			for (int x = 0; x < countX; x++) {
				const ofRectangle r(boardX + x * s, boardY + y * s, s, s);
				bool hasMarker = false;
				for (auto & c : centres) hasMarker |= r.inside(c.x, c.y);
				if (!hasMarker) cairo_rectangle(cr, r.x, r.y, r.width, r.height);
			}
		}
	}
	cairo_set_fill_rule(cr, CAIRO_FILL_RULE_WINDING);
	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_fill(cr);

	// cards: grey cut lines through the middle of the gaps and around the outside,
	// across the board and its margin, so they also work as crop marks
	if (type == Cards) {
		const float step = markerMm + separationMm, half = separationMm * 0.5f, line = 0.2f;
		const float top = std::max(0.f, boardY - marginMm), bottom = std::min(pageH, boardY + boardH + marginMm);
		const float left = std::max(0.f, boardX - marginMm), right = std::min(pageW, boardX + boardW + marginMm);
		for (int i = 0; i <= countX; i++) {
			cairo_rectangle(cr, boardX + i * step - half - line / 2, top, line, bottom - top);
		}
		for (int j = 0; j <= countY; j++) {
			cairo_rectangle(cr, left, boardY + j * step - half - line / 2, right - left, line);
		}
		cairo_set_source_rgb(cr, 170 / 255.0, 170 / 255.0, 170 / 255.0);
		cairo_fill(cr);
	}

	// a note and a 50 mm bar under the board (and its margin), when there is room:
	// measure the bar after printing to check the scale
	const float noteTop = boardY + boardH + marginMm;
	if (pageH - noteTop >= 12) {
		const float y = noteTop + (pageH - noteTop) * 0.5f;
		const float x = std::max(8.f, boardX);
		cairo_set_source_rgb(cr, 0.3, 0.3, 0.3);
		cairo_rectangle(cr, x, y - 4, 50, 0.3);
		cairo_rectangle(cr, x, y - 5.5f, 0.3, 3);
		cairo_rectangle(cr, x + 50 - 0.3f, y - 5.5f, 0.3, 3);
		cairo_fill(cr);
		cairo_select_font_face(cr, "sans-serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
		cairo_set_font_size(cr, 2.8);
		std::string note = board.getName() + "   |   " + dictionaryName.get() + ", marker " + ofToString(markerMm.get(), 1) + " mm";
		if (type == Charuco) note += ", square " + ofToString(squareMm.get(), 1) + " mm";
		cairo_move_to(cr, x + 53, y - 3);
		cairo_show_text(cr, "50 mm: print at 100% / actual size, then measure");
		cairo_move_to(cr, x, y + 2);
		cairo_show_text(cr, note.c_str());
	}
}

// The sheet as pixels (the self-check and the preview), drawn by the same code as the pdf.
bool ofApp::renderPage() {
	const float pxPerMm = std::min(dpi / 25.4f, 2400.f / std::max(pageW, pageH)); // enough to detect, fast to check
	const int W = std::max(1, int(std::round(pageW * pxPerMm))), H = std::max(1, int(std::round(pageH * pxPerMm)));
	cairo_surface_t * surface = cairo_image_surface_create(CAIRO_FORMAT_RGB24, W, H);
	if (cairo_surface_status(surface) != CAIRO_STATUS_SUCCESS) {
		cairo_surface_destroy(surface);
		return false;
	}
	cairo_t * cr = cairo_create(surface);
	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_paint(cr);
	cairo_scale(cr, W / pageW, H / pageH);
	drawSheet(cr);
	cairo_destroy(cr);
	cairo_surface_flush(surface);

	const unsigned char * data = cairo_image_surface_get_data(surface);
	const int stride = cairo_image_surface_get_stride(surface);
	pagePixels.allocate(W, H, OF_PIXELS_GRAY);
	unsigned char * out = pagePixels.getData();
	for (int y = 0; y < H; y++) {
		const unsigned char * row = data + y * stride;
		for (int x = 0; x < W; x++) out[y * W + x] = row[x * 4 + 1]; // grey: every channel is the same
	}
	cairo_surface_destroy(surface);
	return true;
}

bool ofApp::savePdf(const std::string & path) const {
	const double ptPerMm = 72.0 / 25.4;
	cairo_surface_t * surface = cairo_pdf_surface_create(ofToDataPath(path, true).c_str(), pageW * ptPerMm, pageH * ptPerMm);
	cairo_t * cr = cairo_create(surface);
	cairo_scale(cr, ptPerMm, ptPerMm);
	drawSheet(cr);
	cairo_show_page(cr);
	cairo_destroy(cr);
	cairo_surface_finish(surface);
	const bool ok = cairo_surface_status(surface) == CAIRO_STATUS_SUCCESS;
	cairo_surface_destroy(surface);
	return ok;
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
	if (!ready) return;
	const std::string name = getFileName();
	if (!savePdf(name + ".pdf")) {
		status += "\ncould not write bin/data/" + name + ".pdf";
		return;
	}
	board.save(name + ".yml");
	saveSettings();
	ofLogNotice() << "saved " << ofToDataPath(name + ".pdf", true) << " and " << name << ".yml";
	status += "\nsaved bin/data/" + name + ".pdf + .yml";
}

//--------------------------------------------------------------
void ofApp::update() {
	if (dirty) rebuild();
}

void ofApp::draw() {
	// the sheet as it will print, with the paper's outline
	ofRectangle area(360, 10, ofGetWidth() - 370, ofGetHeight() - 130);
	if (ready && preview.isAllocated()) {
		ofRectangle page(0, 0, pageW, pageH);
		page.scaleTo(area);
		const float k = page.width / pageW; // screen px per mm
		ofRectangle sheet;
		sheet.setFromCenter(page.getCenter(), sheetW * k, sheetH * k);

		ofSetColor(255);
		preview.draw(page);
		checker.drawMarkers(page); // the self-check ran on this image
		ofNoFill();
		ofSetColor(fitsPaper ? ofColor(120) : ofColor(255, 60, 60));
		ofDrawRectangle(sheet);
		ofFill();
		ofSetColor(255);
		ofDrawBitmapStringHighlight(papers[paper].name, sheet.x + 4, sheet.getBottom() - 6);
	}
	ofDrawBitmapStringHighlight(status, 360, ofGetHeight() - 105);
	ofDrawBitmapStringHighlight("Print the pdf at 100% / actual size (not \"fit\"), then measure the 50 mm bar and the marker.", 360,
		ofGetHeight() - 15);
	gui.draw();
}

void ofApp::keyPressed(int key) {
	if (key == 's') save();
}

void ofApp::exit() {
	saveSettings();
}
