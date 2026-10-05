#include "SimulatedCameras.h"
#include <glm/gtx/euler_angles.hpp>

glm::mat4 lookAtCv(const glm::vec3 & eye, const glm::vec3 & target) {
	const glm::vec3 z = glm::normalize(target - eye);                      // forward
	const glm::vec3 x = glm::normalize(glm::cross(glm::vec3(0, 1, 0), z)); // right (world y is down)
	const glm::vec3 y = glm::cross(z, x);                                   // down
	glm::mat4 m(1.f);
	m[0] = glm::vec4(x, 0);
	m[1] = glm::vec4(y, 0);
	m[2] = glm::vec4(z, 0);
	m[3] = glm::vec4(eye, 1);
	return m;
}

//--------------------------------------------------------------
void SimulatedScene::setup(const ofxArucoBoard & b) {
	board = b;
	ofPixels pixels;
	// The texture must have exactly the board's proportions, or the simulated board
	// would be slightly bigger/smaller than its definition (= a fake depth error).
	// So the margin is derived from whole pixels, not the other way around.
	const float pixelsPerMeter = 5000;
	const int boardPixels = int(std::round(board.getSize().x * pixelsPerMeter));
	const int marginPixels = int(std::round(0.02f * pixelsPerMeter));
	board.getImage(pixels, boardPixels, marginPixels);
	const float margin = marginPixels * board.getSize().x / boardPixels; // meters
	ofDisableArbTex(); // GL_TEXTURE_2D so it can have mipmaps (sharp at grazing angles)
	texture.allocate(pixels);
	ofEnableArbTex();
	texture.loadData(pixels);
	texture.generateMipmap();
	texture.setTextureMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
	// anisotropic filtering: without it, a board seen at a grazing angle is blurred much more
	// in one direction, which shifts the detected corners (a renderer artifact, real lenses don't do that)
	texture.bind();
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 16.f);
	texture.unbind();

	// board frame: origin top-left, x right, y down. The texture's top-left is the board's top-left.
	const glm::vec2 s = board.getSize();
	quad.setMode(OF_PRIMITIVE_TRIANGLE_FAN);
	quad.addVertex({ -margin, -margin, 0 });
	quad.addTexCoord(texture.getCoordFromPercent(0, 0));
	quad.addVertex({ s.x + margin, -margin, 0 });
	quad.addTexCoord(texture.getCoordFromPercent(1, 0));
	quad.addVertex({ s.x + margin, s.y + margin, 0 });
	quad.addTexCoord(texture.getCoordFromPercent(1, 1));
	quad.addVertex({ -margin, s.y + margin, 0 });
	quad.addTexCoord(texture.getCoordFromPercent(0, 1));

	from = to = boardToWorld = randomPose();
	phaseStart = ofGetElapsedTimef();
}

glm::mat4 SimulatedScene::randomPose() {
	// around 1.8 m in front of camera 0, facing the cameras with some tilt
	const glm::vec3 center(ofRandom(-0.35f, 0.35f), ofRandom(-0.2f, 0.25f), ofRandom(1.5f, 2.1f));
	const glm::mat4 rot = glm::eulerAngleXYZ(glm::radians(ofRandom(-25.f, 25.f)), glm::radians(ofRandom(-30.f, 30.f)),
		glm::radians(ofRandom(-20.f, 20.f)));
	return glm::translate(glm::mat4(1), center) * rot * glm::translate(glm::mat4(1), -board.getCenter());
}

void SimulatedScene::update() {
	const float t = ofGetElapsedTimef() - phaseStart;
	if (t < holdTime) {
		boardToWorld = from; // held still: this is when samples get captured
	} else if (t < holdTime + moveTime) {
		const float a = glm::smoothstep(0.f, 1.f, (t - holdTime) / moveTime);
		const glm::quat q = glm::slerp(glm::quat_cast(glm::mat3(from)), glm::quat_cast(glm::mat3(to)), a);
		boardToWorld = glm::mat4_cast(q);
		boardToWorld[3] = glm::mix(from[3], to[3], a);
	} else {
		from = to;
		to = randomPose();
		phaseStart = ofGetElapsedTimef();
		boardToWorld = from;
	}
}

void SimulatedScene::drawBoard() const {
	ofPushMatrix();
	ofMultMatrix(boardToWorld);
	ofSetColor(255);
	texture.bind();
	quad.draw();
	texture.unbind();
	ofPopMatrix();
}

//--------------------------------------------------------------
void SimulatedCamera::setup(const std::string & n, const ofxArucoIntrinsics & intr, const glm::mat4 & camToWorld,
	std::shared_ptr<SimulatedScene> s) {
	name = n;
	intrinsics = intr;
	cameraToWorld = camToWorld;
	scene = s;
	ofFboSettings settings;
	settings.width = intr.getWidth();
	settings.height = intr.getHeight();
	settings.internalformat = GL_RGB;
	settings.useDepth = true;
	settings.numSamples = 4;
	fbo.allocate(settings);
}

void SimulatedCamera::update() {
	// render the scene through this camera: camera -> world inverse, in OpenCV convention
	fbo.begin();
	ofClear(60, 64, 70, 255);
	intrinsics.begin(ofRectangle(0, 0, fbo.getWidth(), fbo.getHeight()));
	ofEnableDepthTest();
	ofMultMatrix(ofxArucoUtils::rigidInverse(cameraToWorld));
	scene->drawBoard();
	ofDisableDepthTest();
	intrinsics.end();
	fbo.end();
	fbo.readToPixels(pixels);
	frameNew = true;
}
