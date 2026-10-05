#pragma once

// Virtual cameras looking at a printed board that someone moves around and
// holds still from time to time. Rendered with OpenGL using the cameras'
// intrinsics, so the whole calibration workflow can be tried (and checked
// against the ground truth) without any hardware.

#include "CameraSource.h"

class SimulatedScene {
public:
	void setup(const ofxArucoBoard & board);
	void update(); // animates the board
	void drawBoard() const; // in world coordinates (OpenCV convention)
	const glm::mat4 & getBoardToWorld() const { return boardToWorld; }

private:
	glm::mat4 randomPose();
	ofxArucoBoard board;
	ofTexture texture;
	ofMesh quad;
	glm::mat4 boardToWorld{1.f}, from{1.f}, to{1.f};
	float phaseStart = 0;
	const float holdTime = 1.6f, moveTime = 1.0f;
};

class SimulatedCamera : public CameraSource {
public:
	void setup(const std::string & name, const ofxArucoIntrinsics & intrinsics, const glm::mat4 & cameraToWorld,
		std::shared_ptr<SimulatedScene> scene);
	void update() override;
	bool isFrameNew() const override { return frameNew; }
	const ofPixels & getPixels() const override { return pixels; }
	void draw(const ofRectangle & r) const override { fbo.draw(r); }
	const ofxArucoIntrinsics & getIntrinsics() const override { return intrinsics; }
	std::string getName() const override { return name; }

	// ground truth, to compare with the calibration
	const glm::mat4 & getTrueCameraToWorld() const { return cameraToWorld; }

private:
	std::string name;
	ofxArucoIntrinsics intrinsics;
	glm::mat4 cameraToWorld{1.f};
	std::shared_ptr<SimulatedScene> scene;
	ofFbo fbo;
	ofPixels pixels;
	bool frameNew = false;
};

// Camera pose (camera -> world, OpenCV convention) looking from eye to target.
glm::mat4 lookAtCv(const glm::vec3 & eye, const glm::vec3 & target);
