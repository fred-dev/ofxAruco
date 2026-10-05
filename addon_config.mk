meta:
	ADDON_NAME = ofxAruco
	ADDON_DESCRIPTION = Threaded ArUco / ChArUco marker and board detection, pose estimation and multi camera calibration (OpenCV objdetect)
	ADDON_AUTHOR = arturo castro, Frederick Rodrigues
	ADDON_TAGS = "computer vision" "augmented reality" "aruco" "charuco" "calibration"
	ADDON_URL = https://github.com/arturoc/ofxAruco

common:
	# OpenCV (>= 4.7, with the objdetect module) comes from ofxOpenCv
	ADDON_DEPENDENCIES = ofxOpenCv
