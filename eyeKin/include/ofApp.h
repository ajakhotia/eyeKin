#pragma once

#include "ofMain.h"
#include "objectSegmentation.h"
#include "pcl.h"
#include "settings.h"
#include "timer.h"

class ofApp : public ofBaseApp
{
public:
	ofApp();
	~ofApp();

	// Interfaces
	personalRobotics::ObjectSegmentor segmentor;

	// Useful data
	cv::Mat checkerboard;
	ofImage ofCheckerboard;
	int numCheckerPtsX;
	int numCheckerPtsY;

	// Configurations
	int screenWidth;
	int screenHeight;

	// Calibration
	cv::Mat homography;

	// Flags
	bool tablePlaneFound;
	bool homographyFound;
	bool isCalibrating;

	// Debug
	#ifdef DEBUG_POINTCLOUD
		pcl::visualization::CloudViewer viewer;
	#endif

	#ifdef WRITE_TO_VIDEO
		cv::VideoWriter videoWriter;
	#endif
public:
	// Default openframeworks methods
	void setup();
	void update();
	void draw(); 
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	
	// Calibration methods
	void findTable();
	void findHomography();
	void calibrate();

	// Routines
	void updateRoutine();
	void drawRoutine();
};