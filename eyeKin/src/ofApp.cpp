#include "ofApp.h"


// Constructor-Destructor
ofApp::ofApp(){
	// Configuration
	screenWidth = DEFAULT_SCREEN_WIDTH;
	screenHeight = DEFAULT_SCREEN_HEIGHT;

	// Useful Images
	numCheckerPtsX = 0;
	numCheckerPtsY = 0;
	personalRobotics::createCheckerboard(checkerboard, screenWidth, screenHeight, numCheckerPtsX, numCheckerPtsY);
	ofCheckerboard.setFromPixels(checkerboard.data, checkerboard.cols, checkerboard.rows, OF_IMAGE_GRAYSCALE);

	//Flags
	tablePlaneFound = false;
	homographyFound = false;
	isCalibrating = true;
}

ofApp::~ofApp()
{
	
}

// Default openframeworks methods
void ofApp::setup(){
	ofSetVerticalSync(true);
	ofSetFrameRate(30);
	ofBackground(0, 0, 0);
	ofSetLineWidth(5);
}

void ofApp::update(){
	#ifdef DEBUG_PROFILER
		personalRobotics::Timer timer("update()");
	#endif
	if (isCalibrating)
	{
		calibrate();
	}
	else
	{
		updateRoutine();
	}
	#ifdef DEBUG_PROFILER
		timer.toc();
	#endif
}

void ofApp::draw(){
	if (isCalibrating)
		ofCheckerboard.draw(0, 0);
	else
	{	
		drawRoutine();
	}
}

void ofApp::keyPressed(int key){

}
void ofApp::keyReleased(int key){

}
void ofApp::mouseMoved(int x, int y ){

}
void ofApp::mouseDragged(int x, int y, int button){

}
void ofApp::mousePressed(int x, int y, int button){

}
void ofApp::mouseReleased(int x, int y, int button){

}
void ofApp::windowResized(int w, int h){

}
void ofApp::gotMessage(ofMessage msg){

}
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

// Calibration methods
void ofApp::findTable()
{
	tablePlaneFound= segmentor.findTablePlane();
}
void ofApp::findHomography()
{
	vector<cv::Point2f> detectedCorners, checkerboardCorners;
	bool foundCorners = findChessboardCorners(*segmentor.getColorImagePtr(), cv::Size(numCheckerPtsX, numCheckerPtsY), detectedCorners, CV_CALIB_CB_ADAPTIVE_THRESH);
	bool foundProjectedCorners = findChessboardCorners(checkerboard, cv::Size(numCheckerPtsX, numCheckerPtsY), checkerboardCorners, CV_CALIB_CB_ADAPTIVE_THRESH);
	if (foundCorners)
	{
		cv::Mat invertedHomography = cv::findHomography(detectedCorners, checkerboardCorners, CV_RANSAC);
		cv::Mat homographyCorrection = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, -1, (DEFAULT_SCREEN_HEIGHT), 0, 0, 1);
		homography = homographyCorrection*invertedHomography;
		homographyFound = true;
	}
}
void ofApp::calibrate()
{
	if (!tablePlaneFound && segmentor.isDepthAllocated)
		findTable();
	if (!homographyFound && segmentor.isColorAllocated)
		findHomography();
	if (tablePlaneFound && homographyFound)
	{
		isCalibrating = false;
		segmentor.setHomography(homography);
		segmentor.startSegmentor();
	}
}

// Update Routine
void ofApp::updateRoutine()
{

}

// Draw routine
void ofApp::drawRoutine()
{
	ofSetColor(255, 0, 0);
	segmentor.lockList();
	for (std::vector<personalRobotics::Entity>::iterator entityPtr = segmentor.getEntityList()->begin(); entityPtr != segmentor.getEntityList()->end(); entityPtr++)
	{
		entityPtr->drawBoundingBox();
	}
	segmentor.unlockList();
}