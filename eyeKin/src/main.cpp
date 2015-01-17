#include "ofApp.h"

//========================================================================

int main( ){
	ofSetupOpenGL(DEFAULT_SCREEN_WIDTH, DEFAULT_SCREEN_HEIGHT, RENDER_WINDOW_MODE);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofApp *app = new ofApp();
	ofRunApp(app);
	delete app;
}
