#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	panel.setup("control", 0, 0, 300, 600);
	panel.addPanel("control", 1);
	panel.addSlider("x rotation", "xRotation", 0, -180, 180, false);
	panel.addSlider("y rotation", "yRotation", 0, -180, 180, false);
	panel.addSlider("z rotation", "zRotation", 0, -180, 180, false);
	panel.addSlider("x position", "xPosition", 0, -800, 800, false);
	panel.addSlider("y position", "yPosition", 0, -800, 800, false);
	panel.addSlider("z position", "zPosition", 0, -800, 800, false);
	
	panel.addSlider("focusDistance", "focusDistance", 800, 0, 2000, false);
	panel.addSlider("aperture", "aperture", 0, 0, .1, false);
	panel.addSlider("pointBrightness", "pointBrightness", 1, 0, 1, false);
	panel.addSlider("rgbBrightness",  "rgbBrightness", 1, 0, 1, false);
	panel.addSlider("maxPointSize", "maxPointSize", 1, 0, 30, false);
	
	
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();
    
	ofEnableAlphaBlending();
	ofSetFrameRate(30);
	
	dofShader.setup("DOFCloud.vert", "DOFCloud.frag");
}

//--------------------------------------------------------------
void testApp::update()
{
	ofBackground(0, 0, 0);
	kinect.update();
	panel.update();
}

//--------------------------------------------------------------
void testApp::draw()
{	
	ofPushMatrix();
	ofTranslate(ofGetWidth() / 2, ofGetHeight() / 2);
	
	int w = 640;
	int h = 480;
	float* distancePixels = kinect.getDistancePixels();
	int i = 0;
	glColor3f(1, 1, 1);
	ofTranslate(panel.getValueF("xPosition"),
							panel.getValueF("yPosition"),
							panel.getValueF("zPosition"));
	ofRotateX(panel.getValueF("xRotation"));
	ofRotateY(panel.getValueF("yRotation"));
	ofRotateZ(panel.getValueF("zRotation"));
	//ofTranslate(-w / 2, -h / 2);
	//ofScale(4, 4, 4);
	float scaleFactor = .0021;
	float minDistance = -10;
	
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glEnable(GL_ALPHA_TEST);
	glAlphaFunc(GL_GREATER, 0);
	
	// super helpful: http://pmviewer.sourceforge.net/method.php
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_ARB);
	
	dofShader.begin();
	
	dofShader.setUniform1f("focusDistance", panel.getValueF("focusDistance"));
	dofShader.setUniform1f("aperture", panel.getValueF("aperture"));
	dofShader.setUniform1f("pointBrightness", panel.getValueF("pointBrightness"));
	dofShader.setUniform1f("rgbBrightness",  panel.getValueF("rgbBrightness"));
	dofShader.setUniform1f("maxPointSize", panel.getValueF("maxPointSize"));
	
	glBegin(GL_POINTS);
	for(int y = 0; y < h; y++) {
		for(int x = 0; x < w; x++) {
			float z = distancePixels[i++];
			if(z != 0) {
				glVertex3f((x - w / 2) * (z + minDistance) * scaleFactor, (y - h / 2) * (z + minDistance) * scaleFactor, z);
			}
		}
	}
	glEnd();
	ofPopMatrix();
	
	dofShader.end();
	
	panel.draw();
}

//--------------------------------------------------------------
void testApp::exit()
{
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key)
{
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y) {
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button) {	
	panel.mouseDragged(x, y, button);
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button) {	
	panel.mousePressed(x, y, button);
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button) {	
	panel.mouseReleased();
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}

