/*
* 
* Copyright (c) 2012, Ban the Rewind
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

// Includes
#include "cinder/app/AppBasic.h"
#include "cinder/Arcball.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include <CinderKinect.h>
#include <GL/gl.h>		   // Open Graphics Library (OpenGL) header
#include <GL/glut.h>	   // The GL Utility Toolkit (GLUT) Header

// Imports
using namespace ci;
using namespace ci::app;
using namespace std;

/* 
* This application demonstrates how to represent the 
* Kinect's depth image in 3D space.
*/
#define MODE_NORMAL  0
#define MODE_SHADOW  1
#define MODE_REFLECT 2

#define DEF_WIDTH  640
#define DEF_HEIGHT 480
#define FPS_COUNTER_NUM_FRAMES 1000

class PointCloudApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void draw();
	void keyDown( ci::app::KeyEvent event );
	void mouseDown( ci::app::MouseEvent event );
	void mouseDrag( ci::app::MouseEvent event );
	void prepareSettings( ci::app::AppBasic::Settings *settings );
	void shutdown();
	void setup();
	void update();
	uint16_t RawDepthToMeters(uint16_t raw);
	void CreateCube();
	void draw_model(int mode);
private:

	// Kinect
	CinderKinect			mKinect;
	ci::Surface16u			mSurface16;

	// Depth points
	std::vector<ci::Vec3f>	mPoints;

	// Camera
	ci::Arcball				mArcball;
	ci::CameraPersp			mCamera;

	// Save screen shot
	void					screenShot();
	struct {
		Display *disp;
		int screen;
		int fullscreen;

		int w, h;
		int win_w, win_h;
		unsigned int depth;
		int doublebuf;

		GLuint tex_ground;
		GLuint tex_model;
		int texturing;
		int color_texture;
		int lighting;
		int shadowing;
		int reflection;

		GLfloat light_pos[4];
	} appdata;
};

// Kinect image size
const Vec2i	kKinectSize( 640, 480 );

float g_rotation = 0;
float g_rotation_speed = 0.2f;
double t = 0.0, t_speed = 0.0;

// Render
void PointCloudApp::draw()
{

	// Clear window
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::black() );
	gl::setMatrices( mCamera );
	gl::rotate( mArcball.getQuat() );

	// Draw point cloud
	glBegin( GL_POINTS );
	for ( vector<Vec3f>::const_iterator pointIt = mPoints.cbegin(); pointIt != mPoints.cend(); ++pointIt ) {
		float depth = 1.0f - pointIt->z / mCamera.getEyePoint().z * -1.5f;
		gl::color( ColorAf( 1.0f, depth, 1.0f - depth, depth ) );
		gl::vertex( *pointIt );
	}
	//for_each(mPoints.cbegin(), mPoints.cend(), [](const Vec3f & point)
	//{
	///	glVertex3f(point);
	//});
	glEnd();
	int paused = TRUE;
	glPushMatrix();										
		//CreateCube();
		t_speed = (t_speed+0.0002*(!paused))*0.9;
		t += t_speed;
		glScalef(1.0, -1.0, 1.0);
		glLightfv(GL_LIGHT0, GL_POSITION, appdata.light_pos);
		glTranslatef(-0.5, 2.0, 0.0);
		glRotatef(360.0*t, 4.0, 2.0, 1.0);
		glTranslatef(-1.0, -0.5, -1.0);
		glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
		draw_model(MODE_NORMAL);
	glPopMatrix();	
	/*
	glPushMatrix();										
		glColor3f(1,0,0);
		glTranslatef(0,0,0);							
		glRotatef(g_rotation,0,1,0);
		glRotatef(90,0,1,0);
		// Draw the teapot
	    //glutSolidTeapot(40);
		glutSolidCube(40);
	glPopMatrix();	
	g_rotation += g_rotation_speed;*/
}

void PointCloudApp::CreateCube()		// Here's Where We Do All The Drawing
{
    // Clear Screen And Depth Buffer
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // Reset The Current Modelview Matrix
    //glLoadIdentity();

//NEW//////////////////NEW//////////////////NEW//////////////////NEW/////////////

  glTranslatef(0.0f, 0.0f,0.0f);	// Translate Into The Screen 7.0 Units
  glRotatef(g_rotation,0.0f,1.0f,0.0f);	// Rotate The cube around the Y axis
  glRotatef(g_rotation,1.0f,1.0f,1.0f);
  glBegin(GL_QUADS);		// Draw The Cube Using quads
    glColor3f(0.0f,1.0f,0.0f);	// Color Blue
    glVertex3f( 1.0f, 1.0f,-1.0f);	// Top Right Of The Quad (Top)
    glVertex3f(-1.0f, 1.0f,-1.0f);	// Top Left Of The Quad (Top)
    glVertex3f(-1.0f, 1.0f, 1.0f);	// Bottom Left Of The Quad (Top)
    glVertex3f( 1.0f, 1.0f, 1.0f);	// Bottom Right Of The Quad (Top)
    glColor3f(1.0f,0.5f,0.0f);	// Color Orange
    glVertex3f( 1.0f,-1.0f, 1.0f);	// Top Right Of The Quad (Bottom)
    glVertex3f(-1.0f,-1.0f, 1.0f);	// Top Left Of The Quad (Bottom)
    glVertex3f(-1.0f,-1.0f,-1.0f);	// Bottom Left Of The Quad (Bottom)
    glVertex3f( 1.0f,-1.0f,-1.0f);	// Bottom Right Of The Quad (Bottom)
    glColor3f(1.0f,0.0f,0.0f);	// Color Red	
    glVertex3f( 1.0f, 1.0f, 1.0f);	// Top Right Of The Quad (Front)
    glVertex3f(-1.0f, 1.0f, 1.0f);	// Top Left Of The Quad (Front)
    glVertex3f(-1.0f,-1.0f, 1.0f);	// Bottom Left Of The Quad (Front)
    glVertex3f( 1.0f,-1.0f, 1.0f);	// Bottom Right Of The Quad (Front)
    glColor3f(1.0f,1.0f,0.0f);	// Color Yellow
    glVertex3f( 1.0f,-1.0f,-1.0f);	// Top Right Of The Quad (Back)
    glVertex3f(-1.0f,-1.0f,-1.0f);	// Top Left Of The Quad (Back)
    glVertex3f(-1.0f, 1.0f,-1.0f);	// Bottom Left Of The Quad (Back)
    glVertex3f( 1.0f, 1.0f,-1.0f);	// Bottom Right Of The Quad (Back)
    glColor3f(0.0f,0.0f,1.0f);	// Color Blue
    glVertex3f(-1.0f, 1.0f, 1.0f);	// Top Right Of The Quad (Left)
    glVertex3f(-1.0f, 1.0f,-1.0f);	// Top Left Of The Quad (Left)
    glVertex3f(-1.0f,-1.0f,-1.0f);	// Bottom Left Of The Quad (Left)
    glVertex3f(-1.0f,-1.0f, 1.0f);	// Bottom Right Of The Quad (Left)
    glColor3f(1.0f,0.0f,1.0f);	// Color Violet
    glVertex3f( 1.0f, 1.0f,-1.0f);	// Top Right Of The Quad (Right)
    glVertex3f( 1.0f, 1.0f, 1.0f);	// Top Left Of The Quad (Right)
    glVertex3f( 1.0f,-1.0f, 1.0f);	// Bottom Left Of The Quad (Right)
    glVertex3f( 1.0f,-1.0f,-1.0f);	// Bottom Right Of The Quad (Right)
  glEnd();			// End Drawing The Cube

//NEW//////////////////NEW//////////////////NEW//////////////////NEW/////////////
}

// Handles key press
void PointCloudApp::keyDown( KeyEvent event )
{

	// Key on key...
	switch ( event.getCode() ) {
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	case KeyEvent::KEY_f:
		setFullScreen( !isFullScreen() );
		break;
	case KeyEvent::KEY_s:
		screenShot();
		break;
	}

}

void PointCloudApp::mouseDown( ci::app::MouseEvent event )
{
	mArcball.mouseDown( event.getPos() );
}

void PointCloudApp::mouseDrag( ci::app::MouseEvent event )
{
	mArcball.mouseDrag( event.getPos() );
}

// Prepare window
void PointCloudApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 1024, 768 );
	settings->setFrameRate( 60.0f );
}

// Take screen shot
void PointCloudApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

// Set up
void PointCloudApp::setup()
{
	appdata.win_w = DEF_WIDTH;
	appdata.win_h = DEF_HEIGHT;
	appdata.fullscreen = FALSE;
	appdata.texturing = TRUE;
	appdata.color_texture = FALSE;
	appdata.lighting = TRUE;
	appdata.shadowing = TRUE;
	appdata.reflection = TRUE;
	appdata.light_pos[0] = 0.0;
	appdata.light_pos[1] = 1.0;
	appdata.light_pos[2] = 0.0;
	appdata.light_pos[3] = 0.0;

	mKinect = CinderKinect( CinderKinect::Device() );

	// Set up OpenGL
	gl::enable( GL_DEPTH_TEST );
	glHint( GL_POINT_SMOOTH_HINT, GL_NICEST );
	glEnable( GL_POINT_SMOOTH );
	glPointSize( 0.25f );
	gl::enableAlphaBlending();
	gl::enableAdditiveBlending();

	// Start Kinect with isolated depth tracking only
	//mKinect = Kinect::create();
	//mKinect->start( DeviceOptions().enableSkeletonTracking( false ).enableVideo( false ).setDepthResolution( ImageResolution::NUI_IMAGE_RESOLUTION_640x480 ) );

	// Set up camera
	mArcball = Arcball( getWindowSize() );
	mArcball.setRadius( (float)getWindowHeight() );
	mCamera.lookAt( Vec3f( 0.0f, 0.0f, 670.0f ), Vec3f::zero() );
	mCamera.setPerspective( 60.0f, getWindowAspectRatio(), 0.01f, 5000.0f );

}

uint16_t PointCloudApp::RawDepthToMeters(uint16_t raw)
{
	const float k1 = 1.1863;
	const float k2 = 2842.5;
	const float k3 = 0.1236;
	if (raw >2048)
		return k3 * tanf(raw/k2 + k1);
	return 0;
}

// Called on exit
void PointCloudApp::shutdown()
{
	//mKinect->stop();
	mPoints.clear();
}

// Runs update logic
void PointCloudApp::update()
{

	// Device is capturing
	if ( mKinect.checkNewDepthFrame() ) {

		//mSurface16 = mKinect.getDepthImage();
		// Clear point list
		Vec3f offset( Vec2f( kKinectSize ) * Vec2f( -0.5f, 0.5f ) );
		offset.z = mCamera.getEyePoint().z * 1.0f;
		Vec3f position = Vec3f::zero();
		mPoints.clear();

		// Iterate image rows
		for ( int32_t y = 0; y < kKinectSize.y; y++ ) {
			for ( int32_t x = 0; x < kKinectSize.x; x++ ) {

				// Read depth as 0.0 - 1.0 float
				float depth = mKinect.getDepthAt( Vec2i( x, y ) );
				// Add position to point list
				position.z = depth * mCamera.getEyePoint().z * -3.0f;
				mPoints.push_back( position * Vec3f( 1.0f, -1.0f, 1.5f ) + offset );

				// Shift point
				position.x++;

			}

			// Update position
			position.x = 0.0f;
			position.y++;

		}

	} else {

		// If Kinect initialization failed, try again every 90 frames
		//if ( getElapsedFrames() % 90 == 0 ) {
		//	mKinect->start();
		//}

	}

}

void PointCloudApp::draw_model(int mode) 
{
	if(mode == MODE_SHADOW) {
		glColor4f(0.0,0.0,0.0, 0.8);
	} else {
		if(appdata.lighting) {
			glEnable(GL_LIGHTING);
		}
		if(appdata.texturing) {
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, appdata.tex_model);
		}
		if(!appdata.color_texture) {
			glColor3f(1.0, 1.0, 1.0);
		}
	}

	glBegin(GL_QUADS);
	 /* Top: */
	if(mode == MODE_NORMAL) {
		if(appdata.color_texture) {
			glColor3f(1.0, 0.0, 0.0);
		}
		glNormal3f(0,1,0);
	}

	glTexCoord2i(0,0);
	glVertex3i(0,1,0);
	glTexCoord2i(0,2);
	glVertex3i(0,1,2);
	glTexCoord2i(1,2);
	glVertex3i(1,1,2);
	glTexCoord2i(1,0);
	glVertex3i(1,1,0);

	glTexCoord2i(0,0);
	glVertex3i(1,1,0);
	glTexCoord2i(0,1);
	glVertex3i(1,1,1);
	glTexCoord2i(2,1);
	glVertex3i(3,1,1);
	glTexCoord2i(2,0);
	glVertex3i(3,1,0);

	 /* Front: */
	if(mode == MODE_NORMAL) {
		if(appdata.color_texture) {
			glColor3f(1.0, 1.0, 0.0);
		}
		glNormal3f(0,0,1);
	}

	glTexCoord2i(1,0);
	glVertex3i(0,0,2);
	glTexCoord2i(0,0);
	glVertex3i(1,0,2);
	glTexCoord2i(0,1);
	glVertex3i(1,1,2);
	glTexCoord2i(1,1);
	glVertex3i(0,1,2);

	glTexCoord2i(2,0);
	glVertex3i(1,0,1);
	glTexCoord2i(0,0);
	glVertex3i(3,0,1);
	glTexCoord2i(0,1);
	glVertex3i(3,1,1);
	glTexCoord2i(2,1);
	glVertex3i(1,1,1);

	 /* Left: */
	if(mode == MODE_NORMAL) {
		if(appdata.color_texture) {
			glColor3f(0.0, 1.0, 0.0);
		}
		glNormal3f(-1,0,0);
	}

	glTexCoord2i(0,0);
	glVertex3i(0,0,0);
	glTexCoord2i(0,2);
	glVertex3i(0,0,2);
	glTexCoord2i(1,2);
	glVertex3i(0,1,2);
	glTexCoord2i(1,0);
	glVertex3i(0,1,0);

	 /* Right: */
	if(mode == MODE_NORMAL) {
		if(appdata.color_texture) {
			glColor3f(0.0, 1.0, 1.0);
		}
		glNormal3f(1,0,0);
	}

	glTexCoord2i(1,1);
	glVertex3i(1,0,2);
	glTexCoord2i(1,0);
	glVertex3i(1,0,1);
	glTexCoord2i(0,0);
	glVertex3i(1,1,1);
	glTexCoord2i(0,1);
	glVertex3i(1,1,2);

	glTexCoord2i(1,1);
	glVertex3i(3,0,1);
	glTexCoord2i(1,0);
	glVertex3i(3,0,0);
	glTexCoord2i(0,0);
	glVertex3i(3,1,0);
	glTexCoord2i(0,1);
	glVertex3i(3,1,1);

	 /* Back: */
	if(mode == MODE_NORMAL) {
		if(appdata.color_texture) {
			glColor3f(0.0, 0.0, 1.0);
		}
		glNormal3f(0,0,-1);
	}

	glTexCoord2i(3,0);
	glVertex3i(3,0,0);
	glTexCoord2i(0,0);
	glVertex3i(0,0,0);
	glTexCoord2i(0,1);
	glVertex3i(0,1,0);
	glTexCoord2i(3,1);
	glVertex3i(3,1,0);

	 /* Bottom: */
	if(mode == MODE_NORMAL) {
		if(appdata.color_texture) {
			glColor3f(1.0, 0.0, 1.0);
		}
		glNormal3f(0,-1,0);
	}

	glTexCoord2i(0,0);
	glVertex3i(1,0,0);
	glTexCoord2i(0,2);
	glVertex3i(1,0,2);
	glTexCoord2i(1,2);
	glVertex3i(0,0,2);
	glTexCoord2i(1,0);
	glVertex3i(0,0,0);

	glTexCoord2i(0,0);
	glVertex3i(3,0,0);
	glTexCoord2i(0,1);
	glVertex3i(3,0,1);
	glTexCoord2i(2,1);
	glVertex3i(1,0,1);
	glTexCoord2i(2,0);
	glVertex3i(1,0,0);
	glEnd();

	if(mode == MODE_NORMAL) {
		if(appdata.lighting) {
			glDisable(GL_LIGHTING);
		}
		if(appdata.texturing) {
			glDisable(GL_TEXTURE_2D);
		}
	}
}

// Run application
CINDER_APP_BASIC( PointCloudApp, RendererGl )
