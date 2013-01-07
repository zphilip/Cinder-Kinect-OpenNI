/*
* 
* Copyright (c) 2011, Ban the Rewind
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
#include <cinder/app/AppBasic.h>
#include "cinder/Arcball.h"
#include <cinder/Camera.h>
#include <cinder/gl/gl.h>
#include <cinder/gl/Texture.h>
#include <cinder/ImageIo.h>
#include <cinder/Vector.h>
#include <cinder/Utilities.h>
#include <CinderKinect.h>

// Imports
using namespace ci;
using namespace ci::app;
using namespace std;

// Kinect SDK point cloud app
class PointCloudApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void draw();
	void keyDown(ci::app::KeyEvent event);
	void prepareSettings(ci::app::AppBasic::Settings * settings);
	void shutdown();
	void setup();
	void update();
	void update1();
	void mouseDown( ci::app::MouseEvent event );
	void mouseDrag( ci::app::MouseEvent event );
	int RawDepthToMeters(int raw);
private:

	// Kinect
	ci::Surface8u mSurface;
	ci::Surface16u mSurface16;
	CinderKinect mKinect;

	// Depth points
	std::vector<ci::Vec3f> mPoints;
	ci::Vec3f mOffset;
	float mScale;
	ci::Vec3f mSpacing;

	// Camera
	ci::CameraPersp mCamera;
	ci::Arcball		mArcball;
	// Save screen shot
	void screenShot();

};

// Render
void PointCloudApp::draw()
{

	// Clear window
	gl::setViewport(getWindowBounds());
	gl::clear(Colorf(0.0f, 0.0f, 0.0f));
	gl::setMatrices(mCamera);
	gl::rotate( mArcball.getQuat() );
	glPushMatrix();
	glScalef(Vec3f(mScale, mScale, 1.0f));

	// Draw points
	gl::color(ColorAf::white());
	glBegin(GL_POINTS);
	//for_each(mPoints.cbegin(), mPoints.cend(), [](const Vec3f & point)
	//{
	//	glVertex3f(point);
	//});
	for ( vector<Vec3f>::const_iterator pointIt = mPoints.cbegin(); pointIt != mPoints.cend(); ++pointIt ) {
		float depth = 1.0f - pointIt->z / mCamera.getEyePoint().z * -1.5f;
		gl::color( ColorAf( 1.0f, depth, 1.0f - depth, depth ) );
		gl::vertex( *pointIt );
	}
	glEnd();
	glPopMatrix();

}

// Handles key press
void PointCloudApp::keyDown(KeyEvent event)
{

	// Key on key...
	switch (event.getCode())
	{
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	case KeyEvent::KEY_f:
		setFullScreen(!isFullScreen());
		break;
	case KeyEvent::KEY_s:
		screenShot();
		break;
	}

}

// Prepare window
void PointCloudApp::prepareSettings(Settings * settings)
{

	// DO IT!
	settings->setWindowSize(800, 600);
	settings->setFrameRate(60.0f);

}

// Quit
void PointCloudApp::shutdown()
{
	// Stop input
	// mKinect.stop();

	// Clean up
	mPoints.clear();
	if (mSurface)
		mSurface.reset();

}

void PointCloudApp::mouseDown( ci::app::MouseEvent event )
{
	mArcball.mouseDown( event.getPos() );
}

void PointCloudApp::mouseDrag( ci::app::MouseEvent event )
{
	mArcball.mouseDrag( event.getPos() );
}

// Take screen shot
void PointCloudApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

// Set up
void PointCloudApp::setup()
{
	mKinect = CinderKinect( CinderKinect::Device() );

	// Set up OpenGL
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_POINT_SMOOTH);
	glPointSize(0.25f);
	gl::enableAlphaBlending();
	gl::enableAdditiveBlending();
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);

	// Start Kinect with isolated depth tracking only
	//mKinect.removeBackground();
	//mKinect.enableSkeletons(false);
	//mKinect.enableVideo(false);
	//mKinect.start();

	// Set up camera
	mCamera.lookAt(Vec3f(0.0f, 0.0f, 0.001f), Vec3f::zero());
	mCamera.setPerspective(60.0f, getWindowAspectRatio(), 1.0f, 1000.0f);
	mArcball = Arcball( getWindowSize() );
	mArcball.setRadius( (float)getWindowHeight() );
	// Point scaling
	mScale = 1.0f;
	mSpacing.set(1.0f / 320.0f, -1.0f / 240.0f, 1.0f / 255.0f);
	mOffset.set(-1.0f, 1.0f, 0.0f);

}

// Runs update logic
void PointCloudApp::update()
{

	// Check for latest depth map
	if (mKinect.checkNewDepthFrame())
	{

		// Get surface
		//mSurface = mKinect.getDepthImage();
		mSurface = mKinect.getDepth();

		// Clear point list
		Vec3f position = Vec3f::zero();
		mPoints.clear();

		// Iterate image rows
		Surface::Iter iter = mSurface.getIter();
		while (iter.line())
		{

			// Iterate rows in pixel
			while (iter.pixel())
			{

				// Get channels
				uint8_t b = iter.b();
				uint8_t g = iter.g();
				uint8_t r = iter.r();

				// This is not black
				if (b + g + r > 0)
				{

					// Choose which channel to use for depth
					uint8_t depth = b;
					if (g > depth)
						depth = g;
					if (r > depth)
						depth = r;

					// Invert depth
					depth = 256 - depth;

					// Add position to point list
					mPoints.push_back(mOffset + position + Vec3f(0.0f, 0.0f, -mSpacing.z * ((float)depth * 5.0f)));

				}

				// Shift point
				position.x += mSpacing.x;

			}

			// Update position
			position.x = 0.0f;
			position.y += mSpacing.y;

		}

	}

}

int PointCloudApp::RawDepthToMeters(int raw)
{
	const float k1 = 1.1863;
	const float k2 = 2842.5;
	const float k3 = 0.1236;
	if (raw >2048)
		return k3 * tanf(raw/k2 + k1);
	return 0;
}

void PointCloudApp::update1()
{

	// Check for latest depth map
	if (mKinect.checkNewDepthFrame())
	{

		// Get surface
		mSurface16 = mKinect.getDepthImage();

		// Clear point list
		Vec3f position = Vec3f::zero();
		mPoints.clear();

		// Iterate image rows
		Surface16u::Iter iter = mSurface16.getIter();
		while (iter.line())
		{

			// Iterate rows in pixel
			while (iter.pixel())
			{

				// Get channels
				uint16_t b = iter.b();
				uint16_t g = iter.g();
				uint16_t r = iter.r();

				// This is not black
				if (b + g + r > 0)
				{

					// Choose which channel to use for depth
					uint16_t depth = b;
					if (g > depth)
						depth = g;
					if (r > depth)
						depth = r;

					// Invert depth
					depth = 256 - depth;

					// Add position to point list
					mPoints.push_back(mOffset + position + Vec3f(0.0f, 0.0f, -mSpacing.z * ((float)depth * 5.0f)));

				}

				// Shift point
				position.x += mSpacing.x;

			}

			// Update position
			position.x = 0.0f;
			position.y += mSpacing.y;

		}

	}

}
// Run application
CINDER_APP_BASIC(PointCloudApp, RendererGl)
