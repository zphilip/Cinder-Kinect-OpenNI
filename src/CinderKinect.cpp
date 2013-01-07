/*
 Copyright (c) 2010, The Cinder Project, All rights reserved.

 This code is intended for use with the Cinder C++ library: http://libcinder.org

 Portions copyright Rui Madeira: http://ruim.pt

 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
	the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
	the following disclaimer in the documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/


#include "CinderKinect.h"

using namespace std;

namespace cinder {

// statics
std::mutex			CinderKinect::sContextMutex;

class ImageSourceKinectColor : public ImageSource {
  public:
	ImageSourceKinectColor( uint8_t *buffer, shared_ptr<CinderKinect::Obj> ownerObj )
		: ImageSource(), mData( buffer ), mOwnerObj( ownerObj )
	{
		setSize( 640, 480 );
		setColorModel( ImageIo::CM_RGB );
		setChannelOrder( ImageIo::RGB );
		setDataType( ImageIo::UINT8 );
	}

	~ImageSourceKinectColor()
	{
		// let the owner know we are done with the buffer
		mOwnerObj->mColorBuffers.derefBuffer( mData );
	}

	virtual void load( ImageTargetRef target )
	{
		ImageSource::RowFunc func = setupRowFunc( target );
		
		for( int32_t row = 0; row < 480; ++row )
			((*this).*func)( target, row, mData + row * 640 * 3 );
	}
	
  protected:
	shared_ptr<CinderKinect::Obj>		mOwnerObj;
	uint8_t						*mData;
};

class ImageSourceKinectInfrared : public ImageSource {
  public:
	ImageSourceKinectInfrared( uint8_t *buffer, shared_ptr<CinderKinect::Obj> ownerObj )
		: ImageSource(), mData( buffer ), mOwnerObj( ownerObj )
	{
		setSize( 640, 480 );
		setColorModel( ImageIo::CM_GRAY );
		setChannelOrder( ImageIo::Y );
		setDataType( ImageIo::UINT8 );
	}

	~ImageSourceKinectInfrared()
	{
		// let the owner know we are done with the buffer
		mOwnerObj->mColorBuffers.derefBuffer( mData );
	}

	virtual void load( ImageTargetRef target )
	{
		ImageSource::RowFunc func = setupRowFunc( target );
		
		for( int32_t row = 0; row < 480; ++row )
			((*this).*func)( target, row, mData + row * 640 * 1 );
	}
	
  protected:
	shared_ptr<CinderKinect::Obj>		mOwnerObj;
	uint8_t								*mData;
};

class ImageSourceKinectDepth : public ImageSource {
  public:
	ImageSourceKinectDepth( uint16_t *buffer, shared_ptr<CinderKinect::Obj> ownerObj )
		: ImageSource(), mData( buffer ), mOwnerObj( ownerObj )
	{
		setSize( 640, 480 );
		setColorModel( ImageIo::CM_GRAY );
		setChannelOrder( ImageIo::Y );
		setDataType( ImageIo::UINT16 );
	}

	~ImageSourceKinectDepth()
	{
		// let the owner know we are done with the buffer
		mOwnerObj->mDepthBuffers.derefBuffer( mData );
	}

	virtual void load( ImageTargetRef target )
	{
		ImageSource::RowFunc func = setupRowFunc( target );
		
		for( int32_t row = 0; row < 480; ++row )
			((*this).*func)( target, row, mData + row * 640 );
	}
	
  protected:
	shared_ptr<CinderKinect::Obj>		mOwnerObj;
	uint16_t					*mData;
};

// Used as the deleter for the shared_ptr returned by getImageData() and getDepthData()
template<typename T>
class KinectDataDeleter {
  public:
	KinectDataDeleter( CinderKinect::Obj::BufferManager<T> *bufferMgr, shared_ptr<CinderKinect::Obj> ownerObj )
		: mBufferMgr( bufferMgr ), mOwnerObj( ownerObj )
	{}
	
	void operator()( T *data ) {
		mBufferMgr->derefBuffer( data );
	}
	
	shared_ptr<CinderKinect::Obj>			mOwnerObj; // to prevent deletion of our parent Obj
	CinderKinect::Obj::BufferManager<T> *mBufferMgr;
};

CinderKinect::CinderKinect( Device device )
	: mObj( new Obj( device.mIndex ) )
{
}

CinderKinect::Obj::Obj( int deviceIndex )
	: mShouldDie( false ), mNewVideoFrame( false ), mNewDepthFrame( false ), 
		mColorBuffers( 640 * 480 * 3, this ), mDepthBuffers( 640 * 480, this ), mVideoInfrared( false )
{
	mDevice = new Kinect::KinectDevice();
	mDevice->initPrimeSensor();
	mLastVideoFrameInfrared = mVideoInfrared;
	mThread = shared_ptr<thread>( new thread( threadedFunc, this ) );
	// Initialize depth image
	mDepthSurface16u	= Surface16u( 640, 480, false, SurfaceChannelOrder::RGB );
	mDepthSurface8u		= Surface8u(640, 480, false, SurfaceChannelOrder::RGB);
	mRgbDepth16u		= new Pixel16u[ 640 * 480 * 3 ];
	mRgbDepth8u			= new Pixel[640 * 480 * 3];
	mBinary				= false;
	mFlipped			= false;
	mGreyScale			= false;
	mInverted			= false;
	mRemoveBackground	= false;
}

CinderKinect::Obj::~Obj()
{
	mShouldDie = true;
	mThread->join();
}

void CinderKinect::Obj::colorImageCB( CinderKinect::Obj* kinectObj, const void *rgb)
{
	lock_guard<recursive_mutex> lock( kinectObj->mMutex );

	kinectObj->mColorBuffers.derefActiveBuffer();					// finished with current active buffer
	uint8_t *destPixels = kinectObj->mColorBuffers.getNewBuffer();	// request a new buffer
	if( kinectObj->mVideoInfrared )
		memcpy( destPixels, rgb, 640 * 480 * sizeof(uint8_t) );		// blast the pixels in
	else
		memcpy( destPixels, rgb, 640 * 480 * 3 * sizeof(uint8_t) );		// blast the pixels in
	kinectObj->mColorBuffers.setActiveBuffer( (uint8_t *)destPixels );	// set this new buffer to be the current active buffer
	kinectObj->mNewVideoFrame = true;								// flag that there's a new color frame
	kinectObj->mLastVideoFrameInfrared = kinectObj->mVideoInfrared;
}

void CinderKinect::Obj::depthImageCB( CinderKinect::Obj* kinectObj, const void *d)
{
	lock_guard<recursive_mutex> lock( kinectObj->mMutex );

	//uint16_t *depth = reinterpret_cast<uint16_t*>( d );
	uint16_t *depth = (uint16_t*) ( d );

	kinectObj->mDepthBuffers.derefActiveBuffer();					// finished with current active buffer
	uint16_t *destPixels = kinectObj->mDepthBuffers.getNewBuffer(); // request a new buffer
	/*
	for( size_t p = 0; p < 640 * 480; ++p ) {						// out = 1.0 - ( in / 2048 ) ^ 2
		uint32_t v = depth[p];
		destPixels[p] = 65535 - ( v * v ) >> 4;						// 1 / ( 2^10 * 2^10 ) * 2^16 = 2^-4
	}*/
	memcpy( destPixels, d, 640 * 480 * sizeof(uint16_t) );	
	pixelToDepthSurface( (uint16_t*)d );
	pixelToSurface8u(mDepthSurface8u, (uint8_t *)d, true);
	kinectObj->mNewDepthFrame = true;								// flag that there's a new depth frame
	kinectObj->mDepthBuffers.setActiveBuffer( (uint16_t *)destPixels );			// set this new buffer to be the current active buffer
}

void CinderKinect::threadedFunc( CinderKinect::Obj *kinectObj )
{
	while( ! kinectObj->mShouldDie )
	{
		kinectObj->mDevice->Update();
		xn::DepthMetaData * pDepthData = kinectObj->mDevice->getDepthMetaData();
		kinectObj->mMaxDepth = kinectObj->mDevice->getDepthGenerator()->GetDeviceMaxDepth();
		const XnDepthPixel* pDepthPixel = pDepthData->Data();
		kinectObj->colorImageCB(kinectObj,kinectObj->mDevice->getColorBuffer() );
		kinectObj->depthImageCB(kinectObj,pDepthPixel);
	}
}

int	CinderKinect::getNumDevices()
{ 
	return 1;
}

bool CinderKinect::checkNewVideoFrame()
{
	lock_guard<recursive_mutex> lock( mObj->mMutex );
	bool oldValue = mObj->mNewVideoFrame;
	mObj->mNewVideoFrame = false;
	return oldValue;
}

bool CinderKinect::checkNewDepthFrame()
{
	lock_guard<recursive_mutex> lock( mObj->mMutex );
	bool oldValue = mObj->mNewDepthFrame;
	mObj->mNewDepthFrame = false;
	return oldValue;
}

ImageSourceRef CinderKinect::getVideoImage()
{
	// register a reference to the active buffer
	uint8_t *activeColor = mObj->mColorBuffers.refActiveBuffer();
	if( mObj->mLastVideoFrameInfrared )
		return ImageSourceRef( new ImageSourceKinectInfrared( activeColor, this->mObj ) );
	else
		return ImageSourceRef( new ImageSourceKinectColor( activeColor, this->mObj ) );
}

ImageSourceRef CinderKinect::getDepthImage()
{
	// register a reference to the active buffer
	uint16_t *activeDepth = mObj->mDepthBuffers.refActiveBuffer();
	return ImageSourceRef( new ImageSourceKinectDepth( activeDepth, this->mObj ) );
}

// Get depth surface
Surface8u CinderKinect::getDepth() 
{ 
	// Lock thread
	lock_guard<recursive_mutex> lock( this->mObj->mMutex );
	
	// Return surface and turn off new flag
	this->mObj->mNewDepthFrame = false;
	return this->mObj->mDepthSurface8u;

}

float CinderKinect::getDepthAt( const ci::Vec2i &pos )
{
	float depthNorm		= 0.0f;
	if ( this->mObj->mDepthSurface16u ) {
		//float depthraw = RawDepthToMeters(this->mObj->mDepthSurface.getPixel( pos ).r)*1000;
		uint16_t depthraw = this->mObj->mDepthSurface16u.getPixel( pos ).r;
		uint16_t depth	= 0x10000 - depthraw;
		depth			= depth;
		depthNorm		= 1.0f - (float)depth / 65535.0f;
	}
	return depthNorm;
}

std::shared_ptr<uint8_t> CinderKinect::getVideoData()
{
	// register a reference to the active buffer
	uint8_t *activeColor = mObj->mColorBuffers.refActiveBuffer();
	return shared_ptr<uint8_t>( activeColor, KinectDataDeleter<uint8_t>( &mObj->mColorBuffers, mObj ) );	
}

std::shared_ptr<uint16_t> CinderKinect::getDepthData()
{
	// register a reference to the active buffer
	uint16_t *activeDepth = mObj->mDepthBuffers.refActiveBuffer();
	return shared_ptr<uint16_t>( activeDepth, KinectDataDeleter<uint16_t>( &mObj->mDepthBuffers, mObj ) );	
}

float CinderKinect::RawDepthToMeters(uint16_t raw)
{
	const float k1 = 1.1863;
	const float k2 = 2842.5;
	const float k3 = 0.1236;
	if (raw >2048)
		return k3 * tanf(raw/k2 + k1);
	return 0;
}

void CinderKinect::Obj::pixelToDepthSurface( uint16_t *buffer )
{
	int32_t height		= mDepthSurface16u.getHeight();
	int32_t width		= mDepthSurface16u.getWidth();
	int32_t size		= width * height * 6; // 6 is 3 color channels * sizeof( uint16_t )

	Pixel16u* rgbRun	= mRgbDepth16u;
	uint16_t* bufferRun	= buffer;
	memset(mRgbDepth16u, 0, 640*480*3*sizeof(uint16_t));

	if ( mFlipped ) {
		for ( int32_t y = 0; y < height; y++ ) {
			for ( int32_t x = 0; x < width; x++ ) {
				bufferRun		= buffer + ( y * width + ( ( width - x ) - 1 ) );
				rgbRun			= mRgbDepth16u + ( y * width + x );
				*rgbRun			= shortToPixel( *bufferRun );
			}
		}
	} else {
		for ( int32_t i = 0; i < width * height; i++ ) {
			Pixel16u pixel = shortToPixel( *bufferRun );
			bufferRun++;
			*rgbRun = pixel;
			rgbRun++;
		}
	}

	memcpy( mDepthSurface16u.getData(), mRgbDepth16u, size );
}

// Convert and copy pixel data to a surface
void CinderKinect::Obj::pixelToSurface8u(Surface8u & surface, uint8_t * buffer, bool depth)
{

	// Get dimensions
	int32_t height = surface.getHeight();
	int32_t width = surface.getWidth();
	int32_t size = width * height * 3;

	// This is depth data
	if (depth)
	{
		// Draw the bits to the bitmap
		Pixel * rgbRun = mRgbDepth8u;
		uint16_t * bufferRun = (uint16_t *)buffer;
		for (int32_t y = 0; y < height; y++)
			for (int32_t x = 0 ; x < width; x++)
			{
				Pixel pixel = shortToPixel8u(* bufferRun);
				bufferRun++;
				* rgbRun = pixel;
				rgbRun++;
			}

		// Copy depth data to surface
		memcpy(surface.getData(), (uint8_t *)mRgbDepth8u, size);
		mNewDepthFrame = true;

	}
	else
	{

		// Swap red/blue channels
		for (int32_t i = 0; i < size; i += 4)
		{
			uint8_t b = buffer[i];
			buffer[i] = buffer[i + 2];
			buffer[i + 2] = b;
		}

		// Copy color data to surface
		memcpy(surface.getData(), buffer, size);
		mNewVideoFrame = true;

	}

}

CinderKinect::Pixel16u CinderKinect::Obj::shortToPixel( uint16_t value )
{
	//Extract depth and user values
	//uint16_t depth = 0xFFFF - 0x10000 * ( ( value & 0xFFF8 ) >> 3 ) / 0xFFFF;
	uint16_t depth = 0xFFFF - 0x10000 * (value) / this->mMaxDepth;
	//uint16_t user = value & 7;
	uint16_t user = 7;
	
	CinderKinect::Pixel16u pixel;
	pixel.b = 0;
	pixel.g = 0;
	pixel.r = 0;

	// Binary mode
	if ( mBinary ) {

		// Set black and white values
		uint16_t backgroundColor = mInverted ? 0xFFFF : 0;
		uint16_t userColor = mInverted ? 0 : 0xFFFF;

		// Set color
		if ( user == 0 || user == 7 ) {
			pixel.r = pixel.g = pixel.b = mRemoveBackground ? backgroundColor : userColor;
		} else {
			pixel.r = pixel.g = pixel.b = userColor;
		}

	} else if ( mGreyScale ) {

		// Set greyscale value
		if ( user == 0 || user == 7 ) {
			pixel.r = mRemoveBackground ? 0 : depth;
		} else {
			pixel.r = depth;
		}
		pixel.g = pixel.r;
		pixel.b = pixel.g;

	} else {

		// Colorize each user
		switch ( user ) {
		case 0:
			if ( !mRemoveBackground ) {
				pixel.r = depth / 4;
				pixel.g = pixel.r;
				pixel.b = pixel.g;
			}
			break;
		case 1:
			pixel.r = depth;
			break;
		case 2:
			pixel.r = depth;
			pixel.g = depth;
			break;
		case 3:
			pixel.r = depth;
			pixel.b = depth;
			break;
		case 4:
			pixel.r = depth;
			pixel.g = depth / 2;
			break;
		case 5:
			pixel.r = depth;
			pixel.b = depth / 2;
			break;
		case 6:
			pixel.r = depth;
			pixel.g = depth / 2;
			pixel.b = pixel.g;
			break;
		case 7:
			if ( !mRemoveBackground ) {
				pixel.r = 0xFFFF - ( depth / 2 );
				pixel.g = pixel.r;
				pixel.b = pixel.g;
			}
		}

	}

	// Invert image
	pixel.r = 0xFFFF - pixel.r;
	pixel.g = 0xFFFF - pixel.g;
	pixel.b = 0xFFFF - pixel.b;

	return pixel;
}


// Convert value to short to pixel
CinderKinect::Pixel CinderKinect::Obj::shortToPixel8u(uint16_t value)
{

	// Extract depth and user values
    uint16_t realDepth = value;
    uint16_t user = 0;

    // Transform 13-bit depth information into an 8-bit intensity appropriate
    // for display (we disregard information in most significant bit)
    uint8_t intensity = 255 - (uint8_t)(256 * realDepth / this->mMaxDepth);

	// Initialize pixel value
    Pixel pixel;
	pixel.b = 0;
	pixel.g = 0;
	pixel.r = 0;

	// Binary mode
	if (mBinary)
	{
		
		// Set black and white values
		uint8_t backgroundColor = mInverted ? 255 : 0;
		uint8_t userColor = mInverted ? 0 : 255;

		// Set color
		if (user == 0 || user == 7)
			pixel.r = pixel.b = pixel.g = mRemoveBackground ? backgroundColor : userColor;
		else
			pixel.r = pixel.b = pixel.g = userColor;

	}
	else
	{

		// Colorize each user
		switch (user)
		{
		case 0:
			if (!mRemoveBackground)
			{
				pixel.r = intensity / 2;
				pixel.b = intensity / 2;
				pixel.g = intensity / 2;
			}
			break;
		case 1:
			pixel.r = intensity;
			break;
		case 2:
			pixel.g = intensity;
			break;
		case 3:
			pixel.r = intensity / 4;
			pixel.g = intensity;
			pixel.b = intensity;
			break;
		case 4:
			pixel.r = intensity;
			pixel.g = intensity;
			pixel.b = intensity / 4;
			break;
		case 5:
			pixel.r = intensity;
			pixel.g = intensity / 4;
			pixel.b = intensity;
			break;
		case 6:
			pixel.r = intensity / 2;
			pixel.g = intensity / 2;
			pixel.b = intensity;
			break;
		case 7:
			if (!mRemoveBackground)
			{
				pixel.r = 255 - (intensity / 2);
				pixel.g = 255 - (intensity / 2);
				pixel.b = 255 - (intensity / 2);
			}
		}

	}

	// Return pixel
    return pixel;

}
// ************************************** Buffer management
template<typename T>
CinderKinect::Obj::BufferManager<T>::~BufferManager()
{
	for( typename map<T*,size_t>::iterator bufIt = mBuffers.begin(); bufIt != mBuffers.end(); ++bufIt ) {
		delete [] bufIt->first;
	}
}

template<typename T>
T* CinderKinect::Obj::BufferManager<T>::getNewBuffer()
{
	lock_guard<recursive_mutex> lock( mKinectObj->mMutex );

	typename map<T*,size_t>::iterator bufIt;
	for( bufIt = mBuffers.begin(); bufIt != mBuffers.end(); ++bufIt ) {
		if( bufIt->second == 0 ) // 0 means free buffer
			break;
	}
	if( bufIt != mBuffers.end() ) {
		bufIt->second = 1;
		return bufIt->first;
	}
	else { // there were no available buffers - add a new one and return it
		T *newBuffer = new T[mAllocationSize];
		mBuffers[newBuffer] = 1;
		return newBuffer;
	}
}

template<typename T>
void CinderKinect::Obj::BufferManager<T>::setActiveBuffer( T *buffer )
{
	lock_guard<recursive_mutex> lock( mKinectObj->mMutex );
	// assign new active buffer
	mActiveBuffer = buffer;
}

template<typename T>
T* CinderKinect::Obj::BufferManager<T>::refActiveBuffer()
{
	lock_guard<recursive_mutex> lock( mKinectObj->mMutex );
	mBuffers[mActiveBuffer]++;
	return mActiveBuffer;
}

template<typename T>
void CinderKinect::Obj::BufferManager<T>::derefActiveBuffer()
{
	lock_guard<recursive_mutex> lock( mKinectObj->mMutex );
	if( mActiveBuffer ) 	// decrement use count on current active buffer
		mBuffers[mActiveBuffer]--;		
}

template<typename T>
void CinderKinect::Obj::BufferManager<T>::derefBuffer( T *buffer )
{
	lock_guard<recursive_mutex> lock( mKinectObj->mMutex );
	mBuffers[buffer]--;
}

} // namespace cinder