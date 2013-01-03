#ifndef _KinectDevice
#define _KinectDevice

#include "KinectDefine.h"
#include "KinectPCL.h"

namespace Kinect
{
class KinectDevice{
public:
	virtual XnStatus initPrimeSensor(); 
	KinectDevice();	//kinect init for openni
	KinectDevice(void *internalhandle, void *internalmotorhandle);  // takes usb handle.. never explicitly construct! use kinectfinder!
	virtual ~KinectDevice();
	
	//unclasified functions
	bool Opened();
	void SetMotorPosition(double pos);
	void SetLedMode(int NewMode);
	bool GetAcceleroData(float *x, float *y, float *z);

	//void AddListener(KinectListener *K);
	//void RemoveListener(KinectListener *K);
	//std::vector<KinectListener *> mListeners;
		
	CRITICAL_SECTION mListenersLock;
		
	void *mInternalData;
	
	void DrawGLUTDepthMapTexture();
	void KinectDisconnected();
	void DepthReceived();
	void ColorReceived();
	void AudioReceived();

	void ParseColorBuffer();
	void ParseDepthBuffer();

	//functions about handle the texture and buffer, not classify yet
	void ParseColorDepthData(xn::DepthMetaData *depthMetaData,
							xn::SceneMetaData *sceneMetaData,
							xn::ImageMetaData *imageMetaData);
	void ParseColoredDepthData(xn::DepthMetaData *,DepthColoringType);
	void Parse3DDepthData(xn::DepthMetaData *);
	void drawColorImage();
	Vector3f DepthToWorld(int x, int y, int depthValue);
	Vector2i WorldToColor(const Vector3f &pt);
	//update the frame
	virtual bool Update();  //all buffer and texture is updated

	//get depth image Res
	void GetImageRes(XnUInt16 &xRes, XnUInt16 &yRes);

	//read data frame by frame
	void readFrame();
	void closeDevice();
	void shutdown();

	//return the bufferdata
	void* getKinectColorBufferData() const;
	void* getKinectDepthBufferData() const;
	void* getKinectColoredDepthBufferData() const;

	//simple inline functionalities
	int getWidth() const
	{
		return KINECT_DEPTH_WIDTH;
	}

	int getHeight() const
	{
		return KINECT_DEPTH_HEIGHT;
	}

	size_t getBufferSize() const
	{
		return KINECT_DEPTH_WIDTH*KINECT_DEPTH_HEIGHT;
	}

	//normal inline device functions
	xn::Device* getDevice()
	{
		return m_Device.IsValid() ? &m_Device : NULL;
	}
	xn::DepthGenerator* getDepthGenerator()
	{
		return m_DepthGenerator.IsValid() ? &m_DepthGenerator : NULL;
	}
	xn::ImageGenerator* getImageGenerator()
	{
		return m_ImageGenerator.IsValid() ? &m_ImageGenerator : NULL;
	}
	xn::IRGenerator* getIRGenerator()
	{
		return m_IRGenerator.IsValid() ? &m_IRGenerator : NULL;
	}
	xn::UserGenerator* getUserGenerator()
	{
		return m_UserGenerator.IsValid() ? &m_UserGenerator : NULL;
	}
	xn::AudioGenerator* getAudioGenerator()
	{
		return m_AudioGenerator.IsValid() ? &m_AudioGenerator : NULL;
	}

	xn::DepthMetaData* getDepthMetaData()
	{
		m_DepthGenerator.GetMetaData(depthMetaData);
		return m_DepthGenerator.IsValid() ? &depthMetaData : NULL;
	}
	xn::ImageMetaData* getImageMetaData()
	{
		m_ImageGenerator.GetMetaData(imageMetaData);
		return m_ImageGenerator.IsValid() ? &imageMetaData : NULL;
	}
	xn::IRMetaData* getIRMetaData()
	{
		m_IRGenerator.GetMetaData(irMetaData);
		return m_IRGenerator.IsValid() ? &irMetaData : NULL;
	}
	xn::AudioMetaData* getAudioMetaData()
	{
		m_AudioGenerator.GetMetaData(audioMetaData);
		return m_AudioGenerator.IsValid() ? &audioMetaData : NULL;
	}

    unsigned char* getColorBuffer()
	{
		return mColorBuffer; 
	}
	
	unsigned char* getDepthBuffer()
	{
		return mDepthBuffer; 
	}

	unsigned char* getColoredDepthBuffer()
	{
		return mColoredDepthBuffer; 
	}
	
	DepthmapPointCloud * getDepthPointCloud()
	{
		return m_DepthPointCloud; 
	}

protected:

	xn::Device m_Device;
	xn::Player m_Player;
	xn::Context m_Context;
	xn::ScriptNode m_scriptNode;
	bool mIsWorking;

	//xnCallback hands
	XnCallbackHandle m_hPoseCallbacks;
	XnCallbackHandle m_hUserCallbacks;
	XnCallbackHandle m_hCalibrationCallbacks;
	//xnGenerateors
	xn::ProductionNode* m_pPrimary;
#if SHOW_DEPTH
	xn::DepthGenerator m_DepthGenerator;
#endif
	xn::UserGenerator m_UserGenerator;
	xn::ImageGenerator m_ImageGenerator;
	xn::IRGenerator m_IRGenerator;
	xn::HandsGenerator m_HandsGenerator;
	xn::GestureGenerator m_GestureGenerator;
	xn::AudioGenerator m_AudioGenerator;
	xn::SceneAnalyzer m_SceneAnalyzer;

	XnVSessionManager* m_pSessionManager;
	XnUserID m_candidateID;

	//something about the quit slider, not mandatory thing
	XnVFlowRouter* m_pQuitFlow;
	XnVSelectableSlider1D* m_pQuitSSlider;
	
	//skeleton
	double m_SmoothingFactor;
	int m_SmoothingDelta;
	
	//Kinect MetaData
	xn::SceneMetaData sceneMetaData;
	xn::DepthMetaData depthMetaData;
	xn::ImageMetaData imageMetaData;
	xn::IRMetaData irMetaData;
	xn::AudioMetaData audioMetaData;

	//Kinect Data Buffer
	unsigned long   mGammaMap[2048];
	unsigned char	mDepthBuffer[KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT];  //also tempary depth Pixel for Ogre
	unsigned char	mColorBuffer[KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * 3]; // also tmpeary colore pixel for Ogre
	unsigned char	mUserBuffer[KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * 3]; // also tmpeary colore pixel for Ogre
	unsigned char   mColoredDepthBuffer[KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT * 3]; //also tempeary colored depth pixel for Ogre
	unsigned char   m3DDepthBuffer[KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT * 3]; //also tempeary colored depth pixel for Ogre
	float mAudioBuffer[KINECT_MICROPHONE_COUNT][KINECT_AUDIO_BUFFER_LENGTH];
	float depthHist[KINECT_MAX_DEPTH];
	DepthmapPointCloud *m_DepthPointCloud;

	XnUInt8 PalletIntsR [256];
	XnUInt8 PalletIntsG [256];
	XnUInt8 PalletIntsB [256];
	bool	m_front;

	bool mDepthAvailable;	
	bool mColorAvailable;
	bool mUserAvailable;
	bool mColoredDepthAvailable;
	bool m3DDepthAvailable;

	void RawDepthToMeters3(void);
	void RawDepthToMeters2(void);
	void RawDepthToMeters1(void);
	void CalculateHistogram();
	void CreateRainbowPallet();

};

}
#endif