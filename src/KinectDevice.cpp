
#include "KinectDevice.h"

using namespace Kinect;

/*calibrating the depth camera http://openkinect.org/wiki/Imaging_Information
  approximation is given by St¨¦phane Magnenat in this post: distance = 0.1236 * tan(rawDisparity / 2842.5 + 1.1863) in meters. 
  Adding a final offset term of -0.037 centers the original ROS data. The tan approximation has a sum squared difference of .33 cm while the 1/x approximation is about 1.7 cm.
  Once you have the distance using the measurement above, a good approximation for converting (i, j, z) to (x,y,z) is:
		x = (i - w / 2) * (z + minDistance) * scaleFactor * (w/h)
		y = (j - h / 2) * (z + minDistance) * scaleFactor
		z = z
		Where
		minDistance = -10
		scaleFactor = .0021.
		These values were found by hand.
*/
void KinectDevice::RawDepthToMeters1(void)
{
	const float k1 = 1.1863;
    const float k2 = 2842.5;
    const float k3 = 0.1236;
	for (int i=0; i<2048; i++)
	{
        const float depth = k3 * tanf(i/k2 + k1);
		mGammaMap[i]=depth;
	}
}

void KinectDevice::RawDepthToMeters2(void)
{
	for (int i=0; i<2048; i++)
        mGammaMap[i] = float(1.0 / (double(i) * -0.0030711016 + 3.3309495161));
}

void KinectDevice::RawDepthToMeters3(void)
{
	for (int i=0; i<2048; i++)
		mGammaMap[i] = (unsigned short)(float)(powf(i/2048.0f, 3)*6*6*256);
}

KinectDevice::KinectDevice() 
{
	memset(mDepthBuffer,0,KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT);
	memset(mColorBuffer,0,KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * 3);
	memset(mUserBuffer,0,KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * 3);
	memset(mColoredDepthBuffer,0,KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * 3);
	memset(PalletIntsR,0,256*sizeof(XnUInt8));
	memset(PalletIntsG,0,256*sizeof(XnUInt8));
	memset(PalletIntsB,0,256*sizeof(XnUInt8));

	mColorAvailable = false;
	mDepthAvailable = false;
	mUserAvailable = false;
	mColoredDepthAvailable = false;

	m_hUserCallbacks = NULL;
	m_hPoseCallbacks = NULL;
	m_hCalibrationCallbacks = NULL;
	m_pPrimary = NULL;
	mIsWorking=false; 

	RawDepthToMeters1();
	CreateRainbowPallet();

}

KinectDevice::~KinectDevice()
{
	shutdown();
}

//init Kinect or Xtion
XnStatus KinectDevice::initPrimeSensor()
{
		// Init OpenNI from XML
		XnStatus rc = XN_STATUS_OK;
		xn::NodeInfoList nodes;
        xn::EnumerationErrors errors;

		if(mIsWorking)
			return rc;
        
		rc = m_Context.InitFromXmlFile(CONFIG_XML_PATH, m_scriptNode, &errors);
        if (rc == XN_STATUS_NO_NODE_PRESENT)
        {
            XnChar strError[1024];
            errors.ToString(strError, 1024);
            printf("%s\n", strError);
            return rc;
        }
        CHECK_RC(rc, "Open XML failed");

		rc = m_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, m_DepthGenerator);
		if (rc != XN_STATUS_OK)
		{
			printf("No depth generator found. Using a default one...");
			xn::MockDepthGenerator mockDepth;
			rc = mockDepth.Create(m_Context);
			CHECK_RC(rc, "Create mock depth");

			// set some defaults
			XnMapOutputMode defaultMode;
			defaultMode.nXRes = 320;
			defaultMode.nYRes = 240;
			defaultMode.nFPS = 30;
			rc = mockDepth.SetMapOutputMode(defaultMode);
			CHECK_RC(rc, "set default mode");

			// set FOV
			XnFieldOfView fov;
			fov.fHFOV = 1.0225999419141749;
			fov.fVFOV = 0.79661567681716894;
			rc = mockDepth.SetGeneralProperty(XN_PROP_FIELD_OF_VIEW, sizeof(fov), &fov);
			CHECK_RC(rc, "set FOV");

			XnUInt32 nDataSize = defaultMode.nXRes * defaultMode.nYRes * sizeof(XnDepthPixel);
			XnDepthPixel* pData = (XnDepthPixel*)xnOSCallocAligned(nDataSize, 1, XN_DEFAULT_MEM_ALIGN);

			rc = mockDepth.SetData(1, 0, nDataSize, pData);
			CHECK_RC(rc, "set empty depth map");

			m_DepthGenerator = mockDepth;
		}

		rc = m_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, m_ImageGenerator);
		if (rc != XN_STATUS_OK)
		{
			rc = m_ImageGenerator.Create(m_Context);
			CHECK_RC(rc, "Find image generator");
		}

		rc = m_Context.FindExistingNode(XN_NODE_TYPE_USER, m_UserGenerator);
		if (rc != XN_STATUS_OK && SHOW_USER)
		{
			rc = m_UserGenerator.Create(m_Context);
			CHECK_RC(rc, "Find user generator");
		}

		if (!m_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)&&SHOW_SKELETON)
		{
			printf("Supplied user generator doesn't support skeleton\n");
			return XN_STATUS_ERROR;
		}

		// Make sure we have all OpenNI nodes we will be needing for this sample

#if SHOW_DEPTH
		VALIDATE_GENERATOR(XN_NODE_TYPE_DEPTH, "Depth", m_DepthGenerator);
#endif 
		VALIDATE_GENERATOR(XN_NODE_TYPE_USER, "User", m_UserGenerator);
		VALIDATE_GENERATOR(XN_NODE_TYPE_IMAGE, "Image", m_ImageGenerator);
		VALIDATE_GENERATOR(XN_NODE_TYPE_HANDS, "Gesture", m_GestureGenerator);
		VALIDATE_GENERATOR(XN_NODE_TYPE_HANDS, "Hands", m_HandsGenerator);
		
		// Init NITE Controls (UI stuff)
		m_pSessionManager = new XnVSessionManager;
		rc = m_pSessionManager->Initialize(&m_Context, "Click", "RaiseHand");
		m_pSessionManager->SetQuickRefocusTimeout(0);

		// Init OpenNI nodes and register callback Procedure to OpenNI
		m_HandsGenerator.SetSmoothing(1);
		m_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
		//m_UserGenerator.RegisterUserCallbacks(SinbadCharacterController::NewUser, SinbadCharacterController::LostUser, this, m_hUserCallbacks);
		//m_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(SinbadCharacterController::PoseDetected, SinbadCharacterController::PoseLost, this, m_hPoseCallbacks);
#if SHOW_DEPTH
		m_DepthGenerator.GetMirrorCap().SetMirror(m_front);
#endif
		m_ImageGenerator.GetMirrorCap().SetMirror(m_front);
		// Make sure OpenNI nodes start generating
		rc = m_Context.StartGeneratingAll();
		CHECK_RC(rc, "Kinect StartGenerating Context to Ogre");
		if(rc == XN_STATUS_OK)
			mIsWorking=true; 
		m_candidateID = 0;

		//init the depth point cloud
		XnMapOutputMode m_depth_mode; 
        m_depth_mode.nXRes = XN_VGA_X_RES; 
        m_depth_mode.nYRes = XN_VGA_Y_RES; 
        m_depth_mode.nFPS = 30; 
		m_DepthPointCloud = new DepthmapPointCloud(m_Context,m_DepthGenerator,m_depth_mode);

 		return XN_STATUS_OK;
}

//update the all buffer and texture from kinect
bool KinectDevice::Update()
{
	if (!mIsWorking)
		return false;
	//get meta data from kinect
	readFrame();
	//parse data to texture
	ParseColorDepthData(&depthMetaData,&sceneMetaData,&imageMetaData);
	//ParseColoredDepthData(&depthMetaData,DepthColoringType::RAINBOW);
	//Parse3DDepthData(&depthMetaData);
	//m_DepthPointCloud->updataPointCloud(&depthMetaData);
	return true;
}

void KinectDevice::CalculateHistogram()
{
	xn::DepthGenerator* pDepthGen = getDepthGenerator();

	if (pDepthGen == NULL)
		return;

	XnUInt32 nZRes = pDepthGen->GetDeviceMaxDepth() + 1;
	xnOSMemSet(depthHist, 0, nZRes*sizeof(float));
	int nNumberOfPoints = 0;

	XnDepthPixel nValue;

	const XnDepthPixel* pDepth = pDepthGen->GetDepthMap();
	const XnDepthPixel* pDepthEnd = pDepth + (pDepthGen->GetDataSize() / sizeof(XnDepthPixel));

	while (pDepth != pDepthEnd)
	{
		nValue = *pDepth;

		XN_ASSERT(nValue <= nZRes);

		if (nValue != 0)
		{
			depthHist[nValue]++;
			nNumberOfPoints++;
		}

		pDepth++;
	}

	XnUInt32 nIndex;
	for (nIndex = 1; nIndex < nZRes; nIndex++)
	{
		depthHist[nIndex] += depthHist[nIndex-1];
	}
	for (nIndex = 1; nIndex < nZRes; nIndex++)
	{
		if (depthHist[nIndex] != 0)
		{
			depthHist[nIndex] = (nNumberOfPoints-depthHist[nIndex]) / nNumberOfPoints;
		}
	}
}
// --------------------------------
// Code
// --------------------------------
void KinectDevice::CreateRainbowPallet()
{
	unsigned char r, g, b;
	for (int i=1; i<255; i++)
	{
		if (i<=29)
		{
			r = (unsigned char)(129.36-i*4.36);
			g = 0;
			b = (unsigned char)255;
		}
		else if (i<=86)
		{
			r = 0;
			g = (unsigned char)(-133.54+i*4.52);
			b = (unsigned char)255;
		}
		else if (i<=141)
		{
			r = 0;
			g = (unsigned char)255;
			b = (unsigned char)(665.83-i*4.72);
		}
		else if (i<=199)
		{
			r = (unsigned char)(-635.26+i*4.47);
			g = (unsigned char)255;
			b = 0;
		}
		else
		{
			r = (unsigned char)255;
			g = (unsigned char)(1166.81-i*4.57);
			b = 0;
		}

		PalletIntsR[i] = r;
		PalletIntsG[i] = g;
		PalletIntsB[i] = b;
	}
}

//convertDepthToRGB function
void KinectDevice::ParseColoredDepthData(xn::DepthMetaData *depthMetaData,DepthColoringType DepthColoring)
{
	unsigned short *tmpGrayPixels = (unsigned short *)depthMetaData->Data();
	int MaxDepth = getDepthGenerator()->GetDeviceMaxDepth();
	int i=0;
	//XnUInt8 nAlpha = m_DrawConfig.Streams.Depth.fTransparency*255;

	XnUInt16 nColIndex;
	for (int y=0; y<480; y++)
	{
		unsigned char* destrow = mColoredDepthBuffer + (y*(640))*3;
		for (int x=0; x<640; x++)
		{
			XnUInt8 nRed = 0;
			XnUInt8 nGreen = 0;
			XnUInt8 nBlue = 0;
			unsigned short Depth = tmpGrayPixels[i];
			switch (DepthColoring)
			{
				case LINEAR_HISTOGRAM:
					nRed = nGreen = depthHist[Depth]*255;
					break;
				case PSYCHEDELIC:
					switch ((Depth/10) % 10)
					{
					case 0:
						nRed = 255;
						break;
					case 1:
						nGreen = 255;
						break;
					case 2:
						nBlue = 255;
						break;
					case 3:
						nRed = 255;
						nGreen = 255;
						break;
					case 4:
						nGreen = 255;
						nBlue = 255;
						break;
					case 5:
						nRed = 255;
						nBlue = 255;
						break;
					case 6:
						nRed = 255;
						nGreen = 255;
						nBlue = 255;
						break;
					case 7:
						nRed = 127;
						nBlue = 255;
						break;
					case 8:
						nRed = 255;
						nBlue = 127;
						break;
					case 9:
						nRed = 127;
						nGreen = 255;
						break;
					}
					break;
				case RAINBOW:
					nColIndex = (XnUInt16)((Depth / (MaxDepth / 256.)));
					nRed = PalletIntsR[nColIndex];
					nGreen = PalletIntsG[nColIndex];
					nBlue = PalletIntsB[nColIndex];
					break;
				case CYCLIC_RAINBOW:
					nColIndex = (Depth % 256);
					nRed = PalletIntsR[nColIndex];
					nGreen = PalletIntsG[nColIndex];
					nBlue = PalletIntsB[nColIndex];
					break;
				case CYCLIC_RAINBOW_HISTOGRAM:{
					float fHist = depthHist[Depth];
					nColIndex = (Depth % 256);
					nRed = PalletIntsR[nColIndex]   * fHist;
					nGreen = PalletIntsG[nColIndex] * fHist;
					nBlue = PalletIntsB[nColIndex]  * fHist;
					break;
					}
				case COLOREDDEPTH:	
					unsigned long pval = mGammaMap[Depth];
					int lb = pval & 0xff;
					switch (pval>>8) 
					{
						case 0:
							nRed = 255;
							nGreen = 255-lb;
							nBlue = 255-lb;
							break;
						case 1:
							nRed = 255;
							nGreen = lb;
							nBlue = 0;
							break;
						case 2:
							nRed = 255-lb;
							nGreen = 255;
							nBlue = 0;
							break;
						case 3:
							nRed = 0;
							nGreen = 255;
							nBlue = lb;
							break;
						case 4:
							nRed = 0;
							nGreen = 255-lb;
							nBlue = 255;
							break;
						case 5:
							nRed = 0;
							nGreen = 0;
							nBlue = 255-lb;
							break;
						default:
							nRed = 0;
							nGreen = 0;
							nBlue = 0;
							break;
					}
			}
			destrow[0] = nBlue;
			destrow[1] = nGreen;
			destrow[2] = nRed;
			destrow += 3;
			i++;
		}
	}
	
	//copy?? the data to pixelbox
	mColoredDepthAvailable = true;	
}

void KinectDevice::Parse3DDepthData(xn::DepthMetaData * depthMetaData)
{
	unsigned short *tmpGrayPixels = (unsigned short *)depthMetaData->Data();
	int MaxDepth = getDepthGenerator()->GetDeviceMaxDepth();
	// We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
	int skip = 1;
	for (int y=0; y<Kinect::KINECT_DEPTH_HEIGHT; y+=skip)
	{
		unsigned char* destrow = m3DDepthBuffer + (y*(KINECT_DEPTH_WIDTH))*3;
		for (int x=0; x<Kinect::KINECT_DEPTH_WIDTH; x+=skip)
		{
		    int offset = x+y*KINECT_DEPTH_WIDTH;
			// Convert kinect data to world xyz coordinate
			unsigned short rawDepth = tmpGrayPixels[offset];
			Vector3f v = DepthToWorld(x,y,rawDepth);
			destrow[2] = v.x();
			destrow[1] = v.y();
			destrow[0] = v.z();
			destrow += 3;
		}
	}
	m3DDepthAvailable = false;
}

Vector3f KinectDevice::DepthToWorld(int x, int y, int depthValue)
{
    static const double fx_d = 1.0 / 5.9421434211923247e+02;
    static const double fy_d = 1.0 / 5.9104053696870778e+02;
    static const double cx_d = 3.3930780975300314e+02;
    static const double cy_d = 2.4273913761751615e+02;

    Vector3f result;
    const double depth = mGammaMap[depthValue];;
    result.x() = float((x - cx_d) * depth * fx_d);
    result.y() = float((y - cy_d) * depth * fy_d);
	result.z() = float(depth);
	//printf("World x is: %f, y is: %f, z is %f \n", result.x(), result.y(), result.z());
    return result;
}
/*
Vector2i KinectDevice::WorldToColor(const Vector3f &pt)
{
    static const Matrix4 rotationMatrix(
                            Vector3f(9.9984628826577793e-01f, 1.2635359098409581e-03f, -1.7487233004436643e-02f),
                            Vector3f(-1.4779096108364480e-03f, 9.9992385683542895e-01f, -1.2251380107679535e-02f),
                            Vector3f(1.7470421412464927e-02f, 1.2275341476520762e-02f, 9.9977202419716948e-01f));
    static const Vec3f translation(1.9985242312092553e-02f, -7.4423738761617583e-04f, -1.0916736334336222e-02f);
    static const Matrix4 finalMatrix = rotationMatrix.Transpose() * Matrix4::Translation(-translation);
    
    static const double fx_rgb = 5.2921508098293293e+02;
    static const double fy_rgb = 5.2556393630057437e+02;
    static const double cx_rgb = 3.2894272028759258e+02;
    static const double cy_rgb = 2.6748068171871557e+02;

    const Vector3f transformedPos = finalMatrix.TransformPoint(pt);
    const float invZ = 1.0f / transformedPos.z();

    Vector2i result;
    result.x() = Utility::Bound(cv::Math::Round((transformedPos.x * fx_rgb * invZ) + cx_rgb), 0, 639);
    result.y() = Utility::Bound(Math::Round((transformedPos.y * fy_rgb * invZ) + cy_rgb), 0, 479);
    return result;
}*/

//Parse color&depth Texture
void KinectDevice::ParseColorDepthData(xn::DepthMetaData *depthMetaData,
							xn::SceneMetaData *sceneMetaData,
							xn::ImageMetaData *imageMetaData)
{
	// Calculate the accumulative histogram
	unsigned int nValue = 0;
	unsigned int nHistValue = 0;
	unsigned int nIndex = 0;
	unsigned int nX = 0;
	unsigned int nY = 0;
	unsigned int nNumberOfPoints = 0;
		
	//get the depth data and scene data from kinect
	memset(depthHist, 0, KINECT_MAX_DEPTH*sizeof(float));
	const XnDepthPixel* pDepth = depthMetaData->Data();	

	int n = 0;
	for (nY=0; nY < KINECT_DEPTH_HEIGHT; nY++)
	{
		for (nX=0; nX < KINECT_DEPTH_WIDTH; nX++, nIndex++)
		{
			nValue = pDepth[nIndex];

			if (nValue != 0)
			{
				depthHist[nValue]++;
				nNumberOfPoints++;
			}						
		}
	}
	
	for (nIndex=1; nIndex < KINECT_MAX_DEPTH; nIndex++)
	{
		depthHist[nIndex] += depthHist[nIndex-1];
	}

	if (nNumberOfPoints)
	{
		for (nIndex=1; nIndex < KINECT_MAX_DEPTH; nIndex++)
		{
			depthHist[nIndex] = (unsigned int)(256 * (1.0f - (depthHist[nIndex] / nNumberOfPoints)));
		}
	}
	
	const XnLabel* pLabels = sceneMetaData->Data();
	XnLabel label;

	for (int i = 0; i < KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT; i++)
	{			
		nValue = pDepth[i];
		label = pLabels[i];
		XnUInt32 nColorID = label % nColors;
		if (label == 0)
		{
			nColorID = nColors;
		}

		if (nValue != 0) 
		{
			nHistValue = depthHist[nValue];
			mDepthBuffer[i] = nHistValue;

			mUserBuffer[i * 3 + 0] = 255 * oniColors[nColorID][0];
			mUserBuffer[i * 3 + 1] = 255 * oniColors[nColorID][1];
			mUserBuffer[i * 3 + 2] = 255 * oniColors[nColorID][2];
		}
		else 
		{
			mDepthBuffer[i] = 0;

			mUserBuffer[i * 3 + 0] = 0;
			mUserBuffer[i * 3 + 1] = 0;
			mUserBuffer[i * 3 + 2] = 0;
		}
	}

	const XnRGB24Pixel* pImageRow = imageMetaData->RGB24Data(); // - g_imageMD.YOffset();

	for (XnUInt y = 0; y < Kinect::colorHeight; ++y)
	{
		const XnRGB24Pixel* pImage = pImageRow; // + g_imageMD.XOffset();

		for (XnUInt x = 0; x < Kinect::colorWidth; ++x, ++pImage)
		{
			int index = (y*Kinect::colorWidth + x)*3;
			mColorBuffer[index + 2] = (unsigned char) pImage->nBlue;
			mColorBuffer[index + 1] = (unsigned char) pImage->nGreen;
			mColorBuffer[index + 0] = (unsigned char) pImage->nRed;
		}
		pImageRow += Kinect::colorWidth;
	}

	//copy the data to pixelbox
	mDepthAvailable = true;	
	mColorAvailable = true;
	mUserAvailable = true;
}

/*
void KinectDevice::drawColorImage()
{
	if (g_DrawConfig.Streams.bBackground)
		TextureMapDraw(&g_texBackground, pLocation);

	const xn::MapMetaData* pImageMD;
	const XnUInt8* pImage = NULL;

	pImageMD = getImageMetaData();
	pImage = getImageMetaData()->Data();

	if (pImageMD->FrameID() == 0)
	{
		return;
	}

	const xn::DepthMetaData* pDepthMetaData = getDepthMetaData();

	for (XnUInt16 nY = pImageMD->YOffset(); nY < pImageMD->YRes() + pImageMD->YOffset(); nY++)
	{
		XnUInt8* pTexture = TextureMapGetLine(&g_texImage, nY) + pImageMD->XOffset()*4;

		if (pImageMD->PixelFormat() == XN_PIXEL_FORMAT_YUV422)
		{
			YUV422ToRGB888(pImage, pTexture, pImageMD->XRes()*2, g_texImage.Size.X*g_texImage.nBytesPerPixel);
			pImage += pImageMD->XRes()*2;
		}
		else
		{
			for (XnUInt16 nX = 0; nX < pImageMD->XRes(); nX++, pTexture+=4)
			{
				XnInt32 nDepthIndex = 0;

				if (pDepthMetaData != NULL)
				{
					XnDouble dRealX = (nX + pImageMD->XOffset()) / (XnDouble)pImageMD->FullXRes();
					XnDouble dRealY = nY / (XnDouble)pImageMD->FullYRes();

					XnUInt32 nDepthX = dRealX * pDepthMetaData->FullXRes() - pDepthMetaData->XOffset();
					XnUInt32 nDepthY = dRealY * pDepthMetaData->FullYRes() - pDepthMetaData->YOffset();

					if (nDepthX >= pDepthMetaData->XRes() || nDepthY >= pDepthMetaData->YRes())
					{
						nDepthIndex = -1;
					}
					else
					{
						nDepthIndex = nDepthY*pDepthMetaData->XRes() + nDepthX;
					}
				}

				switch (pImageMD->PixelFormat())
				{
				case XN_PIXEL_FORMAT_RGB24:
					pTexture[0] = pImage[0];
					pTexture[1] = pImage[1];
					pTexture[2] = pImage[2];
					pImage+=3; 
					break;
				case XN_PIXEL_FORMAT_GRAYSCALE_8_BIT:
					pTexture[0] = pTexture[1] = pTexture[2] = *pImage;
					pImage+=1; 
					break;
				case XN_PIXEL_FORMAT_GRAYSCALE_16_BIT:
					XnUInt16* p16 = (XnUInt16*)pImage;
					pTexture[0] = pTexture[1] = pTexture[2] = (*p16) >> 2;
					pImage+=2; 
					break;
				}

				// decide if pixel should be lit or not
				if (g_DrawConfig.Streams.Image.Coloring == DEPTH_MASKED_IMAGE &&
					(pDepthMetaData == NULL || nDepthIndex == -1 || pDepthMetaData->Data()[nDepthIndex] == 0))
				{
					pTexture[3] = 0;
				}
				else
				{
					pTexture[3] = 255;
				}
			}
		}
	}

	TextureMapUpdate(&g_texImage);
	TextureMapDraw(&g_texImage, pLocation);
}

void KinectDevice::DrawGLUTDepthMapTexture()
{
    XnUInt16 g_nXRes; 
    XnUInt16 g_nYRes; 
    GetImageRes(g_nXRes,g_nYRes);

    if (g_bDrawPixels)
    {
        m_pUserTrackerObj->FillTexture(pDepthTexBuf,texWidth,texHeight,g_bDrawBackground);
    }
    else
    {
        xnOSMemSet(pDepthTexBuf, 0, 3*2*g_nXRes*g_nYRes); // makes the texture empty.
    }

    // makes sure we draw the relevant texture
    glBindTexture(GL_TEXTURE_2D, depthTexID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, pDepthTexBuf);

    // Display the OpenGL texture map
    glColor4f(0.75,0.75,0.75,1);

    glEnable(GL_TEXTURE_2D);

    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, 0, texcoords);

    GLfloat verts[8] = { g_nXRes, g_nYRes, g_nXRes, 0, 0, 0, 0, g_nYRes };
    glVertexPointer(2, GL_FLOAT, 0, verts);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

    //TODO: Maybe glFinish needed here instead - if there's some bad graphics crap
    glFlush();
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    glDisable(GL_TEXTURE_2D);
}
*/

void* KinectDevice::getKinectColorBufferData() const
{
		return (unsigned char *)mColorBuffer;
}

void* KinectDevice::getKinectDepthBufferData() const
{
		return (unsigned char *)mDepthBuffer;
}

void* KinectDevice::getKinectColoredDepthBufferData() const
{
		return (unsigned char *)mColoredDepthBuffer;
}

void KinectDevice::GetImageRes(XnUInt16 &xRes, XnUInt16 &yRes)
{
    xn::DepthMetaData depthMD;
    m_DepthGenerator.GetMetaData(depthMD);
    xRes = (XnUInt16)depthMD.XRes();
    yRes = (XnUInt16)depthMD.YRes();
}

void KinectDevice::closeDevice()
{
	m_DepthPointCloud->stop();
	m_DepthGenerator.Release();
	m_ImageGenerator.Release();
	m_IRGenerator.Release();
	m_AudioGenerator.Release();
	m_scriptNode.Release();
	m_Device.Release();
	m_Player.Release();
	m_Context.Release();
}

void KinectDevice::shutdown()
{
	if (mIsWorking)
		closeDevice();
	mIsWorking = false;
}
void KinectDevice::readFrame()
{
	XnStatus rc = XN_STATUS_OK;

	if (m_pPrimary != NULL)
	{
		rc = m_Context.WaitOneUpdateAll(*m_pPrimary);
	}
	else
	{
		rc = m_Context.WaitAnyUpdateAll();
	}

	if (rc != XN_STATUS_OK)
	{
		printf("Error: %s\n", xnGetStatusString(rc));
	}

	if (m_DepthGenerator.IsValid())
	{
		m_DepthGenerator.GetMetaData(depthMetaData);
	}

	if (m_ImageGenerator.IsValid())
	{
		m_ImageGenerator.GetMetaData(imageMetaData);
	}

	if (m_IRGenerator.IsValid())
	{
		m_IRGenerator.GetMetaData(irMetaData);
	}

	if (m_AudioGenerator.IsValid())
	{
		m_AudioGenerator.GetMetaData(audioMetaData);
	}
	if (m_UserGenerator.IsValid())
	{
		m_UserGenerator.GetUserPixels(0, sceneMetaData);
	}
}


