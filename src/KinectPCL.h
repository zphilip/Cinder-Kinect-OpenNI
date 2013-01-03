#ifndef _KinectPCL
#define _KinectPCL

#include <XnCppWrapper.h>
#include "KinectDefine.h"
#include <stdio.h>

using namespace std;
using namespace xn;
using namespace Kinect;

/** 
* use  1.create object 2.set output mode 3.initilization 4.update and processing 
* ex.. 1.DepthmapPointCloud cloud; 2.cloud.setOutputMode(getDefaultOutputMode()); 3.cloud.init(); 4. ... ;
*/
class DepthmapPointCloud	
{
public:
	DepthmapPointCloud(Context &contex, DepthGenerator &depthGenerator, XnMapOutputMode &outputMode)
						: m_hContex(contex), m_hDepthGenerator(depthGenerator), m_nStatus(XN_STATUS_OK),
						  m_pProjecttiveCloud(0), m_pRealworldCloud(0) 
	{
		m_nOutputMode.nFPS = outputMode.nFPS;
		m_nOutputMode.nXRes= outputMode.nXRes;
		m_nOutputMode.nYRes = outputMode.nYRes;
		m_pProjecttiveCloud = new XnPoint3D[getPointCloudNum()];
		m_pRealworldCloud = new XnPoint3D[getPointCloudNum()];
		m_pManualPointCloud = new XnPoint3D[getPointCloudNum()];
		RawDepthToMeters();
	}	
	~DepthmapPointCloud()
	{
		stop();
	}
	inline XnPoint3D* getPointCloudData()
	{
		return m_pRealworldCloud;
	}
	inline const XnUInt32 getPointCloudNum() const
	{
		return m_nOutputMode.nXRes*m_nOutputMode.nYRes;
	}
	inline const XnMapOutputMode& getXYRes() const 
	{
		return m_nOutputMode;
	}
	inline const XnMapOutputMode getDefaultOutputMode()
	{
		// set the depth map output mode
		XnMapOutputMode outputMode = {XN_VGA_X_RES, XN_VGA_Y_RES, 30};
		return outputMode;
	}
	/** 
	* test: return projective point cloud data
	*/
	inline XnPoint3D* getRawPointData()
	{
		return m_pProjecttiveCloud;
	}

	/** 
	* test: return projective point cloud data
	*/
	inline XnPoint3D* getManualPointData()
	{
		return m_pProjecttiveCloud;
	}
	/** 
	* test: get field of view
	*/
	inline void getFieldofView(XnFieldOfView &FOV)const
	{
		m_hDepthGenerator.GetFieldOfView(FOV);
	}

	void DepthmapPointCloud::stop()
	{
		m_hContex.Release();
		delete [] m_pProjecttiveCloud;
		delete [] m_pRealworldCloud;
	}

	void DepthmapPointCloud::updataPointCloud(xn::DepthMetaData * depthMetaData)
	{
		try
		{
			// printf("DataSize: %d\n", m_hDepthGenerator.GetDataSize());		// test
			// store depthmap data to projective cloudpoint data
			XnUInt32 index, shiftStep;
			unsigned short *tmpGrayPixels = (unsigned short *)depthMetaData->Data();
			for (XnUInt32 row=0; row<m_nOutputMode.nYRes; ++row)
			{
				shiftStep = row*m_nOutputMode.nXRes;
				for (XnUInt32 column=0; column<m_nOutputMode.nXRes; ++column)
				{
					index = shiftStep + column;
					unsigned short rawDepth = tmpGrayPixels[index];
					Kinect::Vector3f v = DepthToWorld(row,column,rawDepth);
					m_pProjecttiveCloud[index].X = (XnFloat)column;
					m_pProjecttiveCloud[index].Y = (XnFloat)row;
					m_pProjecttiveCloud[index].Z = m_hDepthGenerator.GetDepthMap()[index]*0.001f; // mm -> m
					m_pManualPointCloud[index].X = (XnFloat)v.x();
					m_pProjecttiveCloud[index].Y = (XnFloat)v.y();
					m_pProjecttiveCloud[index].Z = (XnFloat)v.z();
				}
			}
			if (m_nStatus != XN_STATUS_OK) throw m_nStatus;
			// convert projective pointcloud to real world pointcloud
			m_hDepthGenerator.ConvertProjectiveToRealWorld(m_nOutputMode.nXRes*m_nOutputMode.nYRes, m_pProjecttiveCloud, m_pRealworldCloud);
		}
		catch (...)
		{
			printError();
			stop();
		}
	}

	Kinect::Vector3f DepthmapPointCloud::DepthToWorld(int x, int y, int depthValue)
	{
		static const double fx_d = 1.0 / 5.9421434211923247e+02;
		static const double fy_d = 1.0 / 5.9104053696870778e+02;
		static const double cx_d = 3.3930780975300314e+02;
		static const double cy_d = 2.4273913761751615e+02;

		Kinect::Vector3f result;
		const double depth = mGammaMap[depthValue];
		result.x() = float((x - cx_d) * depth * fx_d);
		result.y() = float((y - cy_d) * depth * fy_d);
		result.z() = float(depth);
		//printf("World x is: %f, y is: %f, z is %f \n", result.x(), result.y(), result.z());
		return result;
	}

	void DepthmapPointCloud::RawDepthToMeters(void)
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
private:
	Context &m_hContex;
	DepthGenerator &m_hDepthGenerator;
	XnStatus m_nStatus;
	XnMapOutputMode m_nOutputMode;
	XnPoint3D *m_pProjecttiveCloud;
	XnPoint3D *m_pManualPointCloud;
	XnPoint3D *m_pRealworldCloud;
	unsigned long   mGammaMap[2048];

	inline void printError()
	{
		if (m_nStatus != XN_STATUS_OK)
		{
			printf("Error: %s", xnGetStatusString(m_nStatus));
			exit(-1);
		}
	}
	DepthmapPointCloud(const DepthmapPointCloud &rhs);	// don't define copy constructor
	DepthmapPointCloud& operator=(const DepthmapPointCloud &rhs); // don't define assignment operator
};

#endif