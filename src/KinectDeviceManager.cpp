#include "KinectDeviceManager.h"
#include "KinectDevice.h"

KinectDeviceManager::KinectDeviceManager()
{
	/*
	m_NumDevices = 0;
	m_StopRequested = false;
	checkError( m_Context.Init(), "\n_2Real: Error Could not Initialize OpenNI Context ...\n" );
	m_Context.EnumerateProductionTrees( XN_NODE_TYPE_DEVICE, NULL, m_DeviceInfo, NULL );
	xn::NodeInfoList::Iterator deviceIter = m_DeviceInfo.Begin();
	for ( ; deviceIter!=m_DeviceInfo.End(); ++deviceIter )
	{
		std::stringstream deviceName;
		deviceName << "Device_" << m_NumDevices;
		NodeInfoRef devInfo = NodeInfoRef( new xn::NodeInfo( *deviceIter ) );
		boost::shared_ptr<OpenNIDevice> deviceRef( new OpenNIDevice( m_Context, devInfo,deviceName.str() ) );
		m_Devices.push_back( deviceRef );
		deviceRef->addDeviceToContext();
		m_NumDevices += 1;
	}*/
	//init kinect device
	mKinect = new KinectDevice();
	mDevices.push_back(mKinect);
}

KinectDeviceManager::~KinectDeviceManager()
{
	
	for (unsigned int i=0; i<mDevices.size(); ++i)
	{
		mDevices[i]->shutdown();
		delete mDevices[i];
	}
}

unsigned int KinectDeviceManager::size() const
{
	return mDevices.size();
}

KinectDevice* KinectDeviceManager::operator[](int index)
{
	return mDevices[index];
}