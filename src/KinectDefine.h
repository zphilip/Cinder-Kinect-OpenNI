#ifndef _KinectDefine
#define _KinectDefine

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>
#include <XnVDeviceGenerator.h>
#include <XnVNite.h>
#include <XnTypes.h>
#include <XnV3DVector.h>
#include <vmmlib/vector.hpp>
#include <vmmlib/math.hpp>

namespace Kinect
{

typedef vmml::vector<2, unsigned char> Vector2b;
typedef vmml::vector<3, unsigned char> Vector3b;
typedef vmml::vector<4, unsigned char> Vector4b;

typedef vmml::vector<2, short> Vector2s;
typedef vmml::vector<3, short> Vector3s;
typedef vmml::vector<4, short> Vector4s;

typedef vmml::vector<2, unsigned short> Vector2w;
typedef vmml::vector<3, unsigned short> Vector3w;
typedef vmml::vector<4, unsigned short> Vector4w;    
    
typedef vmml::vector<2, int> Vector2i;
typedef vmml::vector<3, int> Vector3i;
typedef vmml::vector<4, int> Vector4i;

typedef vmml::vector<2, float> Vector2f;
typedef vmml::vector<3, float> Vector3f;
typedef vmml::vector<4, float> Vector4f;
typedef vmml::vector<4, float> Vector6f;

typedef vmml::vector<2, double> Vec2d;
typedef vmml::vector<3, double> Vec3d;
typedef vmml::vector<4, double> Vec4d;
typedef vmml::vector<5, double> Vec6d;

#define SHOW_DEPTH 1
#define SHOW_BAR 0
#define SHOW_USER 1
#define SHOW_SKELETON 0
#define SHOW_IMAGE 1
#define	SHOW_GESTURE 0
#define SHOW_HAND 0

#define XN_CALIBRATION_FILE_NAME "UserCalibration.bin"
#define CONFIG_XML_PATH "Data/SamplesConfig.xml"

#define VALIDATE_GENERATOR(type, desc, generator)				\
{																\
	rc = m_Context.EnumerateExistingNodes(nodes, type);			\
	if (nodes.IsEmpty())										\
{															\
	printf("No %s generator!\n", desc);						\
	return 1;												\
}															\
	(*(nodes.Begin())).GetInstance(generator);					\
}
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
{																\
	printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
	return rc;													\
}

// Note: wont work as expected for > 5 users in scene
static unsigned int g_UsersColors[] = {/*0x70707080*/0 ,0x80FF0000,0x80FF4500,0x80FF1493,0x8000ff00, 0x8000ced1,0x80ffd700};
#define GetColorForUser(i) g_UsersColors[(i)%(sizeof(g_UsersColors)/sizeof(unsigned int))]

enum
{
	KINECT_DEPTH_WIDTH = 640,
	KINECT_DEPTH_HEIGHT = 480,
	KINECT_COLOR_WIDTH = 640,
	KINECT_COLOR_HEIGHT = 480,
	KINECT_MICROPHONE_COUNT = 4,
	KINECT_AUDIO_BUFFER_LENGTH = 1024,
	KINECT_MAX_DEPTH =10000,
};

typedef enum 
{
	DEPTH_OFF,
	LINEAR_HISTOGRAM,
	PSYCHEDELIC,
	PSYCHEDELIC_SHADES,
	RAINBOW,
	CYCLIC_RAINBOW,
	CYCLIC_RAINBOW_HISTOGRAM,
	STANDARD_DEVIATION,
	NUM_OF_DEPTH_TYPES,
	COLOREDDEPTH,
} DepthColoringType;

static XnFloat oniColors[][3] =
{
	{0,1,1},
	{0,0,1},
	{0,1,0},
	{1,1,0},
	{1,0,0},
	{1,.5,0},
	{.5,1,0},
	{0,.5,1},
	{.5,0,1},
	{1,1,.5},
	{0,0,0}
};
static XnUInt32 nColors = 10;

static const unsigned int colorWidth        = KINECT_COLOR_WIDTH;
static const unsigned int colorHeight       = KINECT_COLOR_HEIGHT;
static const unsigned int depthWidth        = KINECT_DEPTH_WIDTH;
static const unsigned int depthHeight       = KINECT_DEPTH_HEIGHT;
static const unsigned int nbMicrophone      = KINECT_MICROPHONE_COUNT;
static const unsigned int audioBufferlength = KINECT_AUDIO_BUFFER_LENGTH;

}

#endif