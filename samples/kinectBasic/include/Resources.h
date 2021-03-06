#pragma once
#include "cinder/CinderResources.h"

#define RES_VERT_ID				CINDER_RESOURCE( ../resources/, mainVert.glsl,	128, GLSL )
#define RES_FRAG_ID				CINDER_RESOURCE( ../resources/, mainFrag.glsl,	129, GLSL )
#define RES_IMAGE_BACKGROUND		CINDER_RESOURCE( ../resources/, zigzag.png, 128, PNG )
#define RES_SHADER_USER_FRAG		CINDER_RESOURCE( ../resources/, user.frag, 129, GLSL )
#define RES_SHADER_USER_VERT		CINDER_RESOURCE( ../resources/, user.vert, 130, GLSL )

//#define RES_MY_RES			CINDER_RESOURCE( ../resources/, image_name.png, 128, IMAGE )
