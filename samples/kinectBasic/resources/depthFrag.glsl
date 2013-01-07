#version 110

uniform sampler2D video;
uniform sampler2D depth;
uniform float scale;
uniform float xOff, yOff, xAdj;

void main()
{
	float sCoord		= gl_TexCoord[0].s;
	float tCoord		= gl_TexCoord[0].t;
	vec2 texCoord		= vec2( sCoord, tCoord );
	float depth			= texture2D( depth, texCoord ).b;
	
	
	float xOffset		= ( 1.0 - depth ) * xAdj;

	float sDCoord		= clamp( gl_TexCoord[0].s * scale + xOff + xOffset, 0.0, 1.0 );
	float tDCoord		= clamp( gl_TexCoord[0].t * scale + yOff, 0.0, 1.0 );
	vec2 texOffsetCoord	= vec2( sDCoord, tDCoord );
	
	vec3 videoFrag		= texture2D( video, texOffsetCoord ).rgb;

	gl_FragColor.rgb	= videoFrag;
	gl_FragColor.a		= depth;
}
