#version 110
uniform sampler2D depthTex, videoTex;
uniform float clip, brightness;

varying vec3 vNormal;
varying float depth;

void main()
{
	if( depth < clip ) discard;
	vec3 colorFrag		= texture2D( depthTex, gl_TexCoord[0].st ).rgb;

	gl_FragColor.rgb	= colorFrag * brightness;
	gl_FragColor.a		= 1.0;
}
