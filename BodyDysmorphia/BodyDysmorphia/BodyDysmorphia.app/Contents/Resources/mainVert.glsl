uniform sampler2D depthTex, normalTex;
uniform float fatness;
uniform vec3 eyePos;

varying vec3 vNormal;
varying float depth;

void main()
{
	gl_TexCoord[0]		= gl_MultiTexCoord0;
	vec4 vVertex		= vec4( gl_Vertex );

	depth				= texture2D( depthTex, gl_TexCoord[0].st ).a;
	vNormal				= texture2D( normalTex, gl_TexCoord[0].st ).rgb;

	vVertex.xy			*= 2.5;
	vec3 dirToVertex	= normalize( eyePos - vVertex.xyz );

	vVertex.xyz			+= depth * 500.0 * dirToVertex;
	vVertex.xyz			+= vNormal * fatness * 3.0 * depth;

	
	gl_Position			= gl_ModelViewProjectionMatrix * vVertex;
}
