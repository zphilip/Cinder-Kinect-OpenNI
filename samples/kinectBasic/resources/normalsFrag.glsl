#version 110
uniform sampler2D tex;
uniform float xOffset;
uniform float yOffset;

void main()
{
	float sCoord		= gl_TexCoord[0].s;
	float tCoord		= gl_TexCoord[0].t;
	float h  = texture2D( tex, vec2( sCoord, tCoord ) ).a;
	float hL = texture2D( tex, vec2( sCoord + xOffset, tCoord ) ).a;
	float hR = texture2D( tex, vec2( sCoord - xOffset, tCoord ) ).a;
	float hU = texture2D( tex, vec2( sCoord, tCoord + yOffset ) ).a;
	float hD = texture2D( tex, vec2( sCoord, tCoord - yOffset ) ).a;
	vec3 normal = normalize( vec3( ( ( hR - h ) + ( h - hL ) ) * 0.5, 
								   ( ( hD - h ) + ( h - hU ) ) * 0.5, 
								   0.01 ) );
							   
	gl_FragColor.rgb	= normal;
	gl_FragColor.a		= 1.0;
}





