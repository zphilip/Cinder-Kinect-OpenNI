//Original Code by Screamer and Hiero


//transforms
float4x4 tW: WORLD; //the models world matrix
float4x4 tV: VIEW; //view matrix as set via Renderer (DX9)
float4x4 tP: PROJECTION; //projection matrix as set via Renderer (DX9)
float4x4 tWVP: WORLDVIEWPROJECTION;

//texture input
texture Tex <string uiname="Texture";>;
sampler Samp = sampler_state //sampler for doing the texture-lookup
{
   Texture   = (Tex);          //apply a texture to the sampler
   MipFilter = LINEAR;         //sampler states
   MinFilter = LINEAR;
   MagFilter = LINEAR;
};
float4x4 tTex: TEXTUREMATRIX <string uiname="Texture Transform";>;

//texture transformation marked with semantic TEXTUREMATRIX to achieve symmetric transformations


bool colors <string uiname="Users Only to XYZ";> = 0 ;

const float fx_d = 1.0f / 594.21434211923247f;
const float fy_d = 1.0f / 591.04053696870778f;
const float cx_d = 339.30780975300314f;
const float cy_d = 242.73913761751615f;
 
double4 DepthToWorld(int x, int y, double depthValue)
{
   //adapted from http://graphics.stanford.edu/~mdfisher/Kinect.html
 
   //presume we're in mm units
   double4 depthXYZ = 0;
   depthXYZ.b = depthValue / 0.01f;
 
   //data[3] = (System.Convert.ToSingle(x) - cx_d) * data[1] * fx_d;
   depthXYZ.r = (x - cx_d) * fx_d * depthXYZ.b;
   //data[2] = (System.Convert.ToSingle(y) - cy_d) * data[1] * fy_d;
   depthXYZ.g = -(y - cy_d) * fy_d * depthXYZ.b;
 
   depthXYZ.a = (depthXYZ.b > 0.0f ? 1.0f : 0.0f);
   return depthXYZ;
}

struct vs2ps
{
   float4 Pos   : POSITION;
   float4 TexCd : TEXCOORD0;
};
// -------------------------------------------------------------------------
// VERTEXSHADERS
// -------------------------------------------------------------------------

vs2ps VS( float4 Pos    : POSITION, float4 TexCd  : TEXCOORD0 )
{
   //inititalize all fields of output struct with 0
   vs2ps Out = (vs2ps)0;

   //transform position
   Out.Pos = mul(Pos, tWVP);

   //transform texturecoordinates
   Out.TexCd = mul(TexCd, tTex);


   return Out;
}

// -------------------------------------------------------------------------
// PIXELSHADERS:
// -------------------------------------------------------------------------
float4 simpas(vs2ps In): COLOR
{
 float4 col = tex2D(Samp, In.TexCd);
 return col;
}

float4 shift2(vs2ps In): COLOR

{
float4 col = tex2D(Samp, In.TexCd);
////////////////////////////////////////
//Parameter Definition for
//HSV conversion  of Texture
////////////////////////////////////////
double  r, g, b, a;
double  colorMax, colorMin;
double  h=0, s=0, v=0;

//HSV-Value Texture
    r = col[0];
    g = col[1];
    b = col[2];
    a = col[3];

    colorMax = max (r,g);
    colorMax = max (colorMax,b);
    colorMin = min (r,g);
    colorMin = min (colorMin,b);
    v = colorMax;               // this is the result
    
//HSV-Saturation  of Texture
    if (colorMax != 0)
    { s = (colorMax - colorMin) / colorMax; }

//HSV-Hue  of Texture

    if (s == 0) // if not achromatic
    {
        col.r = 0;
        col.g = 0;
        col.b = 0;
        col.a = 0;

    }

 return col;
}

double4 shift(vs2ps In): COLOR

{
double4 col = tex2D(Samp, In.TexCd);
double x = In.TexCd.x * 640.0f;
double y = In.TexCd.y * 480.0f;
////////////////////////////////////////
//Parameter Definition for
//HSV conversion  of Texture
////////////////////////////////////////
double  r, g, b, a;
double  colorMax, colorMin;
double  h=0, s=0, v=0;

//HSV-Value Texture
    r = col[0];
    g = col[1];
    b = col[2];
    a = col[3];

    colorMax = max (r,g);
    colorMax = max (colorMax,b);
    colorMin = min (r,g);
    colorMin = min (colorMin,b);
    v = colorMax;               // this is the result
    
//HSV-Saturation  of Texture
    if (colorMax != 0)
    { s = (colorMax - colorMin) / colorMax; }

//HSV-Hue  of Texture
	if (colors == 0)
	{
		col = DepthToWorld(x, y, v);
	} else {
		if (s != 0)
		col = DepthToWorld(x, y, v);
	}
	
    if (s == 0 && colors) // if not achromatic
    {
        col.r = 0;
        col.g = 0;
        col.b = 0;
        col.a = 0;

    }

 return col;
}


// -------------------------------------------------------------------------
// TECHNIQUES:
// -------------------------------------------------------------------------


technique simplePass //name for the technique pin
{
   pass P0
   {
       VertexShader = compile vs_1_1 VS();
       PixelShader  = compile ps_1_1 simpas();
   }
}

technique OnlyColored //name for the technique pin
{
   pass P0
   {
       VertexShader = compile vs_1_1 VS();
       PixelShader  = compile ps_2_a shift2();
   }
}


technique XYZ //name for the technique pin
{
   pass P0
   {
       VertexShader = compile vs_1_1 VS();
       PixelShader  = compile ps_2_a shift();
   }
}
