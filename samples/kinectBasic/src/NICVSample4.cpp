#include <opencv2/opencv.hpp>

#ifdef _DEBUG
#pragma comment(lib, "opencv_imgproc243d.lib")
#pragma comment(lib, "opencv_video243d.lib")
#pragma comment(lib, "opencv_ts243d.lib")
#pragma comment(lib, "opencv_objdetect243d.lib")
#pragma comment(lib, "opencv_ml243d.lib")
#pragma comment(lib, "opencv_legacy243d.lib")
#pragma comment(lib, "opencv_imgproc243d.lib")
#pragma comment(lib, "opencv_highgui243d.lib")
#pragma comment(lib, "opencv_haartraining_engine.lib")
#pragma comment(lib, "opencv_gpu243d.lib")
//#pragma comment(lib, "opencv_ffmpeg243d.lib")
#pragma comment(lib, "opencv_features2d243d.lib")
#pragma comment(lib, "opencv_core243d.lib")
#pragma comment(lib, "opencv_contrib243d.lib")
#pragma comment(lib, "opencv_calib3d243d.lib")
#else
#pragma comment(lib, "opencv_imgproc220.lib")
#pragma comment(lib, "opencv_video220.lib")
#pragma comment(lib, "opencv_ts220.lib")
#pragma comment(lib, "opencv_objdetect220.lib")
#pragma comment(lib, "opencv_ml220.lib")
#pragma comment(lib, "opencv_legacy220.lib")
#pragma comment(lib, "opencv_imgproc220.lib")
#pragma comment(lib, "opencv_highgui220.lib")
#pragma comment(lib, "opencv_haartraining_engine.lib")
#pragma comment(lib, "opencv_gpu220.lib")
#pragma comment(lib, "opencv_ffmpeg220.lib")
#pragma comment(lib, "opencv_features2d220.lib")
#pragma comment(lib, "opencv_core220.lib")
#pragma comment(lib, "opencv_contrib220.lib")
#pragma comment(lib, "opencv_calib3d220.lib")
#endif
using namespace cv;

#include <XnCppWrapper.h>
//#pragma comment(lib,"C:/Program files/OpenNI/Lib/openNI.lib")

#define SAMPLE_XML_PATH "Data/SamplesConfig.xml"
//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
	printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
	return rc;													\
	}

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

using namespace xn;

template <class T>
	static void fillOcclusionInv_(Mat& src, const T invalidvalue, const T minval)
	{
		int bb=1;
		const int MAX_LENGTH=src.cols*0.25;
#pragma omp parallel for
		for(int j=bb;j<src.rows-bb;j++)
		{
			T* s = src.ptr<T>(j);
			
			s[0]=minval;//可能性のある最小を入力
			s[src.cols-1]=minval;//可能性のある最小を入力
			//もし視差が0だったら値の入っている近傍のピクセル（エピポーラ線上）の最小値で埋める
			for(int i=0;i<src.cols;i++)
			{
				if(s[i]==invalidvalue)
				{
					int t=i;
					do
					{
						t++;
						if(t>src.cols-1)break;
					}while(s[t]==invalidvalue);

					const T dd = max(s[i-1],s[t]);
					if(t-i>MAX_LENGTH)
					{
						for(int n=0;n<src.cols;n++)
						{
							s[n]=invalidvalue;
						}
					}
					else
					{
						for(;i<t;i++)
						{
							s[i]=dd;
						}
					}
				}
			}
		}
	}

	template <class T>
	static void fillOcclusion_(Mat& src, const T invalidvalue, const T maxval)
	{
		int bb=1;
		const int MAX_LENGTH=src.cols*0.25;
#pragma omp parallel for
		for(int j=bb;j<src.rows-bb;j++)
		{
			T* s = src.ptr<T>(j);
			s[0]=maxval;//可能性のある最大値を入力
			s[src.cols-1]=maxval;//可能性のある最大値を入力
			//もし視差が0だったら値の入っている近傍のピクセル（エピポーラ線上）の最小値で埋める
			for(int i=0;i<src.cols;i++)
			{
				if(s[i]<=invalidvalue)
				{
					int t=i;
					do
					{
						t++;
						if(t>src.cols-1)break;
					}while(s[t]<=invalidvalue);

					const T dd = min(s[i-1],s[t]);
					if(t-i>MAX_LENGTH)
					{
						for(int n=0;n<src.cols;n++)
						{
							s[n]=invalidvalue;
						}
					}
					else
					{
						for(;i<t;i++)
						{
							s[i]=dd;
						}
					}
				}
			}
		}
	}
	void fillOcclusion(Mat& src, int invalidvalue, bool isInv)
	{
		if(isInv)
		{
			if(src.type()==CV_8U)
			{
				fillOcclusionInv_<uchar>(src, (uchar)invalidvalue,0);
			}
			else if(src.type()==CV_16S)
			{
				fillOcclusionInv_<short>(src, (short)invalidvalue,SHRT_MIN);
			}
			else if(src.type()==CV_16U)
			{
				fillOcclusionInv_<unsigned short>(src, (unsigned short)invalidvalue,0);
			}
			else if(src.type()==CV_32F)
			{
				fillOcclusionInv_<float>(src, (float)invalidvalue,FLT_MIN);
			}

		}
		else
		{
			if(src.type()==CV_8U)
			{
				fillOcclusion_<uchar>(src, (uchar)invalidvalue, UCHAR_MAX);
			}
			else if(src.type()==CV_16S)
			{
				fillOcclusion_<short>(src, (short)invalidvalue, SHRT_MAX);
			}
			else if(src.type()==CV_16U)
			{
				fillOcclusion_<unsigned short>(src, (unsigned short)invalidvalue, USHRT_MAX);
			}
			else if(src.type()==CV_32F)
			{
				fillOcclusion_<float>(src, (float)invalidvalue,FLT_MAX);
			}
		}
	}

	void xcvSetRotMatrix(CvMat* rot,double pitch, double roll, double yaw)
{
	double phi = roll/180.0*CV_PI;
	double theta = pitch/180.0*CV_PI;
	double pusai = yaw/180.0*CV_PI;

	rot->data.db[0]=cos(phi)*cos(theta);
	rot->data.db[1]=cos(phi)*sin(theta)*sin(pusai)-sin(phi)*cos(pusai);
	rot->data.db[2]=cos(phi)*sin(theta)*cos(pusai)+sin(phi)*sin(pusai);

	rot->data.db[3]=sin(phi)*cos(theta);
	rot->data.db[4]=sin(phi)*sin(theta)*sin(pusai)+cos(phi)*cos(pusai);
	rot->data.db[5]=sin(phi)*sin(theta)*cos(pusai)-cos(phi)*sin(pusai);

	rot->data.db[6]=-sin(theta);
	rot->data.db[7]=cos(theta)*sin(pusai);
	rot->data.db[8]=cos(theta)*cos(pusai);
}
void eular2rot(double pitch, double roll, double yaw,Mat& dest)
	{
		if(dest.empty())dest=Mat::zeros(3,3,CV_64F);
		xcvSetRotMatrix(&CvMat(dest),pitch,roll,yaw);
	}
void lookat(Point3d from, Point3d to, Mat& destR)
{
	double x=to.x-from.x;
	double y=to.y-from.y;
	double z=to.z-from.z;

	double pitch =asin(x/sqrt(x*x+z*z))/CV_PI*180.0;
	double yaw   =asin(-y/sqrt(y*y+z*z))/CV_PI*180.0;
	eular2rot(pitch, 0, yaw,destR);
}

template <class T>
static void projectImagefromXYZ_(Mat& image, Mat& destimage, Mat& disp, Mat& destdisp, Mat& xyz, Mat& R, Mat& t, Mat& K, Mat& dist, Mat& mask, bool isSub)
{
	if(destimage.empty())destimage=Mat::zeros(Size(image.size()),image.type());
	if(destdisp.empty())destdisp=Mat::zeros(Size(image.size()),disp.type());

	vector<Point2f> pt;
	cv::projectPoints(xyz,R,t,K,Mat(),pt);	

	destimage.setTo(0);
	destdisp.setTo(0);

#pragma omp parallel for
	for(int j=1;j<image.rows-1;j++)
	{
		int count=j*image.cols;
		uchar* img=image.ptr<uchar>(j);
		uchar* m=mask.ptr<uchar>(j);
		for(int i=0;i<image.cols;i++,count++)
		{
			int x=(int)pt[count].x;
			int y=(int)pt[count].y;
			if(m[i]==255)continue;
			if(pt[count].x>=1 && pt[count].x<image.cols-1 && pt[count].y>=1 && pt[count].y<image.rows-1)
			{
				short v=destdisp.at<T>(y,x);
				if(v<disp.at<T>(j,i))
				{
					destimage.at<uchar>(y,3*x+0)=img[3*i+0];
					destimage.at<uchar>(y,3*x+1)=img[3*i+1];
					destimage.at<uchar>(y,3*x+2)=img[3*i+2];
					destdisp.at<T>(y,x)=disp.at<T>(j,i);

					if(isSub)
					{
						if((int)pt[count+image.cols].y-y>1 && (int)pt[count+1].x-x>1)
						{
							destimage.at<uchar>(y,3*x+3)=img[3*i+0];
							destimage.at<uchar>(y,3*x+4)=img[3*i+1];
							destimage.at<uchar>(y,3*x+5)=img[3*i+2];

							destimage.at<uchar>(y+1,3*x+0)=img[3*i+0];
							destimage.at<uchar>(y+1,3*x+1)=img[3*i+1];
							destimage.at<uchar>(y+1,3*x+2)=img[3*i+2];

							destimage.at<uchar>(y+1,3*x+3)=img[3*i+0];
							destimage.at<uchar>(y+1,3*x+4)=img[3*i+1];
							destimage.at<uchar>(y+1,3*x+5)=img[3*i+2];

							destdisp.at<T>(y,x+1)=disp.at<T>(j,i);						
							destdisp.at<T>(y+1,x)=disp.at<T>(j,i);
							destdisp.at<T>(y+1,x+1)=disp.at<T>(j,i);
						}
						else if((int)pt[count-image.cols].y-y<-1 && (int)pt[count-1].x-x<-1)
						{
							destimage.at<uchar>(y,3*x-3)=img[3*i+0];
							destimage.at<uchar>(y,3*x-2)=img[3*i+1];
							destimage.at<uchar>(y,3*x-1)=img[3*i+2];

							destimage.at<uchar>(y-1,3*x+0)=img[3*i+0];
							destimage.at<uchar>(y-1,3*x+1)=img[3*i+1];
							destimage.at<uchar>(y-1,3*x+2)=img[3*i+2];

							destimage.at<uchar>(y-1,3*x-3)=img[3*i+0];
							destimage.at<uchar>(y-1,3*x-2)=img[3*i+1];
							destimage.at<uchar>(y-1,3*x-1)=img[3*i+2];

							destdisp.at<T>(y,x-1)=disp.at<T>(j,i);						
							destdisp.at<T>(y-1,x)=disp.at<T>(j,i);
							destdisp.at<T>(y-1,x-1)=disp.at<T>(j,i);
						}
						else if((int)pt[count+1].x-x>1)
						{
							destimage.at<uchar>(y,3*x+3)=img[3*i+0];
							destimage.at<uchar>(y,3*x+4)=img[3*i+1];
							destimage.at<uchar>(y,3*x+5)=img[3*i+2];
							
							destdisp.at<T>(y,x+1)=disp.at<T>(j,i);
						}
						else if((int)pt[count-1].x-x<-1)
						{
							destimage.at<uchar>(y,3*x-3)=img[3*i+0];
							destimage.at<uchar>(y,3*x-2)=img[3*i+1];
							destimage.at<uchar>(y,3*x-1)=img[3*i+2];
							
							destdisp.at<T>(y,x-1)=disp.at<T>(j,i);
						}
						else if((int)pt[count+image.cols].y-y>1)
						{
							destimage.at<uchar>(y+1,3*x+0)=img[3*i+0];
							destimage.at<uchar>(y+1,3*x+1)=img[3*i+1];
							destimage.at<uchar>(y+1,3*x+2)=img[3*i+2];

							destdisp.at<T>(y+1,x)=disp.at<T>(j,i);
						}
						else if((int)pt[count-image.cols].y-y<-1)
						{
							destimage.at<uchar>(y-1,3*x+0)=img[3*i+0];
							destimage.at<uchar>(y-1,3*x+1)=img[3*i+1];
							destimage.at<uchar>(y-1,3*x+2)=img[3*i+2];

							destdisp.at<T>(y-1,x)=disp.at<T>(j,i);
						}
					}
				}
			}
		}
	}

	if(isSub)
	{
		Mat image2;
		Mat disp2;
		destimage.copyTo(image2);
		destdisp.copyTo(disp2);
		const int BS=1;
#pragma omp parallel for
		for(int j=BS;j<image.rows-BS;j++)
		{
			uchar* img=destimage.ptr<uchar>(j);
			T* m = disp2.ptr<T>(j);
			T* dp = destdisp.ptr<T>(j);
			for(int i=BS;i<image.cols-BS;i++)
			{
				if(m[i]==0)
				{
					int count=0;
					int d=0;
					int r=0;
					int g=0;
					int b=0;
					for(int l=-BS;l<=BS;l++)
					{
						T* dp2 = disp2.ptr<T>(j+l);
						uchar* img2 = image2.ptr<uchar>(j+l);
						for(int k=-BS;k<=BS;k++)
						{
							if(dp2[i+k]!=0)
							{
								count++;
								d+=dp2[i+k];
								r+=img2[3*(i+k)+0];
								g+=img2[3*(i+k)+1];
								b+=img2[3*(i+k)+2];
							}
						}
					}
					if(count!=0)
					{
						double div = 1.0/count;
						dp[i]=d*div;
						img[3*i+0]=r*div;
						img[3*i+1]=g*div;
						img[3*i+2]=b*div;
					}
				}
			}
		}
	}
}
void projectImagefromXYZ(Mat& image, Mat& destimage, Mat& disp, Mat& destdisp, Mat& xyz, Mat& R, Mat& t, Mat& K, Mat& dist, Mat& mask, bool isSub)
{
	if(mask.empty())mask=Mat::zeros(image.size(),CV_8U);
	if(disp.type()==CV_8U)
	{
		projectImagefromXYZ_<unsigned char>(image,destimage, disp, destdisp, xyz, R, t, K, dist, mask,isSub);
	}
	else if(disp.type()==CV_16S)
	{
		projectImagefromXYZ_<short>(image,destimage, disp, destdisp, xyz, R, t, K, dist, mask,isSub);
	}
	else if(disp.type()==CV_16U)
	{
		projectImagefromXYZ_<unsigned short>(image,destimage, disp, destdisp, xyz, R, t, K, dist, mask,isSub);
	}
	else if(disp.type()==CV_32F)
	{
		projectImagefromXYZ_<float>(image,destimage, disp, destdisp, xyz, R, t, K, dist, mask,isSub);		
	}
	else if(disp.type()==CV_64F)
	{
		projectImagefromXYZ_<double>(image,destimage, disp, destdisp, xyz, R, t, K, dist, mask,isSub);		
	}
}


template <class T>
static void projectBFImagefromXYZ_(Mat& image, Mat& back, Mat& destimage, Mat& disp, Mat& dispBack, Mat& destdisp, Mat& xyz, Mat& xyzBack, Mat& R, Mat& t, Mat& K, Mat& dist, Mat& mask, bool isSub)
{
	if(destimage.empty())destimage=Mat::zeros(Size(image.size()),image.type());
	if(destdisp.empty())destdisp=Mat::zeros(Size(image.size()),disp.type());

	vector<Point2f> cpt;
	vector<Point2f> bpt;
	cv::projectPoints(xyz,R,t,K,Mat(),cpt);	
	cv::projectPoints(xyzBack,R,t,K,Mat(),bpt);	

	destimage.setTo(0);
	destdisp.setTo(0);

#pragma omp parallel for
	for(int j=1;j<image.rows-1;j++)
	{
		int count=j*image.cols;
		uchar* img=image.ptr<uchar>(j);
		uchar* bimg=back.ptr<uchar>(j);
		uchar* m=mask.ptr<uchar>(j);
		for(int i=0;i<image.cols;i++,count++)
		{
			Point2f* pt;
			pt=(Point2f*)&bpt[0];;
			int x=(int)pt[count].x;
			int y=(int)pt[count].y;
			if(m[i]==255)continue;
			if(x>=1 && x<image.cols-1 && y>=1 && y<image.rows-1)
			{
				short v=destdisp.at<T>(y,x);
				if(v<dispBack.at<T>(j,i))
				{
					destimage.at<uchar>(y,3*x+0)=bimg[3*i+0];
					destimage.at<uchar>(y,3*x+1)=bimg[3*i+1];
					destimage.at<uchar>(y,3*x+2)=bimg[3*i+2];
					destdisp.at<T>(y,x)=dispBack.at<T>(j,i);

					if(isSub)
					{
						if((int)pt[count+image.cols].y-y>1 && (int)pt[count+1].x-x>1)
						{
							destimage.at<uchar>(y,3*x+3)=bimg[3*i+0];
							destimage.at<uchar>(y,3*x+4)=bimg[3*i+1];
							destimage.at<uchar>(y,3*x+5)=bimg[3*i+2];

							destimage.at<uchar>(y+1,3*x+0)=bimg[3*i+0];
							destimage.at<uchar>(y+1,3*x+1)=bimg[3*i+1];
							destimage.at<uchar>(y+1,3*x+2)=bimg[3*i+2];

							destimage.at<uchar>(y+1,3*x+3)=bimg[3*i+0];
							destimage.at<uchar>(y+1,3*x+4)=bimg[3*i+1];
							destimage.at<uchar>(y+1,3*x+5)=bimg[3*i+2];

							destdisp.at<T>(y,x+1)=dispBack.at<T>(j,i);						
							destdisp.at<T>(y+1,x)=dispBack.at<T>(j,i);
							destdisp.at<T>(y+1,x+1)=dispBack.at<T>(j,i);
						}
						else if((int)pt[count-image.cols].y-y<-1 && (int)pt[count-1].x-x<-1)
						{
							destimage.at<uchar>(y,3*x-3)=bimg[3*i+0];
							destimage.at<uchar>(y,3*x-2)=bimg[3*i+1];
							destimage.at<uchar>(y,3*x-1)=bimg[3*i+2];

							destimage.at<uchar>(y-1,3*x+0)=bimg[3*i+0];
							destimage.at<uchar>(y-1,3*x+1)=bimg[3*i+1];
							destimage.at<uchar>(y-1,3*x+2)=bimg[3*i+2];

							destimage.at<uchar>(y-1,3*x-3)=bimg[3*i+0];
							destimage.at<uchar>(y-1,3*x-2)=bimg[3*i+1];
							destimage.at<uchar>(y-1,3*x-1)=bimg[3*i+2];

							destdisp.at<T>(y,x-1)=dispBack.at<T>(j,i);						
							destdisp.at<T>(y-1,x)=dispBack.at<T>(j,i);
							destdisp.at<T>(y-1,x-1)=dispBack.at<T>(j,i);
						}
						else if((int)pt[count+1].x-x>1)
						{
							destimage.at<uchar>(y,3*x+3)=bimg[3*i+0];
							destimage.at<uchar>(y,3*x+4)=bimg[3*i+1];
							destimage.at<uchar>(y,3*x+5)=bimg[3*i+2];
							
							destdisp.at<T>(y,x+1)=dispBack.at<T>(j,i);
						}
						else if((int)pt[count-1].x-x<-1)
						{
							destimage.at<uchar>(y,3*x-3)=bimg[3*i+0];
							destimage.at<uchar>(y,3*x-2)=bimg[3*i+1];
							destimage.at<uchar>(y,3*x-1)=bimg[3*i+2];
							
							destdisp.at<T>(y,x-1)=dispBack.at<T>(j,i);
						}
						else if((int)pt[count+image.cols].y-y>1)
						{
							destimage.at<uchar>(y+1,3*x+0)=bimg[3*i+0];
							destimage.at<uchar>(y+1,3*x+1)=bimg[3*i+1];
							destimage.at<uchar>(y+1,3*x+2)=bimg[3*i+2];

							destdisp.at<T>(y+1,x)=dispBack.at<T>(j,i);
						}
						else if((int)pt[count-image.cols].y-y<-1)
						{
							destimage.at<uchar>(y-1,3*x+0)=bimg[3*i+0];
							destimage.at<uchar>(y-1,3*x+1)=bimg[3*i+1];
							destimage.at<uchar>(y-1,3*x+2)=bimg[3*i+2];

							destdisp.at<T>(y-1,x)=dispBack.at<T>(j,i);
						}
					}
				}
			}
			if(disp.at<T>(j,i)>dispBack.at<T>(j,i))
			{
				pt=(Point2f*)&cpt[0];
				x=(int)pt[count].x;
				y=(int)pt[count].y;
			if(x>=1 && x<image.cols-1 && y>=1 && y<image.rows-1)
			{
				short v=destdisp.at<T>(y,x);
				if(v<disp.at<T>(j,i))
				{
					destimage.at<uchar>(y,3*x+0)=img[3*i+0];
					destimage.at<uchar>(y,3*x+1)=img[3*i+1];
					destimage.at<uchar>(y,3*x+2)=img[3*i+2];
					destdisp.at<T>(y,x)=disp.at<T>(j,i);

					if(isSub)
					{
						if((int)pt[count+image.cols].y-y>1 && (int)pt[count+1].x-x>1)
						{
							destimage.at<uchar>(y,3*x+3)=img[3*i+0];
							destimage.at<uchar>(y,3*x+4)=img[3*i+1];
							destimage.at<uchar>(y,3*x+5)=img[3*i+2];

							destimage.at<uchar>(y+1,3*x+0)=img[3*i+0];
							destimage.at<uchar>(y+1,3*x+1)=img[3*i+1];
							destimage.at<uchar>(y+1,3*x+2)=img[3*i+2];

							destimage.at<uchar>(y+1,3*x+3)=img[3*i+0];
							destimage.at<uchar>(y+1,3*x+4)=img[3*i+1];
							destimage.at<uchar>(y+1,3*x+5)=img[3*i+2];

							destdisp.at<T>(y,x+1)=disp.at<T>(j,i);						
							destdisp.at<T>(y+1,x)=disp.at<T>(j,i);
							destdisp.at<T>(y+1,x+1)=disp.at<T>(j,i);
						}
						else if((int)pt[count-image.cols].y-y<-1 && (int)pt[count-1].x-x<-1)
						{
							destimage.at<uchar>(y,3*x-3)=img[3*i+0];
							destimage.at<uchar>(y,3*x-2)=img[3*i+1];
							destimage.at<uchar>(y,3*x-1)=img[3*i+2];

							destimage.at<uchar>(y-1,3*x+0)=img[3*i+0];
							destimage.at<uchar>(y-1,3*x+1)=img[3*i+1];
							destimage.at<uchar>(y-1,3*x+2)=img[3*i+2];

							destimage.at<uchar>(y-1,3*x-3)=img[3*i+0];
							destimage.at<uchar>(y-1,3*x-2)=img[3*i+1];
							destimage.at<uchar>(y-1,3*x-1)=img[3*i+2];

							destdisp.at<T>(y,x-1)=disp.at<T>(j,i);						
							destdisp.at<T>(y-1,x)=disp.at<T>(j,i);
							destdisp.at<T>(y-1,x-1)=disp.at<T>(j,i);
						}
						else if((int)pt[count+1].x-x>1)
						{
							destimage.at<uchar>(y,3*x+3)=img[3*i+0];
							destimage.at<uchar>(y,3*x+4)=img[3*i+1];
							destimage.at<uchar>(y,3*x+5)=img[3*i+2];
							
							destdisp.at<T>(y,x+1)=disp.at<T>(j,i);
						}
						else if((int)pt[count-1].x-x<-1)
						{
							destimage.at<uchar>(y,3*x-3)=img[3*i+0];
							destimage.at<uchar>(y,3*x-2)=img[3*i+1];
							destimage.at<uchar>(y,3*x-1)=img[3*i+2];
							
							destdisp.at<T>(y,x-1)=disp.at<T>(j,i);
						}
						else if((int)pt[count+image.cols].y-y>1)
						{
							destimage.at<uchar>(y+1,3*x+0)=img[3*i+0];
							destimage.at<uchar>(y+1,3*x+1)=img[3*i+1];
							destimage.at<uchar>(y+1,3*x+2)=img[3*i+2];

							destdisp.at<T>(y+1,x)=disp.at<T>(j,i);
						}
						else if((int)pt[count-image.cols].y-y<-1)
						{
							destimage.at<uchar>(y-1,3*x+0)=img[3*i+0];
							destimage.at<uchar>(y-1,3*x+1)=img[3*i+1];
							destimage.at<uchar>(y-1,3*x+2)=img[3*i+2];

							destdisp.at<T>(y-1,x)=disp.at<T>(j,i);
						}
					}
				}
			}
			}
		}
	}

	if(isSub)
	{
		Mat image2;
		Mat disp2;
		destimage.copyTo(image2);
		destdisp.copyTo(disp2);
		const int BS=1;
#pragma omp parallel for
		for(int j=BS;j<image.rows-BS;j++)
		{
			uchar* img=destimage.ptr<uchar>(j);
			T* m = disp2.ptr<T>(j);
			T* dp = destdisp.ptr<T>(j);
			for(int i=BS;i<image.cols-BS;i++)
			{
				if(m[i]==0)
				{
					int count=0;
					int d=0;
					int r=0;
					int g=0;
					int b=0;
					for(int l=-BS;l<=BS;l++)
					{
						T* dp2 = disp2.ptr<T>(j+l);
						uchar* img2 = image2.ptr<uchar>(j+l);
						for(int k=-BS;k<=BS;k++)
						{
							if(dp2[i+k]!=0)
							{
								count++;
								d+=dp2[i+k];
								r+=img2[3*(i+k)+0];
								g+=img2[3*(i+k)+1];
								b+=img2[3*(i+k)+2];
							}
						}
					}
					if(count!=0)
					{
						double div = 1.0/count;
						dp[i]=d*div;
						img[3*i+0]=r*div;
						img[3*i+1]=g*div;
						img[3*i+2]=b*div;
					}
				}
			}
		}
	}
}
void projectBFImagefromXYZ(Mat& image, Mat& back, Mat& destimage, Mat& disp, Mat& dispBack, Mat& destdisp, Mat& xyz, Mat& xyzBack, Mat& R, Mat& t, Mat& K, Mat& dist, Mat& mask, bool isSub)
{
	if(mask.empty())mask=Mat::zeros(image.size(),CV_8U);
	if(disp.type()==CV_8U)
	{
		projectBFImagefromXYZ_<unsigned char>(image,back,destimage, disp, dispBack,destdisp, xyz,xyzBack, R, t, K, dist, mask,isSub);
	}
	else if(disp.type()==CV_16S)
	{
		projectBFImagefromXYZ_<short>(image,back,destimage, disp, dispBack,destdisp, xyz,xyzBack, R, t, K, dist, mask,isSub);
	}
	else if(disp.type()==CV_16U)
	{
		projectBFImagefromXYZ_<unsigned short>(image,back,destimage, disp, dispBack,destdisp, xyz,xyzBack, R, t, K, dist, mask,isSub);
	}
	else if(disp.type()==CV_32F)
	{
		projectBFImagefromXYZ_<float>(image,back,destimage, disp, dispBack,destdisp, xyz,xyzBack, R, t, K, dist, mask,isSub);
	}
	else if(disp.type()==CV_64F)
	{
		projectBFImagefromXYZ_<double>(image,back,destimage, disp, dispBack,destdisp, xyz,xyzBack, R, t, K, dist, mask,isSub);
	}
}
void reprojectKinectDepth3D(Mat& src, Mat& dest, const double focal_length, Point2d imageCenter=Point2d(-1,-1))
{
	if(dest.empty())dest=Mat::zeros(src.size().area(),1,CV_32FC3);
	
	 const double bigZ = 10000.;
	const double hw=(src.cols-1)*0.5;
	const double hh=(src.rows-1)*0.5;
	if(imageCenter.x==-1&&imageCenter.y==-1)imageCenter=Point2d(hw,hh);
	double ifocal_length=1.0/focal_length;
	for(int j=0;j<src.rows;j++)
	{
		float* data=dest.ptr<float>(j*src.cols);
		unsigned short* s=src.ptr<unsigned short>(j);
		for(int i=0;i<src.cols;i++)
		{
			data[0]=*s*ifocal_length*(i-imageCenter.x);
			data[1]=*s*ifocal_length*(j-imageCenter.y);
			if(*s==0)data[2]=bigZ;
			else data[2]=*s;
			
			data+=3;
			s++;
		}
	}
}
int main(int argc, char** argv)
{
	XnStatus nRetVal = XN_STATUS_OK;

	Context context;
	
	EnumerationErrors errors;

	nRetVal = context.InitFromXmlFile(SAMPLE_XML_PATH, &errors);

	if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (nRetVal);
	}
	else if (nRetVal != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(nRetVal));
		return (nRetVal);
	}

	DepthGenerator depth;// depth context
	nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
	CHECK_RC(nRetVal, "Find depth generator");
	ImageGenerator image;//image context
	ImageGenerator image2;//image context
	nRetVal = context.FindExistingNode(XN_NODE_TYPE_IMAGE, image);
	CHECK_RC(nRetVal, "Find image generator");

	DepthMetaData depthMD;
	ImageMetaData imageMD;

	depth.GetAlternativeViewPointCap().SetViewPoint(image);

	const double focal_length = 580.0;
	const double baseline = 7.5;
	const double step = baseline*5;

	Mat K=Mat::eye(3,3,CV_64F);
	K.at<double>(0,0)=focal_length;
	K.at<double>(1,1)=focal_length;
	K.at<double>(0,2)=(640-1.0)/2.0;
	K.at<double>(1,2)=(480-1.0)/2.0;

	Mat R=Mat::eye(3,3,CV_64F);
	Mat t=Mat::zeros(3,1,CV_64F);

	Point3d lookatpoint(0,0,-2025);
	Point3d viewpoint(0.0,0.0,step);

	Mat xyz;
	Mat xyzback=Mat::zeros(Size(640,480).area(),1,CV_32FC3);
	Mat imageback(Size(640,480),CV_8UC3);
	Mat depthback=Mat::ones(Size(640,480),CV_8U);

	Mat depthshow;
	Mat destdisp;

	Mat show;
	Mat destshow;

	int key=0;
	bool isSub=false;
	bool isBack=false;
	while (key!='q')
	{
		//wait and error processing 
		nRetVal = context.WaitAnyUpdateAll();
		if (nRetVal != XN_STATUS_OK)
		{
			printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
			continue;
		}

		image.GetMetaData(imageMD);
		depth.GetMetaData(depthMD);

		//for opencv Mat
		Mat imni(Size(640,480),CV_8UC3,(uchar*)imageMD.Data());
		
		//convert color space RGB2BGR
		cvtColor(imni,show,CV_RGB2BGR);	

		XnDepthPixel* data = depthMD.WritableData();
		Mat depth16(Size(640,480),CV_16U,(unsigned short*)data);
		fillOcclusion(depth16,0,true);

		//convert depth 11 bit image 2 8 bit image
		depth16.convertTo(depthshow,CV_8U,-255/4096.0,255);

		//RGB(XYZ)の32Fとかのデータ
		reprojectKinectDepth3D(depth16, xyz, focal_length);
		
		lookat(viewpoint, lookatpoint , R);
		t.at<double>(0,0)=viewpoint.x;
		t.at<double>(1,0)=viewpoint.y;
		t.at<double>(2,0)=viewpoint.z;
		t=R*t;

		if(isBack)
		projectBFImagefromXYZ(show,imageback,destshow,depthshow,depthback,destdisp,xyz,xyzback,R,t,K,Mat(),Mat(),isSub);
		else projectImagefromXYZ(show,destshow,depthshow,destdisp,xyz,R,t,K,Mat(),Mat(),isSub);
		
		imshow("warp depth",destdisp);
		imshow("warp image",destshow);

		if(key=='b')
		{
			isBack=isBack?false:true;
		}
		if(key=='B')
		{
			depthshow.copyTo(depthback);
			show.copyTo(imageback);
			xyz.copyTo(xyzback);
		}
		if(key=='r')
		{
			viewpoint=Point3d (0.0,0.0,0.0);
		}
		if(key=='f')
		{
			isSub=isSub?false:true;
		}
		if(key=='s')
		{
			imwrite("image.bmp",show);
			FILE* fp=fopen("out.raw","wb");
			fwrite(depthMD.WritableData(),sizeof(unsigned short),640*480,fp);
				fclose(fp);
		}
		if(key=='k')
		{
			viewpoint.y+=step;
		}
		if(key=='j')
		{
			viewpoint.y-=step;
		}
		if(key=='h')
		{
			viewpoint.x+=step;
		}
		if(key=='l')
		{
			viewpoint.x-=step;
		}
		if(key=='K')
		{
			viewpoint.z+=step;
		}
		if(key=='J')
		{
			viewpoint.z-=step;
		}
		key = waitKey(1);
	}
	context.Shutdown();
	return 0;
}