

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "math.h"
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <sstream>
#include <queue>
#include <ctime>
using namespace std;
using namespace cv;

#define PI 3.1415926

//定义视频的大小： 640*480
const CvSize winSize=cvSize(640,480);

//矩形连通区域
class CvTarget{                                                          
public:
	int erea;                                                                 //size
	int top;                                                                   //position
	int bottom;
	int left;
	int right;
	CvTarget():erea(0),top(0),bottom(0),left(0),right(0){}
	CvPoint getCenter(){return cvPoint((right+left)>>1,(bottom+top)>>1);}
	int getWidth(){return right-left;}
	int getHeight(){return bottom-top;}
};
//拟合椭圆
class Ellipse{                                                     
public: 
	int c_x;
	int c_y;
	double axis_a;
	double axis_b;
	double angle;                           //逆时针

	Ellipse(): c_x(0), c_y(0), axis_a(0.0), axis_b(0.0 ), angle (0.0) {}
	void initEllipseCenter(CvPoint center ){
		c_x=center.x;
		c_y=center.y;
	}

};
inline int dist(Point p1, Point p2){
	return (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y);
}
inline double distance(CvPoint src, Ellipse elp){
	return pow( (src.x-elp.c_x)*cos(elp.angle)+(src.y-elp.c_y)*sin(elp.angle), 2)/elp.axis_a/elp.axis_a   \
		+pow( (src.y-elp.c_y)*cos(elp.angle)-(src.x-elp.c_x)*sin(elp.angle), 2)/elp.axis_b/elp.axis_b;
}

struct HandParameter{
	CvPoint center;
	double area;
	double angle;
	HandParameter(){
		center= cvPoint(0,0);
		area= 0.0 ; 
		angle= 0.0;
	}
};
