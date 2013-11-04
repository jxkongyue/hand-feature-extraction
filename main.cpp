#include "recognition.h"

#define PI 3.1415926


void skinSegment(const Mat& src, Mat& dst){
	dst.create( src.rows ,src.cols , CV_8U);
	Mat imgYCrCb;

	cvtColor(src, imgYCrCb, CV_BGR2YCrCb);

	vector<Mat> ycrcb( imgYCrCb.channels());
	split(imgYCrCb, ycrcb); 
	int y, cr, cb, x1, y1, value;
	int iRows=src.rows;
	int iCols=src.cols;
	for (int h=0; h<iRows; h++){              // h:行
		uchar* data_YCrCb0=ycrcb[0].ptr<uchar>(h);
		uchar* data_YCrCb1=ycrcb[1].ptr<uchar>(h);
		uchar* data_YCrCb2=ycrcb[2].ptr<uchar>(h);

		const uchar* d1 = src.ptr<uchar>(h);
		uchar* data_dst = dst.ptr<uchar>(h);
		for (int w=0; w<iCols ;w++){
			y = data_YCrCb0[w];
			cr = data_YCrCb1[w];
			cb = data_YCrCb2[w];
			cb -= 109;
			cr -= 152;
			x1 = (819*cr-614*cb)/32 + 51;
			y1 = (819*cr+614*cb)/32 + 77;
			x1 = x1*41/1024;
			y1 = y1*73/1024;
			value = x1*x1+y1*y1;
			if(y<100)	
				data_dst[w] =(value<700) ? 255:0;
			else	
				data_dst[w] =(value<850)? 255:0;
		}

	}
}

void skinSegAndBackground(const Mat& src, Mat& background, Mat& dst){
	dst.create( src.rows ,src.cols , CV_8U);
	Mat imgYCrCb;

	cvtColor(src, imgYCrCb, CV_BGR2YCrCb);

	vector<Mat> ycrcb( imgYCrCb.channels());
	split(imgYCrCb, ycrcb); 
	int y, cr, cb, x1, y1, value;
	int iRows=src.rows;
	int iCols=src.cols;
	for (int h=0; h< iRows ;h++){              // h:行
		uchar* data_YCrCb0=ycrcb[0].ptr<uchar>(h);
		uchar* data_YCrCb1=ycrcb[1].ptr<uchar>(h);
		uchar* data_YCrCb2=ycrcb[2].ptr<uchar>(h);

		const uchar* d1 = src.ptr<uchar>(h);
		const uchar* d2 = background.ptr<uchar>(h);   

		uchar* data_dst = dst.ptr<uchar>(h);
		for (int w=0; w< iCols ;w++){
			y = data_YCrCb0[w];
			cr = data_YCrCb1[w];
			cb = data_YCrCb2[w];
			cb -= 109;
			cr -= 152;
			x1 = (819*cr-614*cb)/32 + 51;
			y1 = (819*cr+614*cb)/32 + 77;
			x1 = x1*41/1024;
			y1 = y1*73/1024;
			value = x1*x1+y1*y1;

			int temp=(d1[3*w]-d2[3*w])*(d1[3*w]-d2[3*w])+
				(d1[3*w+1]-d2[3*w+1])*(d1[3*w+1]-d2[3*w+1])+
				(d1[3*w+2]-d2[3*w+2])*(d1[3*w+2]-d2[3*w+2]);
			if ( temp> 100)
			{
				if(y<100)	
					data_dst[w] =(value<700) ? 255:0;
				else	
					data_dst[w] =(value<850)? 255:0;		
			}
			else	data_dst[w]= 0;
		}
	}
}
void backSubtraction( Mat& src, Mat& background){   //仅背景分割
	for (int h=0; h<src.rows ;h++){              // h:行
		uchar* d1 = src.ptr<uchar>(h);
		uchar* d2 = background.ptr<uchar>(h);
		for (int w=0; w<src.cols ;w++){
			int temp=(d1[3*w]-d2[3*w])*(d1[3*w]-d2[3*w])+
				(d1[3*w+1]-d2[3*w+1])*(d1[3*w+1]-d2[3*w+1])+
				(d1[3*w+2]-d2[3*w+2])*(d1[3*w+2]-d2[3*w+2]);
			if (temp<40){
				d1[3*w]=0;
				d1[3*w+1]=0;
				d1[3*w+2]=0;
			}
		}
	}
}
static bool _rankTargets(CvTarget t1,CvTarget t2)
{
	return t1.erea>t2.erea;
}
void findTargets(const Mat& img,const int erea_threshold,vector<CvTarget> &targets){
	assert(img.channels()==1);
	targets.clear();
	Mat imgCopy;
	img.copyTo(imgCopy);               //互不影响的复制，用 = 只是地址复制

	int iRow= imgCopy.rows;
	int iCol= imgCopy.cols;
	for (int h=0; h< iRow ;h++){
		for (int w=0; w< iCol ;w++){
			if (   imgCopy.at<unsigned char> (h ,w) ==255){
				CvTarget target;
				target.top=h;
				target.bottom=h;  //底部
				target.left=w;
				target.right=w;   
				queue<CvPoint> points;
				points.push(cvPoint(w,h));
				imgCopy.at<unsigned char> (h ,w) =0;
				//find target with breadth iteration 
				while(!points.empty()){
					target.erea++;
					CvPoint p=points.front();    //第一个元素       图像上的点
					points.pop();	         				

					if (p.x>0&&imgCopy.at<unsigned char> (p.y, p.x-1 ) ==255){//left
						imgCopy.at<unsigned char> (p.y, p.x-1 ) =0;					
						points.push(cvPoint(p.x-1,p.y));
						if (target.left>p.x-1){
							target.left=p.x-1;
						}
					}
					if (p.y+1<iRow &&imgCopy.at<unsigned char> (p.y+1, p.x ) ==255){//bottom
						imgCopy.at<unsigned char>  (p.y+1, p.x ) =0;					
						points.push(cvPoint(p.x,p.y+1));
						if (target.bottom<p.y+1){
							target.bottom=p.y+1;
						}
					}
					if (p.x+1<iCol &&imgCopy.at<unsigned char> (p.y,p.x+1) ==255){//right
						imgCopy.at<unsigned char>(p.y,p.x+1)=0;					
						points.push(cvPoint(p.x+1,p.y));
						if (target.right<p.x+1){
							target.right=p.x+1;
						}
					}
					if (p.y>0&&  imgCopy.at<unsigned char> (p.y-1,p.x ) ==255){//top
						imgCopy.at<unsigned char> (p.y-1,p.x ) =0;					
						points.push(cvPoint(p.x,p.y-1));
					}
				}
				if (target.erea>erea_threshold){
					targets.push_back(target);      //阈值
				}				
			}
		}
	}
	sort(targets.begin(),targets.end(),_rankTargets);
}
//改进了
void findFingerPoint(const Mat& img,CvTarget target,Point &center,Point &finger){
	assert(img.channels()==1);
	unsigned int h_sum=0;
	unsigned int w_sum=0;
	int i=0;
	center=Point(0 ,0);
	for (int h=target.top;h<=target.bottom;h++){
		for (int w=target.left;w<=target.right;w++){
			if(  img.at<unsigned char> (h ,w)==255)
			{
				h_sum=h_sum+h;
				w_sum=w_sum+w;
				i++;
			}
		}
	}
	if(i>0){                             //求离中心最远点
		center = Point(w_sum/i,h_sum/i);
		finger = center;

		for (int h=target.top;h<=target.bottom;h++){
			for (int w=target.left;w<=target.right;w++){
				if ( img.at<unsigned char> (h ,w)==255&&     
					dist(Point(w, h) , center) > dist(finger,center) )   
					finger= Point(w, h);			
			}
		}
	}

}

bool handJudge(const Mat& img, const CvTarget &target){
	unsigned int fingerNum=0;
	assert(img.channels()==1);
	Point pCenter, p1;
	Mat imgTarget;
	imgTarget.create( target.bottom-target.top ,target.right-target.left , CV_8U);  //先rows，再cols
	findFingerPoint(img, target, pCenter, p1);
	int p1ToCenter= dist(p1, pCenter);

	//将target区域从img 中分离出来。形成新的图像 imgTarget
	for (int w=target.left;w<target.right;w++){           //赋值

		for (int h=target.top;h<target.bottom;h++){
			if (  ((w-pCenter.x)*(p1.x-pCenter.x)+(h-pCenter.y)*(p1.y-pCenter.y))>0  \
				&& 4* dist( Point(w,h) ,pCenter)>  p1ToCenter   \
				&& img.at<unsigned char> (h ,w) ==255 )
			{
				imgTarget.at<unsigned char> (h-target.top, w-target.left)=255;
			}
			else 
				imgTarget.at<unsigned char> (h-target.top, w-target.left)= 0;
		}
	}

	// 求指尖的个数
	int iRow= imgTarget.rows;
	int iCol= imgTarget.cols;
	for (int h=0; h< iRow ;h++){
		for (int w=0; w< iCol ;w++){
			if (   imgTarget.at<unsigned char> (h ,w) ==255){
				CvTarget target;
				target.top=h;
				target.bottom=h;  //底部
				target.left=w;
				target.right=w;   
				queue<CvPoint> points;
				points.push(cvPoint(w,h));
				imgTarget.at<unsigned char> (h ,w) =0;
				//find target with breadth iteration 
				while(!points.empty()){
					target.erea++;
					CvPoint p=points.front();    //第一个元素       图像上的点
					points.pop();	         				

					if (p.x>0&&imgTarget.at<unsigned char> (p.y, p.x-1 ) ==255){//left
						imgTarget.at<unsigned char> (p.y, p.x-1 ) =0;					
						points.push(cvPoint(p.x-1,p.y));
						if (target.left>p.x-1){
							target.left=p.x-1;
						}
					}
					if (p.y+1<iRow &&imgTarget.at<unsigned char> (p.y+1, p.x ) ==255){//bottom
						imgTarget.at<unsigned char>  (p.y+1, p.x ) =0;					
						points.push(cvPoint(p.x,p.y+1));
						if (target.bottom<p.y+1){
							target.bottom=p.y+1;
						}
					}
					if (p.x+1<iCol &&imgTarget.at<unsigned char> (p.y,p.x+1) ==255){//right
						imgTarget.at<unsigned char>(p.y,p.x+1)=0;					
						points.push(cvPoint(p.x+1,p.y));
						if (target.right<p.x+1){
							target.right=p.x+1;
						}
					}
					if (p.y>0&&  imgTarget.at<unsigned char> (p.y-1,p.x ) ==255){//top
						imgTarget.at<unsigned char> (p.y-1,p.x ) =0;					
						points.push(cvPoint(p.x,p.y-1));
					}
				}
				if (target.erea> 200 && target.getHeight()>5&&target.getWidth() >5){
					fingerNum++;
				}				
			}
		}
	}

	//根据 fingerNum 个数判断是否是手掌
	if ( fingerNum >3 && fingerNum< 6)
		return true;
	return false;
}
//计算椭圆
Ellipse calcEllipse(const Mat& img, const CvTarget &target){
	Ellipse ellipse1;

	Point fingerCenter, fingerPoint;
	findFingerPoint(img, target,  fingerCenter, fingerPoint);
	ellipse1.initEllipseCenter(fingerCenter );

	////////////////////////////////////////////////
	int targetPointSum=0;
	for (int w=target.left;w<target.right;w++){           //求肤色块区域点的个数
		for (int h=target.top;h<target.bottom;h++){
			if (255==img.at<unsigned char> (h ,w) )
			{
				targetPointSum++;
			}
		}
	}
	CvMat* mat = cvCreateMat(targetPointSum , 2, CV_64FC1 );  
	unsigned int i=0;
	for (int w=target.left;w<target.right;w++){           //赋值
		for (int h=target.top;h<target.bottom;h++){
			if (255==	img.at<unsigned char> (h ,w))
			{
				CV_MAT_ELEM(*mat , double ,i, 0 )=w;
				CV_MAT_ELEM(*mat , double ,i, 1 )=h;
				i++;
			}

		}
	}

	/** 分配协方差矩阵的空间 */       
	CvMat* covarianceOfMat = cvCreateMat( 2,2, CV_64FC1 );   
	cvZero( covarianceOfMat );    

	cvCalcCovarMatrix( (const void **)&mat, 1, covarianceOfMat,
		NULL, CV_COVAR_NORMAL | CV_COVAR_ROWS /*  | CV_COVAR_SCALE 未使用自带归一化功能 */ );

	//calculate
	double sigma_xx=CV_MAT_ELEM(*covarianceOfMat,double ,0, 0) /(i-1) ;    ////取消归一化  /(i-1)
	double sigma_xy=CV_MAT_ELEM(*covarianceOfMat,double ,0, 1) /(i-1) ;
	double sigma_yx=CV_MAT_ELEM(*covarianceOfMat,double ,1, 0) /(i-1) ;
	double sigma_yy=CV_MAT_ELEM(* covarianceOfMat,double ,1, 1) /(i-1) ;

	double Lambda=sqrt( (sigma_xx-sigma_yy)*(sigma_xx-sigma_yy) + 4*sigma_xy*sigma_xy);  ///////////////
	double lambda1=(sigma_xx+ sigma_yy+ Lambda)/2;
	double lambda2=(sigma_xx+ sigma_yy-  Lambda)/2;
	ellipse1.axis_a=sqrt( lambda1);
	ellipse1.axis_b=sqrt( lambda2);
	ellipse1.angle=atan( sigma_xy /(lambda1- sigma_yy) );

	double num= PI *ellipse1.axis_a *ellipse1.axis_b; 

	ellipse1.axis_a=(ellipse1.axis_a) * sqrt( double (i)/ num);
	ellipse1.axis_b=(ellipse1.axis_b) * sqrt( double (i)/ num);
	return ellipse1;
}
bool isMoving(HandParameter handPre,HandParameter handCul){

	int delta_center= dist( handPre.center, handCul.center);

	double delta_area_rate= abs(sqrt(handPre.area)-sqrt(handCul.area) )/sqrt(handCul.area);	
	double delta_angle= abs(handPre.angle-handCul.angle);
	cout<< delta_center<<"   ,"<<delta_area_rate<<"   ,"<<delta_angle<<endl;
	if ( delta_center > 50 || delta_area_rate> 0.01 || delta_angle > 5)
	{
		return 1;
	}
	return 0;
}
void checkAction( vector<HandParameter> &vctHand){
	if ( vctHand.size()< 1 ) return;
	int last = vctHand.size()-1;
	int delta_x= vctHand[last].center.x -vctHand[0].center.x;
	int delta_y= vctHand[last].center.y -vctHand[0].center.y;
	if( delta_x >winSize.width/3 && abs( delta_y) < winSize.height/5  )  cout<<"向左"<<"  ,";
	if( delta_x <- winSize.width/3 && abs( delta_y) < winSize.height/5  )  cout<<"向右"<<"  ,";
	if( delta_y >winSize.height/3 && abs( delta_x) < winSize.width/4  )  cout<<"向下"<<"  ,";
	if( delta_y < -winSize.height/3 && abs( delta_x) < winSize.width/4  )  cout<<"向上"<<"  ,";

	double delta_angle= vctHand[last].angle- vctHand[0].angle;
	if( delta_angle >70 && delta_angle <110)  cout<<"顺时针旋转90度"<<"  ,";
	if( delta_angle <-70 && delta_angle >-110)  cout<<"逆时针旋转90度"<<"  ,";

	double areaSqrtLast=sqrt(vctHand[last].area);
	double areaSqrt0=sqrt(vctHand[0].area);
	double delta_area_rate= (areaSqrt0-areaSqrtLast)/areaSqrtLast;	
	if( delta_area_rate> 0.3)	cout<<"缩小"<<"  ,";
	if( delta_area_rate<-0.3)	cout<<"放大"<<"  ,";
	cout<<endl;
}
int main(){
	VideoCapture capture;
	capture.open( 0);
	// Read options
	if (!capture.isOpened())
	{
		cout << "capture device failed to open!" << endl;
		return 1;
	} 
	namedWindow("inputImage", CV_WINDOW_AUTOSIZE);
	namedWindow("imgSegment", CV_WINDOW_AUTOSIZE);
	Mat inputFrame;
	Mat imgSegment;
	capture>> inputFrame;

	HandParameter handCur, handPre;
	bool preStatus=0;         //初始是静止的
	vector<HandParameter> vctHand;

	const int i_x= 100;
	const int i_y= 120;
	while (1)
	{

		capture >> inputFrame;
		skinSegment(inputFrame, imgSegment);
		erode(imgSegment,imgSegment,Mat() );               //腐蚀图像，使用3*3矩阵结构
		dilate(imgSegment, imgSegment, Mat() );             //膨胀图像

		vector< CvTarget> targets;
		findTargets(imgSegment ,1000, targets);

		Point cent, point1;
		if ( targets.size() >0 )
		{
			findFingerPoint(imgSegment, targets[0], cent, point1 );
			rectangle(inputFrame,Point(cent.x- i_x ,cent.y - i_y),   
				cvPoint(cent.x+ i_x ,cent.y + i_y),CV_RGB(255,0,0),3);
		}


		imshow("inputImage", inputFrame);
		imshow("imgSegment",imgSegment);
		if (cvWaitKey(10) == 27) {	break; }


		//		cout<<iMiddle-ibegin<<",  "<<i3-iMiddle<<" , "<<i4-i3<<" , "<<iend-i4<<endl;
	}

}
