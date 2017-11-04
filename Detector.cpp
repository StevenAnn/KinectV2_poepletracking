#include "Detector.h"
//
using namespace cv;
using namespace std;

Mat backgroundImage;

//----------------------------------------------------------------------
// 输出结果
//----------------------------------------------------------------------

CDetector::CDetector(int history, double varThreshold )
{
    //create Background Subtractor objects
    // default values history = 50 and varThreshold = 9
    pMOG2 = createBackgroundSubtractorMOG2(history, varThreshold); //MOG2 approach

}

void CDetector::DetectContour(Mat& mask, Mat& color_frame,vector<Rect>& Rects,vector<Point2d>& centers)
{
	double area=0;
	Rects.clear();
	centers.clear();
	vector<vector<Point> > contours;
	
	vector<Vec4i> hierarchy;
	int target = 0;
	Scalar color(0,255,100);
	findContours(mask,contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());

	
	if( contours.size() == 0 )
	{
	  return;
	}
	
	int idx = 0;
//	double minArea = 1500; double maxArea = 7000;
	double minArea = 1500; double maxArea = 10000;
	for( ; idx >= 0; idx = hierarchy[idx][0] )
	{
	  const vector<Point>& c = contours[idx];
	  double area = fabs(contourArea(Mat(c)));
//	  cout << "c: " << c << "\n" << endl;
//	  cout << "Area: " << area << "\n" << endl;
//	  cout << "zhouc: " << c.size() << "\n" << endl;
//	  double length = cvArcLength(contours[idx], CV_WHOLE_SEQ, -1);

	  if(area>minArea && area<maxArea)
	  {
	    target = idx;
//	    cout<<"Area"<<area<<endl;
	    Rect r=cv::boundingRect(contours[idx]);
	    Rects.push_back(r);
	    centers.push_back((r.br()+r.tl())*0.5);

		cout << "rect: " << r.br() << "    " << r.tl() << "\n" << endl;
	  
	    //drawContours(color_frame, contours, target, color, FILLED, LINE_8, hierarchy );
//	    rectangle(color_frame, r, color, 4, LINE_8,0 );
		rectangle(color_frame, r, color, 4, LINE_8, 0);
	  }
	}
	imshow("color inside dectector",color_frame);
	imwrite("color_inside_dectector.bmp", color_frame);
}


void CDetector::refineSegments(Mat& depth_frame, Mat& mask, Mat& dst)
{
    int niters = 1;
    Mat temp; 
    dst = Mat::zeros(depth_frame.size(), CV_8UC3);    
    Mat depth_thresh = Mat::zeros(depth_frame.size(), CV_8UC3);
    // threshold the depth image and filter with the thresholded mask
//	threshold(depth_frame, depth_thresh, 80, 255, THRESH_BINARY_INV);
    threshold(depth_frame, depth_thresh, 80, 255, THRESH_BINARY_INV);

	imshow("depth_thresh", depth_thresh);
//	imwrite("depth_thresh.bmp", depth_thresh);

	

    dst = depth_frame & depth_thresh;// masking

	imshow("depth_frame & depth_thresh", dst);
//	imwrite("depth_frame & depth_thresh.bmp", dst);

    // filter the thresholded mask with the motion mask
    Mat motion_mask_rgb = Mat::zeros(depth_frame.size(), CV_8UC3);
    cvtColor(mask,motion_mask_rgb,COLOR_GRAY2RGB);
    dst = dst & motion_mask_rgb;
    //convert the mask back to gray
    cvtColor(dst,mask,COLOR_RGB2GRAY);
//	dst = mask; // if do not want to dilate uncomment
//    Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*9 + 1, 2*9+1 ),Point( 9, 9 ) );
//    morphologyEx( mask, dst, 1, element,Point(-1,-1),niters*1); // 1 - dilate
	Mat element = getStructuringElement(MORPH_RECT, Size(2 * 9 + 1, 2 * 9 + 1), Point(9, 9));
	morphologyEx(mask, dst, 3, element, Point(-1, -1), niters * 1); // 3 - MORPH_CLOSE
	imshow("morphologyEx", dst);
//	imwrite("morphologyEx.bmp", dst);
}

vector<Point2d> CDetector::Detect(Mat& depth, Mat& color)
{
	Mat refined_fg;

	Mat back_image;

//	Timer *time1 = new Timer();
//	int ms1;
//	time1->start();
//	imwrite("Depth_jiaozheng.bmp", depth);

	Mat element = getStructuringElement(MORPH_RECT, Size(15,15), Point(7, 7));
	//进行闭运算的形态学操作
	morphologyEx(depth, depth, MORPH_CLOSE, element);
	morphologyEx(depth, depth, MORPH_OPEN, element);

	imshow("DEPTH_With_CLOSE", depth);
//	imwrite("Depth_with_CloseandOpen.bmp", depth);
	medianBlur(depth, depth, 5);
	imshow("DEPTH_With_median", depth);
//	imwrite("DEPTH_With_median.bmp", depth);


	pMOG2->apply(depth, fgMaskMOG2,0.01);// can give learning rate	
//	pMOG2->apply(depth, fgMaskMOG2, 0.0001);// can give learning rate

	pMOG2->getBackgroundImage(back_image);
	imshow("BackgroundImage", back_image);
//	imwrite("BackgroundImage.bmp", back_image);

//	ms1 = time1->getElapsedTimeInMilliSec();
//	cout << "MOGSec = " << (ms1) <<"\n"<< endl;

	imshow("fgMaskMOG2.bmp",fgMaskMOG2);
//	imwrite("fgMaskMOG2.bmp", fgMaskMOG2);



	imshow("MOG2_Depth", depth);

//	imwrite("MOG2_Depth.bmp", depth);



	refineSegments(depth,fgMaskMOG2,refined_fg);
	DetectContour(refined_fg,color,rects,centers);	
	return centers;
}

CDetector::~CDetector(void)
{
  
}
