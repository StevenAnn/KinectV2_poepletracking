#pragma once
#include "Kalman.h"
#include "HungarianAlg.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

class CTrack
{
public:
	vector<Point2d> trace;
	static size_t NextTrackID;
	size_t track_id;
	size_t skipped_frames; 
	Point2d prediction;
	TKalmanFilter* KF;
	CTrack(Point2f p, float dt, float Accel_noise_mag);
	~CTrack();
};


class CTracker
{
public:
	
	//时间间隔	
	float dt; 

	float Accel_noise_mag;

	// 建立没有建立。如果发现约等于点建立在电弧建立同盟,建立既
	// 超过这个阈值,则不建立蒸汽分配问题建立的年代
	double dist_thres;
	// 可靠跟踪得到该处与建立建立建立测量数据
	int maximum_allowed_skipped_frames;
	// 	
	int max_trace_length;

	vector<CTrack*> tracks;
	void Update(vector<Point2d>& detections, int& counter_in, int & counter_out);
	CTracker(float _dt, float _Accel_noise_mag, double _dist_thres=60, int _maximum_allowed_skipped_frames=10,int _max_trace_length=10);
	~CTracker(void);
};

