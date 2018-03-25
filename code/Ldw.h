#pragma once

#include <opencv.hpp>
#include "core.hpp"
#include "highgui.hpp"
#include "imgproc.hpp"
#include "objdetect.hpp"

using namespace cv;
using namespace std;

#define LANE_CASCADE_NAME "..\\code\\LBPcascade_lane.xml"  


typedef struct 
{
	Point edgePoint;
	float grad;
	int   interVal;
	int	  gray;
}stLanePoints;

typedef struct
{
	int   tracked;
	float preTheta;
	float curTheta;
	float bvTheta;

	Point bvEnd;
	Point preBvEnd;

	Vec4f preLane;
	Vec4f curLane;
	Vec4f bvLane;
	Vec4f preBvLane;
}stLane;

typedef struct 
{
	int left_start_th;      //左边报警起始点      300 （430 - W_car/2）
	int left_end_th;        //左边报警终止点      360 (430 - W_car/2 + 60)

	int right_start_th;     //右边报警起始点      550 (430 + W_car/2)
	int right_end_th;       //右边报警终止点      490 (430 + W_car/2 - 60)

	int Max_left;            //左线最大值         520 (430 + W_car/2 - 30)
	int Min_right;           //右线最小值         340（430 - W_car/2 + 30） 

	int left2right_Low;     //左右线最小间距      300 (400 - 100)
	int left2right_High;    //左右线最大间距      500 (400 + 100) 
}Thres;

class Ldw
{
public:
	Ldw();

	int warnFlag, preWarnFlag;
	int departFlag;
	int rightGetFlag;
	int leftGetFlag;
	float pre_main_direct;
	Mat frame;			//		大图
	Mat smallFrame;		//		1/4小图
	Mat tran_H;
	CascadeClassifier cascade;

	void Detect(void);

	int pos0, pos1;
	int carWidth;
	float leftDist;
	float rightDist;

	// 生成mask图像与cannyRoi区域
	vector<Rect> laneTargets;
	int InitMaskAndRoi(Rect roi, Mat& mask, Rect& cannyRoi, stLane& line);
	
	void InitEdgePoints(Rect roi, Mat mask);

	void InitMainDirectPoints(void);

	void FitLine(stLane& lane);

	void PlotSingleLine(Mat& frame, Vec4f& plotLine, double dL);

	inline void GetPointInLine(Vec4f line, int y, Point& point);

	bool LaneStat(stLane& leftLine, stLane& rightLine);

	void ADS_LaneDetect_Test_Show_point();

	void ADS_LaneDetect_get_other_lane(Vec4f& one_line, int dir_flag, Vec4f& other_line);

	void ADS_LD_projected_Point_BV(Point2f point, Mat& trans_Matrix, Point2f& trans_point);

	bool CheckLane(stLane& leftLine, stLane& rightLine);
	stLane leftLane, rightLane;

	int ADS_LaneDetect_Check_Circle();

	// 左侧车道线
	Mat leftMask;
	Rect leftLaneRoi;
	Rect leftCannyRoi;
	FILE *file_log;

	vector<stLanePoints> leftLanePoints;
	Mat rightMask;
	Rect rightLaneRoi;
	Rect rightCannyRoi;
	
	int turnFlag;
	int turnCount[3];
	float Dtheta;
	vector<stLanePoints> rightLanePoints;

	inline float mySobel(Mat greyFrame, int x, int y);

	
	void LaneDet(Rect roi, Mat& mask, Rect cannyRoi, vector<stLanePoints> &lanePoints,
		         stLane& lane);

	// 鸟瞰图参数
	Mat trans_Matrix;
	Mat back_trans_Matrix;

	void InitMatrixData(void);

	Vec4f leftBvLane, rightBvLane;
	void Lane2BvLane(stLane& lane);

	void AsertROI(Rect &roi);
};

