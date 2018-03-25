// LDWCalibration-aruco.cpp : Defines the entry point for the console application.
//

#include "opencv.hpp"
//#include "aruco.hpp"
#include "Ldw.h"

using namespace cv;
using namespace std;

#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720
//#define VERTICAL_TAN_PER_PIXEL 43.25/239/360
#define VERTICAL_TAN_PER_PIXEL 61.0/96500

inline void GetPointInLine(Vec4f line, int y, Point& point)
{
	if (line[0] * line[1] == 0)
	{
		point.x = 0;
		point.y = 0;
	}

	point.x = (int)((y - line[3]) / line[1] * line[0] + line[2]);
	point.y = y;
}

void PlotSingleLine(Mat& frame, Vec4f& plotLine, double dL)
{
	Point p1, p2;

	GetPointInLine(plotLine, frame.rows * 0.5, p1);
	GetPointInLine(plotLine, frame.rows, p2);
	line(frame, p1, p2, Scalar(255, 255, 0), 10);
}


int main()
{
	Mat H;
	FileStorage fs("..\\code\\H-Í¾°².xml", FileStorage::READ);
	fs["H"] >> H;
	   
	char c = 0;
	Ldw cLdw;
	cLdw.InitMatrixData();
	VideoCapture capture;
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

	if (!capture.open("..\\data\\20170804105111_clip.avi"))
	{
		cout << "Cannot open the video." << endl;
		return -1;
	}

	cLdw.tran_H = H;
	cLdw.carWidth = 190;
	int i = 0;
	Mat frame, greyFrame;
	while (c != 27)
	{
		i++;
		cout << "frame" << i << endl;
		capture >> frame;

		cvtColor(frame, greyFrame, CV_RGB2GRAY);
		cLdw.frame = greyFrame;
		cLdw.Detect();

		if (cLdw.leftLane.tracked && cLdw.rightLane.tracked)
		{
			PlotSingleLine(frame, cLdw.leftLane.curLane, 300.0);
			PlotSingleLine(frame, cLdw.rightLane.curLane, 300.0);
		}
		
		char dist[400];
		string distPrint;
		Point pos;
		sprintf(dist, "%.1d cm", cLdw.leftLane.bvEnd.x);
		distPrint = dist;
		pos = Point(100, 700);
		putText(frame, distPrint, pos, CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 3);

		sprintf(dist, "%.1d cm", cLdw.rightLane.bvEnd.x);
		distPrint = dist;
		pos = Point(1000, 700);
		putText(frame, distPrint, pos, CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 3);

		if (cLdw.warnFlag == -1 && cLdw.preWarnFlag == -1)
		{
			putText(frame, "LEFT", cvPoint(640, 50), CV_FONT_HERSHEY_COMPLEX, 1.0, Scalar(0, 0, 0), 3);
			//printf("\a");
		}
		else if (cLdw.warnFlag == 1 && cLdw.preWarnFlag == 1)
		{
			putText(frame, "RIGHT", cvPoint(640, 50), CV_FONT_HERSHEY_COMPLEX, 1.0, Scalar(0, 0, 0), 3);
			//printf("\a");
		}

		imshow("Demo", frame);
		c = cvWaitKey(1);
	}

	return 0;
}