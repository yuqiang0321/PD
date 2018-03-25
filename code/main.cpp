// LDWCalibration-aruco.cpp : Defines the entry point for the console application.
//

#include "opencv.hpp"

using namespace cv;
using namespace std;

#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720

int main()
{
   
	char c = 0;
	int i = 0;
	VideoCapture capture;
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

	if (!capture.open("..\\data\\20170804105111_clip.avi"))
	{
		cout << "Cannot open the video." << endl;
		return -1;
	}

	Mat frame, greyFrame;
	while (c != 27)
	{
		i++;
		cout << "frame" << i << endl;
		capture >> frame;

		cvtColor(frame, greyFrame, CV_RGB2GRAY);
		
		imshow("re", frame);
		c = cvWaitKey(1);
	}

	return 0;
}