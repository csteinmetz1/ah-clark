// OpenCVCam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "PSEyeDemo.h"
#include <Windows.h>
#include <iostream>
#include <iomanip>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/tracking.hpp>
//#include <opencv2/core/ocl.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std; //allows aceess to all std lib functions without using the namespace std::
using namespace cv; // allows ... without using namespace cv::


#define FRAME_RATE 60
#define RESOLUTION CLEYE_VGA
// QVGA or VGA
typedef vector<Point2f> Point2fVector;
typedef struct{
	CLEyeCameraInstance CameraInstance;
	Mat *Frame;
	unsigned char *FramePtr;
	int Threshold;
	int scale;
	Mat *warped_display;
	Mat *binary_display;
}CAMERA_AND_FRAME;


static DWORD WINAPI CaptureThread(LPVOID ThreadPointer);

void printMatrix(const Mat_<double>& C)
{
	cout << setprecision(3) << right << fixed;

	for (int row = 0; row < C.rows; ++row)
	{
		for (int col = 0; col < C.cols; ++col)
		{
			cout << setw(5) << C(row, col) << " ";
		}
		cout << endl;
	}
}

void MousCallback(int mEvent, int x, int y, int flags, void* param)
{
	Point2fVector* pPointVec = (Point2fVector*)param;
	if (mEvent == CV_EVENT_LBUTTONDOWN)
	{
		pPointVec->push_back(Point2f(float(x), float(y)));
	}
}
//Hue
//Saturation
//Value
int iLowH = 0;
int iHighH = 179;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;
Mat_<double> Homography;
int setup;



int _tmain(int argc, _TCHAR* argv[])
{
	int Width,Height;
	int KeyPress;
	setup = 0;
	CLEyeCameraInstance EyeCamera=NULL;

	Mat Frame;
	Mat warped_display;
	Mat binary_display;

	CAMERA_AND_FRAME ThreadPointer;
	HANDLE _hThread;
	CLEyeCameraParameter CamCurrentParam=(CLEyeCameraParameter)0;
	bool CamParam=0;

	Point2fVector points;
	//////////////////////

	//////////////////// EYE CAMERA SETUP///////////////////////////////////
	// all of this code and more is included in my header file CameraControl.h hence why its commented out
	// I left it here simply for your reference
	EyeCamera=StartCam(FRAME_RATE,RESOLUTION);//this does all the commented out code

	// Get camera frame dimensions;
	CLEyeCameraGetFrameDimensions(EyeCamera, Width, Height);
	// Create a window in which the captured images will be presented
	namedWindow( "Camera", CV_WINDOW_AUTOSIZE );
	//Make a image to hold the frames captured from the camera
	Frame=Mat(Height,Width,CV_8UC4);//8 bit unsiged 4 channel image for Blue Green Red Alpa (8 bit elements per channel)
									//Start the eye camera
	CLEyeCameraStart(EyeCamera);

	/////////////////////////////////////MAIN CODE//////////////////////////////////////

	// For high frame rate launch a seperate thread

	//Need to copy vars into one var to launch the second thread
	ThreadPointer.CameraInstance=EyeCamera;
	ThreadPointer.Frame = &Frame;
	ThreadPointer.warped_display = &warped_display;
	ThreadPointer.binary_display = &binary_display;

	ThreadPointer.Threshold=0;
	//Launch thread and confirm its running
	//int setup = 1;
	_hThread = CreateThread(NULL, 0, &CaptureThread, &ThreadPointer, 0, 0);
	if(_hThread == NULL)
	{
		printf("Failed to create thread...");
		getchar();
		return false;
	}
	
	//set up the homography
	//copy the current frame over for setup
	Mat setup_img;
	
	

	double scale = 5.0;
	Point2fVector points2;
	//set up the manual dimensions

	
	//do the homography and get the H matrix

	
	//Mat unwarpImage;

	

	//Mat DT_img;//detection tracking image
	//main loop that runs during camera feed operation
	while( 1 ) {

		//This will capture keypresses and do whatever you want if you assign the appropriate actions to the right key code
		KeyPress = waitKey(1);
		switch (KeyPress){
			case 27: //escape pressed
				return 0;
				break;
			case 13: //enter key pressed, begin setup.  This can only be done once
				setup_img = Frame.clone();
				imshow("initial image", setup_img);
				MessageBoxA(NULL, "Please click four corners of the simulated air hockey table.\n"
					"Click the left up corner first and clockwise for the rest.",
					"Click", MB_OK);
				cvSetMouseCallback("initial image", MousCallback, &points);

				while (1)
				{
					// wait for mouse clicks
					waitKey(10);
					if (points.size() == 4)
					{
						cout << "4 points gathered" << endl;
						break;
					}
				}
				points2.push_back(Point2f(0.0, 2000.0 / scale));
				points2.push_back(Point2f(1000.0 / scale, 2000.0 / scale));
				points2.push_back(Point2f(1000.0 / scale, 0.0));
				points2.push_back(Point2f(0.0, 0.0));

				Homography = findHomography(Mat(points), Mat(points2));

				cout << "The transformation Matrix is :" << endl;
				printMatrix(Homography);
				cout << endl;

				//warpPerspective(setup_img, unwarpImage, H, Size(1000.0 / scale, 2000.0 / scale));
				//slider bars for adjusting the hue, saturation, and value settings
				namedWindow("Control",CV_WINDOW_AUTOSIZE);
				
				cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
				cvCreateTrackbar("HighH", "Control", &iHighH, 179);

				cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
				cvCreateTrackbar("HighS", "Control", &iHighS, 255);

				cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
				cvCreateTrackbar("HighV", "Control", &iHighV, 255);

				//imshow("warped image", unwarpImage);
				setup = 1;



				KeyPress = 0;
				break;
			default: //do nothing
				break;
		}
		
		//Display the captured frame
		imshow( "Camera", Frame );
		if (setup == 2)
		{

			imshow("Warped", warped_display);
			imshow("Binary Image", binary_display);
		}
	}
	
	CLEyeCameraStop(EyeCamera);
	CLEyeDestroyCamera(EyeCamera);
	EyeCamera = NULL;
	
	cvSetMouseCallback("Camera Input", NULL, NULL);
	while (1)
	{
		waitKey(10);
	}

	return 0;
}

///////////////////////SUB THREAD///////////////////////////
//for high frame rates you will process images here the main function will allow interactions and display only
static DWORD WINAPI CaptureThread(LPVOID ThreadPointer){
	CAMERA_AND_FRAME *Instance=(CAMERA_AND_FRAME*)ThreadPointer; //type cast the void pointer back to the proper type so we can access its elements

	int FramerCounter=0;
	Mat CamImg=Mat(*(Instance->Frame)).clone();
	Mat warped_display;// = Mat(*(Instance->Warped));
	Mat binary_display;
	int scale = 5;
	Mat imgHSV;
	Mat thresholded;

	int lastx = -1;
	int lasty = -1;

	double dM01;
	double dM10;
	double dArea;
	int breakit = 268;
	
	int posX, posY;
	Moments oMoments;
	int change_amt = 30;
	int circle_rad = 0;

	Point points;
	points.x = 0;
	points.y = 0;
	clock_t StartTime,EndTime;
	while(1){
		//Get Frame From Camera
		CLEyeCameraGetFrame(Instance->CameraInstance,CamImg.data);
		

		// DO YOUR IMAGE PROCESSING HERE
		//after we have the homography; setup==0
		if (setup >= 1)
		{

			warpPerspective(CamImg, warped_display, Homography, Size(1000.0 / scale, 2000.0 / scale));
			imgHSV = warped_display.clone();
			cvtColor(warped_display, imgHSV, COLOR_BGR2HSV);
			inRange(imgHSV,Scalar(iLowH,iLowS,iLowV),Scalar(iHighH,iHighS,iHighV),thresholded);

			erode(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
			dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

			erode(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

			oMoments = moments(thresholded,true);
			dM01 = oMoments.m01;
			dM10 = oMoments.m10;
			dArea = oMoments.m00;

			if (dArea > 10)
			{
				posX = dM10 / dArea;
				posY = dM01 / dArea;
				//stuff if there is a acceptable difference in the last and current then we change the last if ()
				if (lastx >= posX+change_amt || lastx <= posX-change_amt || lasty >= posY+change_amt || lasty <= posY - change_amt)
				{
					if (lastx != -1)
						posX = lastx;
					if (lasty != -1)
						posY = lasty;

					points.x = posX;
					points.y = posY;
					//stuff if there is a acceptable difference in the last and current then we change the last if ()
					circle_rad = dArea / 10;
					circle(warped_display, Point(posX, posY), circle_rad%100, Scalar(255, 0, 255), 2, 8, 0);
				}
				/*if (lastx != -1)
					posX = lastx;
				if (lasty != -1)
					posY = lasty;

				points.x = posX;
				points.y = posY;
				//stuff if there is a acceptable difference in the last and current then we change the last if ()

				circle(warped_display,Point(posX,posY),dArea/10,Scalar(255,0,255),2,8,0);*/
			}
			cout << "Puck location: X-" << posX << " Y-" << posY << endl;
			setup = 1;
			*(Instance->warped_display) = warped_display;

			*(Instance->binary_display) = thresholded;
			setup = 2;





		}

		//copy it to main thread image.
		*(Instance->Frame) = CamImg;
		//imshow("Camera Feed",CamImg);

		// Track FPS
		if(FramerCounter==0) StartTime=clock();
		FramerCounter++;
		EndTime=clock();
		if((EndTime-StartTime)/CLOCKS_PER_SEC>=1){
			cout << "FPS:" << FramerCounter << endl;
			FramerCounter=0;
		}
	}
	return 0;
}

