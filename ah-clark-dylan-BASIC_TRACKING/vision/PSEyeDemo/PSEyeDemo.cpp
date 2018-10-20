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
#include "Homography.h"
#include "Image_Proc.h"
//#include "UDP_FullDuplexS9.h"
#include "Tiva.h"
#include "xPCUDPSock.h"
//#include <winsock2.h>
#include "UDP_setup.h"
//#include <vector>
//#include "Puck.h"

using namespace std; //allows aceess to all std lib functions without using the namespace std::
using namespace cv; // allows ... without using namespace cv::

//void printMatrix(const Mat_<double>& C);
//void MousCallback(int mEvent, int x, int y, int flags, void* param);

#define FRAME_RATE 60
#define RESOLUTION CLEYE_VGA
// QVGA or VGA

/*used for passing data between main and camera thread*/
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
static DWORD WINAPI ArmThread(LPVOID);
void inst_taskbars(void);



int iLowH = 42;				//Hue
int iHighH = 92;

int iLowS = 40;				//Saturation
int iHighS = 255;

int iLowV = 0;				//Value
int iHighV = 255;

int gain = 0;
int exposure = 180;



Mat_<double> Homography;	//H Matrix

int setup;					//Variable that tells us which state we are in
							//helps with system setup
							
							//variables help communicate data between the arm and image proc threads
double sendx;
double sendy;
float lastQ1 = 0;
float lastQ2 = 0;

int arm_comm;


/*main function that ties everything together, threads and camera functionality will be initiated here*/
int _tmain(int argc, _TCHAR* argv[])
{
	int Width,Height;
	int KeyPress;
	setup = 0;
	CLEyeCameraInstance EyeCamera=NULL;

	Mat Frame;						//camera's native perspective
	Mat warped_display;				//camera perspective after the homography is applied
	Mat binary_display;				//binary, thresholded image (after erodes, dilates, and HSV application)

	CAMERA_AND_FRAME ThreadPointer;
	HANDLE _hThread;
	CLEyeCameraParameter CamCurrentParam=(CLEyeCameraParameter)0;
	bool CamParam=0;

	Point2fVector points;			//vector that holds the mouse click coordinates from setup image
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
	_hThread = CreateThread(NULL, 0, &CaptureThread, &ThreadPointer, 0, 0);
	if(_hThread == NULL)
	{
		printf("Failed to create thread...");
		getchar();
		return false;
	}
	_hThread = CreateThread(NULL, 0, &ArmThread, NULL, 0, 0);
	if (_hThread == NULL)
	{
		printf("Failed to create thread...");
		getchar();
		return false;
	}
	Mat setup_img;				//image that is used during the setup stage

	


	double scale = 5.0;
	Point2fVector points2;		//Holds the dimension coordinates of the warped image
	points2.push_back(Point2f(0.0, 2000.0 / scale));
	points2.push_back(Point2f(1000.0 / scale, 2000.0 / scale));
	points2.push_back(Point2f(1000.0 / scale, 0.0));
	points2.push_back(Point2f(0.0, 0.0));

	//main loop that runs during camera feed operation and 
	while( 1 ) {

		//This will capture keypresses and do whatever you want if you assign the appropriate actions to the right key code
		KeyPress = waitKey(1);
		switch (KeyPress){
			case 27: //escape pressed
				return 0;
				break;
			case 13: //enter key pressed, begin setup.  This can only be done once.  Will maybe look at
					 //doing this for a different part of the setup phase later.  This will require
				     //more different conditionals involving the setup variable.
				setup_img = Frame.clone();
				 

				//uncomment to test the coordinates
				imshow("initial image", setup_img);
				/*MessageBoxA(NULL, "Please click four corners of the simulated air hockey table.\n"
					"Click the left up corner first and clockwise for the rest.",
					"Click", MB_OK);
				cvSetMouseCallback("initial image", MousCallback, &points);

				//will wait for 4 mouse clicks before breaking out of loop
				while (1)
				{
					// wait for mouse clicks
					waitKey(10);
					if (points.size() == 4)
					{
						cout << "4 points gathered" << endl;
						cout << points[0].x << "\t" << points[0].y<<endl;
						cout << points[1].x << "\t" << points[1].y << endl;
						cout << points[2].x << "\t" << points[2].y << endl;
						cout << points[3].x << "\t" << points[3].y << endl;

						break;
					}
				}
				getchar();*/
				
				points.push_back(Point2f(167, 344));
				points.push_back(Point2f(160, 138));
				points.push_back(Point2f(565, 3));
				points.push_back(Point2f(570, 490));

				//returns the H matrix
				Homography = findHomography(Mat(points), Mat(points2));
				
				//printing the H matrix, if needed
				cout << "The transformation Matrix is :" << endl;
				printMatrix(Homography);
				cout << endl;

				//slider bars for adjusting the hue, saturation, and value settings
				//control will be the name of the window
				inst_taskbars();


				setup = 1;
				KeyPress = 0;
				break;

			default: //do nothing
				break;
		}
		

		//Display the captured frame
		try{
			imshow("Camera", Frame);
		}
		catch(exception&)
		{
			cout << "camera exception" << endl;
		}
		//Dispay warped and 
		if (setup == 2)
		{
			try{
				imshow("Warped", warped_display);
			}
			catch (exception&)
			{
				cout << "warped exception" << endl;
			}
			try{
				imshow("Binary Image", binary_display);
			}
			catch (exception&)
			{
				cout << "binary image exception" << endl;
			}
		}
		CLEyeSetCameraParameter(EyeCamera, CLEYE_GAIN, gain);
		CLEyeSetCameraParameter(EyeCamera, CLEYE_EXPOSURE, exposure);
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
	Mat warped_display;
	Mat binary_display;
	Mat flipped_display;
	int scale = 5;
	Mat imgHSV;				//warped image after HSV is applied
	Mat thresholded, ed1, ed2, final_thresh;

	double lastx = -1.0;
	double lasty = -1.0;

	double dM01;
	double dM10;
	double dArea;
	double lastArea = 0.0;
	
	double posX, posY;
	Moments oMoments;
	int change_amt = 30;				//puck location will not update unless it moves within bounds set by change_amt
	int circle_rad = 0;




	Point points;
	points.x = 0;
	points.y = 0;
	clock_t StartTime,EndTime;

	


		while (1) {
			//Get Frame From Camera
			CLEyeCameraGetFrame(Instance->CameraInstance, CamImg.data);


			// DO YOUR IMAGE PROCESSING HERE
			//after we have the homography; setup==0
			if (setup >= 1)
			{
				//what performs the homography, and warps image to our rectangular "rink"
				setup = 1;
				warpPerspective(CamImg, warped_display, Homography, Size(1000.0 / scale, 2000.0 / scale));

				//flip image around the y axis
				flip(warped_display, flipped_display, 1);
				warped_display = flipped_display.clone();

				//this could possible be removed without error in functionality
				imgHSV = warped_display.clone();

				//apply the HSV to the
				cvtColor(warped_display, imgHSV, COLOR_BGR2HSV);

				//take a threshold of the image based off of the HSV values
				inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresholded);

				//this line sets up final_thresh's width and height params to that of thresholded
				final_thresh = thresholded.clone();
				noise_reduction(final_thresh, thresholded);


				//gather the area and XY center of the centroid/contour
				oMoments = moments(final_thresh, true);
				puck_location(warped_display, oMoments, &lastx, &lasty, &lastArea, &posX, &posY);

				//display the XY coordinates of the puck in real time (according to the warped image)
				//cout << posX << "\t"<< posY << endl; 
				setup = 1;
				sendx = (posX - 9.1)*0.368715;
				sendy = (posY - 5.5)*0.345896;
				//Send coordinates to arm
				if (FramerCounter % 30 == 0)
				{
					arm_comm = 1;
					//if (sendx > 75)
						//sendx = 75;
					//if (sendx < 10)
						//sendx = 10;
					

					
					cout <<"Image: "<< sendx << "\t" << sendy << endl;

				}


				*(Instance->warped_display) = warped_display;

				*(Instance->binary_display) = final_thresh;
				setup = 2;
			}

			//copy it to main thread image.
			if (setup == 0)
			{
				//this can change later
				circle(CamImg, Point(438, 372), 2, Scalar(255, 0, 255), 2, 8, 0);
				circle(CamImg, Point(285, 244), 2, Scalar(255, 0, 255), 2, 8, 0);
				circle(CamImg, Point(434, 121), 2, Scalar(255, 0, 255), 2, 8, 0);
				//circle(CamImg, Point(463, 385), 2, Scalar(255, 0, 255), 2, 8, 0);


			}
			*(Instance->Frame) = CamImg;
			//imshow("Camera Feed",CamImg);

			// Track FPS
			if (FramerCounter == 0) StartTime = clock();
			FramerCounter++;
			EndTime = clock();
			if ((EndTime - StartTime) / CLOCKS_PER_SEC >= 1) {
				cout << "FPS:" << FramerCounter << endl;
				FramerCounter = 0;
			}
		}
	
	return 0;
}


//Thread does takes care of all the arm movement
//communication
//processing of game logic
//velocity
static DWORD WINAPI ArmThread(LPVOID)
{
	/*puck and tiva variables*/
	if (!InitUDPLib())
	{
		cout << "UDP Failed" << endl;
		exit(0);
	}
	else {
		//33.0 -10.0 (x offset , yoffset)
		TivaController Tiva = TivaController(1.0, 46.6163, 25.0825, 33.02, -15);

		// instantiate set point for arm
		Vec_double setPoint;

		setPoint.x = 0;
		setPoint.y = 0;

		Tiva.moveArm(setPoint, false);

		Vec_double initPos;
		Vec_double initVel;
		Vec_double initAcl;

		initPos.x = 30.0;
		initPos.y = 60.0;

		initVel.x = -0.5;
		initVel.y = -2.0;

		initAcl.x = -0.0125;
		initAcl.y = -0.0125;

		double radius = 50.0;
		double widthCm = 66.0;
		double heightCm = 136.0;
		//Puck puck = Puck(initPos, initVel, initAcl, radius, 1.0, widthCm, heightCm);

		//std::vector<Vec_double> trajectory;

		int estimation_size = 60;

		//trajectory = puck.computeTrajectory(puck, estimation_size);

		// Create receiver, with packet size equal to that of PACKIN and port at 12403 or the output port for the Tiva in virtual port 3
		CUDPReceiver receiver(sizeof(PACKIN), 12403);

		// Create sender, with packet size equal to that of PACKOUT and port at port is 12302 or input port for the Tiva in virtual port 2, 
		// and remote address 127.0.0.1(localhost)
		CUDPSender sender(sizeof(PACKOUT), 12302, "127.0.0.1");

		// Define buffers for input and output
		PACKIN pkin;
		PACKOUT pkout;
		char string[256];
		//float x, y;
		float q1, q2;// lastQ1, lastQ2;
		char* pEnd;

		int nRetCode = 0;
		int userInput = 0;

		//endless loop that will run until program quits
		//
		while (1)
		{
			if (arm_comm == 1)
			{
				if (sendx < 10 && sendx>0)
					sendx = 10;
				if (sendy < 15 && sendy >0)
					sendy = 15;
				if ((sendy) > 50)
				{
					sendy = 45;
					//moveArm(sendx, 30, &lastQ1, &lastQ2);
					receiver.GetData(&pkin);
					setPoint.x = sendx;
					setPoint.y = sendy;
					Tiva.moveArm(setPoint, false);

					q1 = (float)Tiva.getMotor1Angle() * (180 / 3.14159265359);
					q2 = (float)Tiva.getMotor2Angle() * (180 / 3.14159265359);

					cout << "~~~TIVA~~~" << q1 << "," << q2 << endl;
					// repack the data
					//Send most up to date information if the value isn't NaN, otherwise use
					//last good known value
					if (!isnan(q1) && !isnan(q2) && sendx > 0 && sendy > 0) {
						pkout.flt1 = q1;
						pkout.flt2 = q2;
						lastQ1 = q1;
						lastQ2 = q2;
					}
					else {
						cout << "NAN ERROR" << endl;
						pkout.flt1 = lastQ1;
						pkout.flt2 = lastQ2;
					}
					sender.SendData(&pkout);


				}
				else {
					receiver.GetData(&pkin);

					setPoint.x = sendx;
					setPoint.y = sendy;
					Tiva.moveArm(setPoint, false);

					q1 = (float)Tiva.getMotor1Angle() * (180 / 3.14159265359);
					q2 = (float)Tiva.getMotor2Angle() * (180 / 3.14159265359);

					cout << "~~~TIVA~~~" << q1 << "," << q2 << endl;
					// repack the data
					//Send most up to date information if the value isn't NaN, otherwise use
					//last good known value
					if (!isnan(q1) && !isnan(q2) && sendx > 0 && sendy > 0) {
						pkout.flt1 = q1;
						pkout.flt2 = q2;
						lastQ1 = q1;
						lastQ2 = q2;
					}
					else {
						cout << "NAN ERROR" << endl;
						pkout.flt1 = lastQ1;
						pkout.flt2 = lastQ2;
					}
					//moveArm(sendx, sendy, &lastQ1, &lastQ2);
					sender.SendData(&pkout);
				}
				arm_comm = 0;
			}

		}
		// Initialize the UDP lib. If failed, quit running.
	}
}







//function that instantiates the trackbars with sliders
void inst_taskbars(void)
{
	namedWindow("Control", CV_WINDOW_AUTOSIZE);
	namedWindow("Cam Control", CV_WINDOW_AUTOSIZE);

	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);
	
	cvCreateTrackbar("Gain", "Cam Control", &gain, 255);
	cvCreateTrackbar("Exposure", "Cam Control", &exposure, 255);
}


