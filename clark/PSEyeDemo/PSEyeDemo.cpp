#include "stdafx.h"
#include "PSEyeDemo.h"
#include <Windows.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <string>
// OpenCV
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
// Our header files
#include "Homography.h"
#include "Image_Proc.h"
#include "Tiva.h"
// UDP
#include "xPCUDPSock.h"
#include "UDP_setup.h"
#include <ctime>

using namespace std; //allows aceess to all std lib functions without using the namespace std::
using namespace cv; // allows ... without using namespace cv::

#define FRAME_RATE 60
#define RESOLUTION CLEYE_VGA
// QVGA or VGA


/* used for passing data between main and camera thread */
typedef struct{
	CLEyeCameraInstance CameraInstance;
	Mat *Frame;
	unsigned char *FramePtr;
	int Threshold;
	int scale;
	Mat *warped_display;
	Mat *binary_display;
} CAMERA_AND_FRAME;

/*Function Prototypes*/

static DWORD WINAPI CaptureThread(LPVOID ThreadPointer); // ah this is how you get data back and forth between threads 
static DWORD WINAPI ArmThread(LPVOID);
void moveArm(CUDPReceiver *, CUDPSender *, Vec_double , PACKIN , PACKOUT , TivaController *);
void inst_taskbars(void);

//////////////////////////////////////////////////
// default segmentation values
//////////////////////////////////////////////////
int iLowH = 42;				// Hue
int iHighH = 92;
int iLowS = 40;				// Saturation
int iHighS = 255;
int iLowV = 0;				// Value
int iHighV = 255;
int gain = 0;
int exposure = 180;
//////////////////////////////////////////////////

//////////////////////////////////////////////////
// Globals - these need to be cleaned up
//////////////////////////////////////////////////
int puck_found = 0;

Mat_<double> Homography;	// H Matrix

int setup;					// Variable that tells us which state we are in
							// helps with system setup
							// variables help communicate data between the arm and image proc threads
double sendx;
double sendy;

int arm_comm = 0;
int frame_number;
vector<Vec_double> traj_line;  //used for debugging 
int FPS;
Vec_double hit_location;

//////////////////////////////////////////////////

// main function that ties everything together, threads and camera functionality will be initiated here
int _tmain(int argc, _TCHAR* argv[])
{
	int Width,Height;
	int KeyPress;
	setup = 0;
	CLEyeCameraInstance EyeCamera = NULL;

	Mat Frame;						// camera's native perspective
	Mat warped_display;				// camera perspective after the homography is applied
	Mat binary_display;				// binary, thresholded image (after erodes, dilates, and HSV application)

	CAMERA_AND_FRAME ThreadPointer;
	HANDLE _hThread;
	CLEyeCameraParameter CamCurrentParam=(CLEyeCameraParameter)0;
	bool CamParam = 0;

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

	// Launch threads and confirm they are running running
	_hThread = CreateThread(NULL, 0, &CaptureThread, &ThreadPointer, 0, 0);
	if(_hThread == NULL)
	{
		printf("Failed to create Vision thread...");
		getchar();
		return false;
	}
	_hThread = CreateThread(NULL, 0, &ArmThread, NULL, 0, 0);
	if (_hThread == NULL)
	{
		printf("Failed to create Arm Control thread...");
		getchar();
		return false;
	}

	Mat setup_img;				// image that is used during the setup stage
	double scale = 5.0;
	Point2fVector points2;		// Holds the dimension coordinates of the warped image
	points2.push_back(Point2f(0.0, 2000.0 / scale));
	points2.push_back(Point2f(1000.0 / scale, 2000.0 / scale));
	points2.push_back(Point2f(1000.0 / scale, 0.0));
	points2.push_back(Point2f(0.0, 0.0));

	//main loop that runs during camera feed operation and 
	while(1) {
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
				//imshow("initial image", setup_img);
				//MessageBoxA(NULL, "Please click four corners of the simulated air hockey table.\n"
				//	"Click the left up corner first and clockwise for the rest.",
				//	"Click", MB_OK);
				//cvSetMouseCallback("initial image", MousCallback, &points);

				////will wait for 4 mouse clicks before breaking out of loop
				//while (1)
				//{
				//	// wait for mouse clicks
				//	waitKey(10);
				//	if (points.size() == 4)
				//	{
				//		cout << "4 points gathered" << endl;
				//		cout << points[0].x << "\t" << points[0].y<<endl;
				//		cout << points[1].x << "\t" << points[1].y << endl;
				//		cout << points[2].x << "\t" << points[2].y << endl;
				//		cout << points[3].x << "\t" << points[3].y << endl;

				//		break;
				//	}
				//}
				//getchar();
				
				points.push_back(Point2f(159, 342));
				points.push_back(Point2f(152, 146));
				points.push_back(Point2f(535, 3));
				points.push_back(Point2f(554, 475));

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
			if (setup == 0)
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

	Point points;
	points.x = 0;
	points.y = 0;
	clock_t StartTime,EndTime;

		while (1) {
			//Get Frame From Camera
			CLEyeCameraGetFrame(Instance->CameraInstance, CamImg.data);

			// DO YOUR IMAGE PROCESSING HERE
			// after we have the homography; setup==0
			if (setup >= 1)
			{
				// performs the homography, and warps image to our rectangular "rink"
				setup = 1;
				warpPerspective(CamImg, warped_display, Homography, Size(1000.0 / scale, 2000.0 / scale));
				// flip image around the y axis
				flip(warped_display, flipped_display, 1);
				warped_display = flipped_display.clone();

				// this could possible be removed without error in functionality
				imgHSV = warped_display.clone();

				// apply the HSV to the
				cvtColor(warped_display, imgHSV, COLOR_BGR2HSV);

				// take a threshold of the image based off of the HSV values
				inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresholded);

				// this line sets up final_thresh's width and height params to that of thresholded
				final_thresh = thresholded.clone();
				noise_reduction(final_thresh, thresholded);

				//gather the area and XY center of the centroid/contour
				oMoments = moments(final_thresh, true);
				puck_location(warped_display, oMoments, &lastx, &lasty, &lastArea, &posX, &posY, &puck_found, traj_line);

				//display the XY coordinates of the puck in real time (according to the warped image)
				//cout << posX << "\t" << posY << endl;

				setup = 1; // why is this getting set to 1 again?
				if (posX == -1.0 || posY == -1.0)
					puck_found = 0;

				// this is our tuning of the vision coordinates
				sendx = (posX)/3.04762; 
				sendy = (posY)/2.985;
				//cout << sendx << "\t" << sendy << endl;
				// tell the arm we are ready
				if (arm_comm == 0)
					arm_comm = 1;
				circle(warped_display, Point(hit_location.x*3.04762, hit_location.y*2.985), 2, Scalar(0, 255, 0), 2, 8, 0);
				flip(warped_display, flipped_display, 0);

				*(Instance->warped_display) = flipped_display;
				*(Instance->binary_display) = final_thresh;
				setup = 2; // what does this do?
			}

			//copy it to main thread image.
			if (setup == 0)
			{
				//this can change later
				circle(CamImg, Point(419, 364), 2, Scalar(255, 0, 255), 2, 8, 0);
				circle(CamImg, Point(273, 243), 2, Scalar(255, 0, 255), 2, 8, 0);
				circle(CamImg, Point(412, 121), 2, Scalar(255, 0, 255), 2, 8, 0);
				//circle(CamImg, Point(463, 385), 2, Scalar(255, 0, 255), 2, 8, 0);
			}
			*(Instance->Frame) = CamImg;
			//imshow("Camera Feed",CamImg);

			// Track FPS
			++frame_number; // difference between frame_number and FrameCounter?
			if (FramerCounter == 0) StartTime = clock();
			FramerCounter++;
			EndTime = clock();
			if ((EndTime - StartTime) / CLOCKS_PER_SEC >= 1) {
				//cout << "FPS:" << FramerCounter << endl;
				FPS = FramerCounter;
				FramerCounter = 0;
			}
		}
	return 0;
}

// Thread does takes care of all the arm movement
// I would like to break the two threads into separate cpp files if possible?
static DWORD WINAPI ArmThread(LPVOID)
{

	if (!InitUDPLib())
	{
		cout << "UDP Failed" << endl;
		exit(0);
	}
	else {
		//33.0 -10.0 (x offset , yoffset)
		TivaController Tiva = TivaController(1.0, 46.6163, 25.0825, 30, -23);
		Vec_double corner_cases;
		vector<Vec_double> left_corner;
		vector<Vec_double> right_corner;
		//timing variables
		clock_t start, end;
		
		///// Don't worry I am going to write a method that lets us trace out paths like this

		//right corner case
		corner_cases.x = 27; corner_cases.y = 12;  right_corner.push_back(corner_cases);
		corner_cases.x = 20; corner_cases.y = 8;  right_corner.push_back(corner_cases);
		corner_cases.x = 12.3; corner_cases.y = 6;  right_corner.push_back(corner_cases);
		corner_cases.x = 6; corner_cases.y = 8;  right_corner.push_back(corner_cases);
		corner_cases.x = 7; corner_cases.y = 14;  right_corner.push_back(corner_cases);

		//left corner case
		corner_cases.x = 44.8; corner_cases.y = 12;  left_corner.push_back(corner_cases);
		corner_cases.x = 50.4; corner_cases.y = 8;  left_corner.push_back(corner_cases);
		corner_cases.x = 58.2; corner_cases.y = 6;  left_corner.push_back(corner_cases);
		corner_cases.x = 61.0; corner_cases.y = 8;  left_corner.push_back(corner_cases);
		corner_cases.x = 61.0; corner_cases.y = 14;  left_corner.push_back(corner_cases);

		// Initialize UDP communication
		CUDPReceiver receiver(sizeof(PACKIN), 12403);
		CUDPSender sender(sizeof(PACKOUT), 12302, "127.0.0.1");

		// Define buffers for input and output
		PACKIN pkin;
		PACKOUT pkout;

		// Puck initialization variables
		double radius = 3.15;		// puck radius in cm
		double widthCm = 66.0;		// rink width in cm
		double heightCm = 134.0;	// rink height in cm
		Vec_double initAcl;			// default puck acceleration 
		initAcl.x = 0.0;			// we may include this in the future
		initAcl.y = 0.0;

		int sample_size = 4;				// number of samples to use for velocity prediction
		Vec_double point;					// a single puck point
		vector<Vec_double> puck_points;		// vector of samples puck points
		std::vector<Vec_double> trajectory; // vector of future puck points
		int estimation_size = 30;			// number of frames to look ahead into the future

		// arm home position
		Vec_double home;
		home.x = 33;
		home.y = 20;
		int home_status = 0;				//0 = not at home
		
		//puck disappears variable.  If the puck disappears at any point during frame sampling set disappear to 1
		int disappear = 0;
		// set arm to home - make sure to manually move the arm home before starting the program!
		Tiva.moveArm(home, false);

		//endless loop that will run until program quits
		while (1)
		{
			if (puck_found == 0) // go home
			{
				if (home_status == 0)
				{
					std::vector<Vec_double> arm_path = Tiva.computePath(Tiva.getArm2Location(), home, 250);

					for (auto point : arm_path) 
					{
						receiver.GetData(&pkin);
						Tiva.moveArm(point, false);
						pkout.flt1 = (float)Tiva.getMotor1AngleDegrees();
						pkout.flt2 = (float)Tiva.getMotor2AngleDegrees();
						sender.SendData(&pkout);
						Sleep(1);
					}
					traj_line.clear();
					home_status = 1;
				}
			}
			else if (arm_comm == 1)
			{
				puck_points.clear(); // remove all previous points
				//Sleep(5);
				// acquire puck locations over sample size
				while (puck_points.size() < sample_size)
				{
					frame_number = 1;
					if (sendx < 0 || sendy < 0) //the puck is gone
					{
						disappear = 1;
						break;
					}
					point.x = sendx;
					point.y = sendy;
					puck_points.push_back(point);
					while (frame_number % 2 != 0) {};
				}
				//if we get garbage values then we need to continue to the next iteration in our loop
				if (disappear == 1)
				{
					arm_comm = 0;
					disappear = 0;
					continue;
				}
				start = clock();
				Puck puck = Puck(puck_points, initAcl, radius, 1.0, widthCm, heightCm);

				// vector to hold trajectory points
				std::vector<Vec_double> trajectory;

				// vector to hold path points
				std::vector<Vec_double> path;
				trajectory = puck.computeTrajectory(estimation_size);
				traj_line = trajectory;
				// Now let's hit a puck
				std::vector<Vec_double> hitPath, blockPath;

				// Define target - the center of the goal
				Vec_double targetPoint;
				targetPoint.x = 33.0;
				targetPoint.y = 136.0;
				//Dylan is a wienie and he knows it

				blockPath = Tiva.computeHitPath(trajectory, targetPoint, (double)FPS, 35.0, 10.0, 10.0, 200, "block");
				//blockPath = Tiva.computeHitPath(trajectory, targetPoint, (double)FPS, 25.0, 10.0, 10.0, 300, "hit");
				end = clock();
				//cout << "clock time: " << ((double)(end - start)) / CLOCKS_PER_SEC << endl;
				
				if (blockPath.size() > 0) // check if the path contains points
				{
					//std::cout << "hit path" << std::endl;
					hit_location = blockPath.back();

					//for (auto point : blockPath)
					//{
					//	//std::cout << point.x << " " << point.y << std::endl;
					//	receiver.GetData(&pkin);
					//	Tiva.moveArm(point, false);
					//	pkout.flt1 = (float)Tiva.getMotor1AngleDegrees();
					//	pkout.flt2 = (float)Tiva.getMotor2AngleDegrees();
					//	sender.SendData(&pkout);
					//	Sleep(1);
					//}
					home_status = 0;
					arm_comm = 0;
				}
	
			}
		}
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

void moveArm(CUDPReceiver *receiver, CUDPSender *sender, Vec_double point, PACKIN pkin, PACKOUT pkout, TivaController *Tiva)
{
	receiver->GetData(&pkin);
	Tiva->moveArm(point, false);
	pkout.flt1 = (float)Tiva->getMotor1AngleDegrees();
	pkout.flt2 = (float)Tiva->getMotor2AngleDegrees();
	sender->SendData(&pkout);
	Sleep(1);
}

