#include "stdafx.h"
#include "PSEyeDemo.h"
#include <Windows.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <ctime>
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

using namespace std; //allows aceess to all std lib functions without using the namespace std::
using namespace cv; // allows ... without using namespace cv::

#define FRAME_RATE 60
#define RESOLUTION CLEYE_VGA
// QVGA or VGA


/* used for passing data between main and camera thread */
typedef struct {
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
static DWORD WINAPI TrajectoryThread(LPVOID);

void moveArm(CUDPReceiver *, CUDPSender *, Vec_double, PACKIN, PACKOUT, TivaController *);
void inst_taskbars(void);

//////////////////////////////////////////////////
// default segmentation values
//////////////////////////////////////////////////
int iLowH = 42;				// Hue
int iHighH = 92;
int iLowS = 60;				// Saturation
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

int home_status = 0;				// 0 = not at home
int penalty_status = 0;				// 0 = not at penalty

int frame_number;
int FPS;
Vec_double hit_location;

string game_mode = "normal";    //the three possible modes are normal, penalty defense, penalty offense

vector<Vec_double> blockPath;

// Puck initialization variables
double radius = 3.15;		// puck radius in cm
double widthCm = 66.0;		// rink width in cm
double heightCm = 134.0;	// rink height in cm

Puck puck(radius, 1.0, widthCm, heightCm);

bool essential_ops = true;

//////////////////////////////////////////////////

// main function that ties everything together, threads and camera functionality will be initiated here
int _tmain(int argc, _TCHAR* argv[])
{
	int Width, Height;
	int KeyPress;
	setup = 0;
	CLEyeCameraInstance EyeCamera = NULL;

	Mat Frame;						// camera's native perspective
	Mat warped_display;				// camera perspective after the homography is applied
	Mat binary_display;				// binary, thresholded image (after erodes, dilates, and HSV application)

	CAMERA_AND_FRAME ThreadPointer;
	HANDLE _hThread;
	CLEyeCameraParameter CamCurrentParam = (CLEyeCameraParameter)0;
	bool CamParam = 0;

	Point2fVector points;			//vector that holds the mouse click coordinates from setup image
	//////////////////////

	//////////////////// EYE CAMERA SETUP///////////////////////////////////
	// all of this code and more is included in my header file CameraControl.h hence why its commented out
	// I left it here simply for your reference
	EyeCamera = StartCam(FRAME_RATE, RESOLUTION);//this does all the commented out code

	// Get camera frame dimensions;
	CLEyeCameraGetFrameDimensions(EyeCamera, Width, Height);
	// Create a window in which the captured images will be presented
	namedWindow("Camera", CV_WINDOW_AUTOSIZE);
	//Make a image to hold the frames captured from the camera
	Frame = Mat(Height, Width, CV_8UC4);//8 bit unsiged 4 channel image for Blue Green Red Alpa (8 bit elements per channel)
									//Start the eye camera
	CLEyeCameraStart(EyeCamera);

	/////////////////////////////////////MAIN CODE//////////////////////////////////////

	// For high frame rate launch a seperate thread
	//Need to copy vars into one var to launch the second thread
	ThreadPointer.CameraInstance = EyeCamera;
	ThreadPointer.Frame = &Frame;
	ThreadPointer.warped_display = &warped_display;
	ThreadPointer.binary_display = &binary_display;
	ThreadPointer.Threshold = 0;

	// Launch threads and confirm they are running running
	_hThread = CreateThread(NULL, 0, &CaptureThread, &ThreadPointer, 0, 0);
	if (_hThread == NULL)
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

	_hThread = CreateThread(NULL, 0, &TrajectoryThread, NULL, 0, 0);
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
	while (1) {
		//This will capture keypresses and do whatever you want if you assign the appropriate actions to the right key code
		KeyPress = waitKey(1);
		switch (KeyPress) {
		case 113:
			game_mode = "normal";
			penalty_status = 0;
			cout << "game mode is normal"<<endl;
			break;
		case 119:
			game_mode = "penalty defense";
			cout << "game mode is penalty defense" << endl;
			break;
		case 101:
			game_mode = "penalty offense";
			penalty_status = 0;
			cout << "game mode is penalty offense" << endl;
			break;
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
		try {
			if (setup == 0)
				imshow("Camera", Frame);
		}
		catch (exception&)
		{
			cout << "camera exception" << endl;
		}
		//Dispay warped and 
		if (setup == 2 && essential_ops)
		{
			try {
				imshow("Warped", warped_display);
			}
			catch (exception&)
			{
				cout << "warped exception" << endl;
			}
			try {
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
static DWORD WINAPI CaptureThread(LPVOID ThreadPointer) {
	CAMERA_AND_FRAME *Instance = (CAMERA_AND_FRAME*)ThreadPointer; //type cast the void pointer back to the proper type so we can access its elements

	int FramerCounter = 0;
	Mat CamImg = Mat(*(Instance->Frame)).clone();
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
	clock_t StartTime, EndTime;

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
			//flip(warped_display, flipped_display, 1);
			//warped_display = flipped_display.clone();

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
			puck_location(warped_display, oMoments, &lastx, &lasty, &lastArea, &posX, &posY, &puck_found);

			//display the XY coordinates of the puck in real time (according to the warped image)
			//cout << posX << "\t" << posY << endl;

			setup = 1; // why is this getting set to 1 again?
			if (posX == -1.0 || posY == -1.0)
				puck_found = 0;

			// this is our tuning of the vision coordinates
			sendx = (posX) / 3.04762;
			sendy = (posY) / 2.985;
			//cout << sendx << "\t" << sendy << endl;

			//not essential
			if (essential_ops)
			{
				vector<Vec_double>  trajectory = puck.getTrajectory();
				vector<Vec_double> path = blockPath;
				if (trajectory.size() > 0 && 1)
				{
					for (auto point : trajectory)
					{
						point.x = point.x * 3.04762;
						point.y = point.y * 2.985;
						circle(warped_display, Point(point.x, point.y), 1, Scalar(255, 0, 0), 2, 8, 0);
					}
				}

				if (path.size() > 0 && 1)
				{
					for (int i = 0; i < path.size() - 1; i += 50)
					{
						path[i].x = path[i].x * 3.04762;
						path[i].y = path[i].y * 2.985;
						circle(warped_display, Point(path[i].x, path[i].y), 1, Scalar(244, 205, 65), 2, 8, 0);
					}
				}


				circle(warped_display, Point(hit_location.x*3.04762, hit_location.y*2.985), 2, Scalar(0, 255, 0), 2, 8, 0);

				flip(warped_display, flipped_display, 0);
				//flip(flipped_display, warped_display, 1);
				flip(final_thresh, *(Instance->binary_display),0);

				*(Instance->warped_display) = flipped_display;
				//*(Instance->binary_display) = final_thresh;
			}
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

	Sleep(1000);

	if (!InitUDPLib())
	{
		cout << "UDP Failed" << endl;
		exit(0);
	}
	else {
		TivaController Tiva = TivaController(1.0, 46.75, 24.25, 34.0, -21.5);   //modified 11/25/2018
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

		int sample_size = 4;				// number of samples to use for velocity prediction
		Vec_double point;					// a single puck point
		vector<Vec_double> puck_points;		// vector of samples puck points

		// arm home position
		Vec_double home;
		home.x = 33;
		home.y = 15.0;

		// goal zone approx. lines
		double leftLineSlope = (15.0 - 0.0) / (33.0 - 18.0);
		double rightLineSlope = (15.0 - 0.0) / (33.0 - 48.0);

		// gameplay constants 
		double velocityThreshold = 0.5; //was 1.0

		// set arm to home - make sure to manually move the arm home before starting the program!
		Tiva.moveArm(home, false);

		//endless loop that will run until program quits
		while (1)
		{
			if (game_mode == "normal")
			{
				if (puck_found == 0) // go home
				{
					if (home_status == 0)
					{
						std::vector<Vec_double> arm_path = Tiva.computeLinearPath(Tiva.getArm2Location(), home, 300, true);

						for (auto point : arm_path)
						{
							receiver.GetData(&pkin);
							Tiva.moveArm(point, false);
							pkout.flt1 = (float)Tiva.getMotor1AngleDegrees();
							pkout.flt2 = (float)Tiva.getMotor2AngleDegrees();
							sender.SendData(&pkout);
							Sleep(1);
						}
						home_status = 1;
					}
				}
				else
				{
					// Now let's hit a puck
					std::vector<Vec_double> hitPath;

					// Define target - the center of the goal
					Vec_double targetPoint;
					targetPoint.x = 33.0;
					targetPoint.y = 136.0;

					string blockType; // type of blocking mode

					double yhit = 20.0;
					double xlim = 10.0;
					double ylim = 10.0;

					double current_x = sendx;
					double current_y = sendy;

					// corner cases
					if (abs(puck.getVelocity().y) < 0.25 && abs(puck.getVelocity().x) < 0.25 && current_y > 0 && current_x > 0 
						&& ((current_y < 20 && current_x < 18) || (current_y < 20 && current_x > 44)))
					{
						Vec_double curveStart, curveEnd, hitEnd;

						if (current_x < 18)
						{
							curveStart.x = 20;
							curveStart.y = 3.5;
							curveEnd.x = 5;
							curveEnd.y = 20;
						}
						else if (current_x > 44)
						{
							curveStart.x = 44;
							curveStart.y = 0.0;
							curveEnd.x = 61;
							curveEnd.y = 20;
						}

						hitEnd.x = curveEnd.x;
						hitEnd.y = 35.0;

						vector<Vec_double> initPath, curvePath, hitPath;

						initPath = Tiva.computeLinearPath(Tiva.getArm2Location(), curveStart, 600, true);
						curvePath = Tiva.computeCurvedPath(curveStart, curveEnd, 600, 4.0);
						hitPath = Tiva.computeLinearPath(curveEnd, hitEnd, 300, true);

						std::cout << "corner" << std::endl;
						blockType = "corner";

						// pack the two paths into a sigle vector
						blockPath.reserve(initPath.size() + curvePath.size() + hitPath.size()); // preallocate memory
						blockPath.insert(blockPath.end(), initPath.begin(), initPath.end());
						blockPath.insert(blockPath.end(), curvePath.begin(), curvePath.end());
						blockPath.insert(blockPath.end(), hitPath.begin(), hitPath.end());

					}
					// follow and hit 
					else if (abs(puck.getVelocity().y) < 1.0 && abs(puck.getVelocity().x) < 1.0 && sendy < 45)
					{
						if (sendx > 0 && sendy > 0 && sendy > Tiva.getArm2Location().y)
						{
							// Eddie's method
							Vec_double hitPoint, endPoint;
							hitPoint.x = sendx;
							hitPoint.y = sendy - 5.0; // some point 'behind the puck'

							endPoint.x = sendx;
							endPoint.y = sendy + 5.0;

							std::vector<Vec_double> initPath, hitPath;
							initPath = Tiva.computeLinearPath(Tiva.getArm2Location(), hitPoint, 0, true);
							hitPath = Tiva.computeLinearPath(hitPoint, endPoint, 100, true); // was 100

							blockType = "follow+hit";

							blockPath.reserve(initPath.size() + hitPath.size()); // preallocate memory
							blockPath.insert(blockPath.end(), initPath.begin(), initPath.end());
							blockPath.insert(blockPath.end(), hitPath.begin(), hitPath.end());

							std::cout << "follow and hit" << std::endl;
						}
						else if (sendx > 0 && sendy > 0 && sendy < Tiva.getArm2Location().y)
						{
							Vec_double endPoint, startPoint;

							if (sendx >= 48.0)
							{
								endPoint.x = sendx + 5;
								endPoint.y = sendy;
							}

							else if (sendx <= 18.0)
							{
								endPoint.x = sendx - 5;
								endPoint.y = sendy;
							}
							else
							{
								continue;
							}

							blockPath = Tiva.computeLinearPath(Tiva.getArm2Location(), endPoint, 0, true);

							blockType = "follow+hit";

							std::cout << "SIDE - follow and hit" << std::endl;
						}
					}
					else if (puck.getVelocity().y > 0 && sendy > 40)
					{
						blockPath = Tiva.computeLinearPath(Tiva.getArm2Location(), home, 300, true);
						std::cout << "go home" << std::endl;
						blockType = "home";
					}
					else if (puck.getVelocity().y < 0) // may want to add an upper limit of -1.0?
					{
						blockPath = Tiva.computeBlockAndHitPath(puck.getTrajectory(), targetPoint, puck.getSampleTime(), 20.0, 0.8);
						std::cout << "block and hit" << std::endl;
						blockType = "block+hit";
					}

					if (blockPath.size() > 0 && 1) // check if the path contains points
					{
						// store starting velcoity to check against updated value
						Vec_double start_vel = puck.getVelocity();

						// send hit location to be printed on the screen as green dot
						hit_location = blockPath.back();

						for (auto point : blockPath)
						{
							// check if current path is valid for latest velocity
							if (abs(start_vel.x - puck.getVelocity().x) + abs(start_vel.y - puck.getVelocity().y) > velocityThreshold && sendy > 20.0 && blockType == "block+hit" ) 
							{
								blockPath.clear();
								break;
							}
							else if (abs(start_vel.x - puck.getVelocity().x) + abs(start_vel.y - puck.getVelocity().y) > velocityThreshold && blockType == "home")
							{
								blockPath.clear();
								break;
							}
							else if ((abs(start_vel.x - puck.getVelocity().x) + abs(start_vel.y - puck.getVelocity().y) > velocityThreshold || sendy > 45) && blockType == "follow+hit")
							{
								blockPath.clear();
								break;
							}
							else if (sendy > 45 && blockType == "corner") // stop corner case if puck leaves the zone
							{
								blockPath.clear();
								break;
							}
							else
							{
								receiver.GetData(&pkin);

								// check if the paddle will collide with the wall
								if (point.x >= 63.0) { point.x = 63.0; }
								else if (point.x <= 5.0) { point.x = 5.0; }
								else if (point.y <= 0.0) { point.y = 0.0; }
								else if (point.y >= 45.0) { point.y = 45.0; }

								// check if the paddle will go into the goal zone

								if (point.y < (leftLineSlope * (point.x - 33.0) + 15.0) && (point.x <= 33.0))
								{
									point.x = point.x; // don't change x value
									point.y = leftLineSlope * (point.x - 33.0) + 15.0;
									std::cout << "in goal zone" << std::endl;
								}
								else if (point.y < (rightLineSlope * (point.x - 33.0) + 15.0) && (point.x > 33.0))
								{
									point.x = point.x; // don't change x value
									point.y = rightLineSlope * (point.x - 33.0) + 15.0;
									std::cout << "in goal zone" << std::endl;

								}
								
								Tiva.moveArm(point, false);
								pkout.flt1 = (float)Tiva.getMotor1AngleDegrees();
								pkout.flt2 = (float)Tiva.getMotor2AngleDegrees();
								sender.SendData(&pkout);
								Sleep(1);
							}
						}
						home_status = 0;
						blockPath.clear(); // clear the last path
					}
				}
			}
			else if (game_mode == "penalty defense")
			{
				blockPath.clear();
				cout << "hello from penalty defense" << endl;

				Vec_double offField;
				offField.x = 5;
				offField.y = 5;

				if (penalty_status == 0)
				{
					blockPath = Tiva.computeLinearPath(Tiva.getArm2Location(), offField, 1000, true);
					home_status = 0;
					penalty_status = 1;

					for (auto point : blockPath)
					{
						receiver.GetData(&pkin);

						// check if the paddle will collide with the wall
						if (point.x >= 61.0) { point.x = 61.0; }
						else if (point.x <= 6.0) { point.x = 6.0; }
						else if (point.y <= 3.5) { point.y = 3.5; }
						else if (point.y >= 45.0) { point.y = 45.0; }

						Tiva.moveArm(point, false);
						pkout.flt1 = (float)Tiva.getMotor1AngleDegrees();
						pkout.flt2 = (float)Tiva.getMotor2AngleDegrees();
						sender.SendData(&pkout);
						Sleep(1);
					}
					while (penalty_status == 1) {};
					blockPath.clear();
				}

			}
			//else if (game_mode == "penalty offense")
			//{
			//	cout << "hello from penalty offense" << endl;
			//	// Eddie's method
			//	Vec_double hitPoint, endPoint;
			//	hitPoint.x = sendx;
			//	hitPoint.y = sendy - 10.0; // some point 'behind the puck'

			//	endPoint.x = sendx;
			//	endPoint.y = sendy + 5;

			//	std::vector<Vec_double> initPath, hitPath;
			//	initPath = Tiva.computeLinearPath(Tiva.getArm2Location(), hitPoint, 0, true);
			//	hitPath = Tiva.computeLinearPath(hitPoint, endPoint, 100, true);

			//	blockPath.reserve(initPath.size() + hitPath.size()); // preallocate memory
			//	blockPath.insert(blockPath.end(), initPath.begin(), initPath.end());
			//	blockPath.insert(blockPath.end(), hitPath.begin(), hitPath.end());

			//	for (auto point : blockPath)
			//	{
			//		receiver.GetData(&pkin);

			//		// check if the paddle will collide with the wall
			//		if (point.x >= 63.0) { point.x = 63.0; }
			//		else if (point.x <= 5.0) { point.x = 5.0; }
			//		else if (point.y <= 3.5) { point.y = 3.5; }
			//		else if (point.y >= 45.0) { point.y = 45.0; }

			//		Tiva.moveArm(point, false);
			//		pkout.flt1 = (float)Tiva.getMotor1AngleDegrees();
			//		pkout.flt2 = (float)Tiva.getMotor2AngleDegrees();
			//		sender.SendData(&pkout);
			//		Sleep(1);
			//	}
			//}
		}
	}
}

static DWORD WINAPI TrajectoryThread(LPVOID)
{
	Sleep(1000);
	int sample_size = 4;			// number of frames to use
	Vec_double point;				// placeholder for puck location
	vector<Vec_double> puck_points; // vector of sampled puck locations

	while (1)
	{
		// start the clock
		clock_t begin = clock();

		// sample frames 
		while (puck_points.size() < sample_size)
		{
			frame_number = 1;

			// the puck is gone
			if (sendx < 0 || sendy < 0)
			{
				puck_points.clear();	 // clear sampled points
				clock_t begin = clock(); // restart the clock
			}
			// sample a frame
			else
			{
				point.x = sendx;
				point.y = sendy;
				puck_points.push_back(point);
				while (frame_number % 2 != 0) {};
			}
		}

		clock_t end = clock(); // stop the clock
		double elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
		double sampleTime = elapsedTime / double(puck_points.size());

		puck.updatePuck(puck_points, sampleTime);		// update the position, velocity, and trajectory
		puck_points.clear();							// clear out vector to get new samples
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