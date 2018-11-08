// UDP_FullDuplexS9.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
// Two neccessary header files you need to include.
// Place the include for winsock2.h at the beginning of your include statements to avoid a conflict with an older library
#include <winsock2.h>
#include "xPCUDPSock.h"
#include <stdio.h>
#include <string>
#include "Tiva.h"
#include "Draw.h"
#include <ctime>

#pragma pack(push,1) // Important! Tell the compiler to pack things up tightly 

//buffer structure definition
struct PACKIN
{
	float flt1;
	float flt2;
};

struct PACKOUT
{
	float flt1;
	float flt2;
};

#pragma pack(pop) // Fall back to previous setting

int _tmain(int argc, TCHAR* argv[])
{
	int nRetCode = 0;
	int userInput = 0;

	Sleep(10);

	// Initialize the UDP lib. If failed, quit running.
	if (!InitUDPLib())
	{

		nRetCode = 2;
	}
	else
	{
		// Create receiver, with packet size equal to that of PACKIN and port at 12403 or the output port for the Tiva in virtual port 3
		CUDPReceiver receiver(sizeof(PACKIN), 12403);

		// Create sender, with packet size equal to that of PACKOUT and port at port is 12302 or input port for the Tiva in virtual port 2, 
		// and remote address 127.0.0.1(localhost)
		CUDPSender sender(sizeof(PACKOUT), 12302, "127.0.0.1");

		// Define buffers for input and output
		PACKIN pkin;
		PACKOUT pkout;
		char string[256];
		float x, y;
		float radius;
		float q1, q2;
		char* pEnd;
		TivaController Tiva = TivaController(1.0, 46.8313, 25.4, 31.5, -23.25);
		Tiva.resetArm();

		std::vector<Vec_double> path;
		Vec_double home;
		home.x = 33;
		home.y = 20;

		int initPos = 1;
		std::vector<Vec_double> setupPath;

		// Routing data endlessly
		while (1)
		{
			printf("Enter x: ");
			fgets(string, 100, stdin);
			x = strtof(string, &pEnd);

			printf("Enter y: ");
			fgets(string, 100, stdin);
			y = strtof(string, &pEnd);
			printf("\n");

			Vec_double targetPoint;
			targetPoint.x = x;
			targetPoint.y = y;

			if (initPos == 1)
			{
				setupPath = Tiva.computePath(Tiva.getArm2Location(), home, 1000);
				for (auto point : setupPath)
				{

					receiver.GetData(&pkin);

					Tiva.moveArm(point, false);

					q1 = (float)Tiva.getMotor1AngleDegrees();
					q2 = (float)Tiva.getMotor2AngleDegrees();

					// repack the data
					pkout.flt1 = q1;
					pkout.flt2 = q2;

					//std::cout << q1 << std::endl;
					//std::cout << q2 << std::endl;
					std::cout << point.x << " " << point.y << std::endl;

					// send the repacked data through sender
					sender.SendData(&pkout);
					Sleep(1);
				}
				initPos = 0;
			}

			//
			Vec_double initPos;
			Vec_double secondPos;
			Vec_double initAcl;
			initPos.x = 33.0;
			initPos.y = 100.0;
			secondPos.x = 38.5;
			secondPos.y = 85.0;
			int frames = 10;
			initAcl.x = 0;
			initAcl.y = 0;
			double radius = 0.0;
			double widthCm = 66.0;
			double heightCm = 136.0;
			Puck puck = Puck(initPos, secondPos, initAcl, radius, 1.0, widthCm, heightCm, frames);

			clock_t startTime, endTime;
			// vector to hold trajectory points
			std::vector<Vec_double> trajectory;
			// vector to hold path points
			std::vector<Vec_double> path;
			trajectory = puck.computeTrajectory(60);
			// Now let's hit a puck
			std::vector<Vec_double> initPath;
			std::vector<Vec_double> hitPath;
			// Define target - the center of the goal
			hitPath = Tiva.computeHitPath(trajectory, targetPoint,20, 10, 10, 125);
			initPath = Tiva.computePath(Tiva.getArm2Location(), hitPath.front(), 500);
			//


			// preent from running to fast
			Sleep(1);
			// get latest data from receiver

			for (auto point : initPath) 
			{

				receiver.GetData(&pkin);

				Tiva.moveArm(point, false);

				q1 = (float)Tiva.getMotor1AngleDegrees();
				q2 = (float)Tiva.getMotor2AngleDegrees();

				// repack the data
				pkout.flt1 = q1;
				pkout.flt2 = q2;

				//std::cout << q1 << std::endl;
				//std::cout << q2 << std::endl;
				std::cout << point.x << " " << point.y << std::endl;

				// send the repacked data through sender
				sender.SendData(&pkout);
				Sleep(1);
			}
			startTime = clock();
			for (auto point : hitPath)
			{
				receiver.GetData(&pkin);

				Tiva.moveArm(point, false);

				q1 = (float)Tiva.getMotor1AngleDegrees();
				q2 = (float)Tiva.getMotor2AngleDegrees();

				// repack the data
				pkout.flt1 = q1;
				pkout.flt2 = q2;

				//std::cout << q1 << std::endl;
				//std::cout << q2 << std::endl;
				std::cout << point.x << " " << point.y << std::endl;

				// send the repacked data through sender
				sender.SendData(&pkout);
				Sleep(1);
			}
			endTime = clock();
			std::cout << "Hit puck timing: " << ((double)(endTime - startTime) / CLOCKS_PER_SEC) << std::endl;
			
		}
	}


	return nRetCode;
}
