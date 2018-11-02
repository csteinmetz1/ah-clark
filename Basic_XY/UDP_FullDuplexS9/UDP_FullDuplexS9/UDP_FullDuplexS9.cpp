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
		float q1, q2;
		char* pEnd;
		TivaController Tiva = TivaController(1.0, 46.8313, 25.4, 33.02, -26.67);
		Tiva.resetArm();
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
			// prevent from running to fast
			Sleep(1);
			// get latest data from receiver
			receiver.GetData(&pkin);

			Vec_double setPoint;
			setPoint.x = x;
			setPoint.y = y;

			std::vector<Vec_double> path;

			path = Tiva.computePath(Tiva.getArm2Location(), setPoint, 50);

			for (auto point : path) {

				Tiva.moveArm(point, false);

				q1 = (float)Tiva.getMotor1Angle() * (180 / 3.14159265359);
				q2 = (float)Tiva.getMotor2Angle() * (180 / 3.14159265359);

				// repack the data
				pkout.flt1 = q1;
				pkout.flt2 = q2;

				//std::cout << q1 << std::endl;
				//std::cout << q2 << std::endl;
				//std::cout << point.x << " " << point.y << std::endl;

				// send the repacked data through sender
				sender.SendData(&pkout);
				Sleep(1);
			}
			
		}
	}


	return nRetCode;
}
