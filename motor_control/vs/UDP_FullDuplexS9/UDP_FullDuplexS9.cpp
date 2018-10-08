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
		float motor1, motor2;
		char* pEnd;
		// Routing data endlessly
		while (1)
		{

			printf("Enter angle for MAIN motor: ");
			fgets(string, 100, stdin);
			motor1 = strtof(string, &pEnd);

			printf("Enter angle for SECONDARY motor: ");
			fgets(string, 100, stdin);
			motor2 = strtof(string, &pEnd);
			printf("\n");
			// prevent from running to fast
			Sleep(1);
			// get latest data from receiver
			receiver.GetData(&pkin);

			/*if (value > 110) {
			value = 110;
			}
			else if (value < -110) {
			value = -110;
			}*/

			// repack the data
			pkout.flt1 = motor1;
			pkout.flt2 = motor2;

			// send the repacked data through sender
			sender.SendData(&pkout);
		}
	}


	return nRetCode;
}
