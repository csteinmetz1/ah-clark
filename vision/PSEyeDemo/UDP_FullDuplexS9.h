#pragma once

#include "stdafx.h"
#include <iostream>
//#include <winsock2.h>
// Two neccessary header files you need to include.
// Place the include for winsock2.h at the beginning of your include statements to avoid a conflict with an older library

#include "xPCUDPSock.h"
#include <stdio.h>
#include <string>
#include "Tiva.h"

void moveArm(float x, float y, float*, float*);