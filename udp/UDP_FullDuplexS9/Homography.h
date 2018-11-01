#pragma once
#include "stdafx.h"
#include <Windows.h>
#include <iostream>
#include <iomanip>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
using namespace cv;
using namespace std;

typedef vector<Point2f> Point2fVector;


void printMatrix(const Mat_<double>& C);
void MousCallback(int mEvent, int x, int y, int flags, void* param);




//Mat_<double> Homography;	//H Matrix
