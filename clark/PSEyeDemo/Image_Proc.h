#pragma once

#include "stdafx.h"
#include <Windows.h>
#include <iostream>
#include <iomanip>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Puck.h"

using namespace cv;
using namespace std;

void noise_reduction(Mat, Mat);
void puck_location(Mat, Moments, double *, double *, double *, double *, double *, int *, vector<Vec_double>);


