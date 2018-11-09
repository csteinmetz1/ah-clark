#include "stdafx.h"
#include "Image_Proc.h"
#include "Puck.h"

/*Function that does the erodes and dilates for each frame
(argument that returns the result, argument that is considered the input)*/
void noise_reduction(Mat dest, Mat source)
{
	Mat ed1, ed2;
	
	//erode and dialate to capture the contour of the puck and eliminate noise
	erode(source, ed1, getStructuringElement(MORPH_ELLIPSE, Size(4, 4)));
	dilate(ed1, ed2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	//final_thresh = thresholded.clone();

	erode(ed2, ed1, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
	dilate(ed1, dest, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));

}

void puck_location(Mat dest, Moments oMoments, double * lastx, double * lasty, double * lastArea, double *posX, double *posY, int *puck_found)
{

	// puck location will not update unless it moves within bounds set by change_amt 
	double change_amt = 60.0; // how is this value determined
	int circle_rad = 0;
	double dM01;
	double dM10;
	double dArea;
	double prevArea = 0.0;

	dM01 = oMoments.m01;
	dM10 = oMoments.m10;
	dArea = oMoments.m00;
	
	//gather the area and XY center of the centroid/contour

	//to avoid reading noise, only update the puck image under and over a specific area size
	if (dArea > 50 && dArea < 500)
	{
		*puck_found = 1;

		*posX = dM10 / dArea;
		*posY = dM01 / dArea;
		// detect the puck and start anew
		if (*lastx == -1 || *lasty == -1)
		{
			*lastx = *posX;
			*lasty = *posY;
		}

		if (*posX > *lastx + change_amt || *posX < *lastx - change_amt)
		{
			*posX = *lastx;
			*posY = *lasty;
			dArea = *lastArea;
		}

		if (*posY > *lasty + change_amt || *posY < *lasty - change_amt)
		{
			*posX = *lastx;
			*posY = *lasty;
			dArea = *lastArea;
		}
		// stuff if there is a acceptable difference in the last and current then we change the last if ()
		circle_rad = dArea / 10;

		// display a circle around the centroid of the puck.
		circle(dest, Point(*posX, *posY), circle_rad % 100, Scalar(255, 0, 255), 2, 8, 0);

		*lastArea = dArea;
		*lastx = *posX;
		*lasty = *posY;
	}
	else
	{
		puck_found = 0;
		*lastx = -1;
		*lasty = -1;
		*posX = -1;
		*posY = -1;
	}
}
