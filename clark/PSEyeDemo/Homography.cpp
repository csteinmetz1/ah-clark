/*Homography functionality*/
#include "stdafx.h"
#include "Homography.h"

/*Simply displays the matrix and is formatted based off of the cols and rows*/
void printMatrix(const Mat_<double>& C)
{
	cout << setprecision(3) << right << fixed;

	for (int row = 0; row < C.rows; ++row)
	{
		for (int col = 0; col < C.cols; ++col)
		{
			cout << setw(5) << C(row, col) << " ";
		}
		cout << endl;
	}
}
/*This function takes in a particular mouse even (mEvent), left click in this case.
When the left click is used, this function takes the x and y coordinate of the mouse pointer, and puts it
into a vector.  The pointer to this vector is return thru param
*/
void MousCallback(int mEvent, int x, int y, int flags, void* param)
{
	Point2fVector* pPointVec = (Point2fVector*)param;
	if (mEvent == CV_EVENT_LBUTTONDOWN)
	{
		pPointVec->push_back(Point2f(float(x), float(y)));
	}
}