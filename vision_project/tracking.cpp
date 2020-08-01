#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv/cv.hpp>
#include <iostream>
#include <fstream>

#define PI 3.14159265
#define MIN_AREA 350

//For compiling in Linux
/*g++ `pkg-config --cflags opencv` track1_0.cpp `pkg-config --libs opencv` -o tracking*/

using namespace cv;
using namespace std;

// This function calculates the angle of the line from A to B with respect to the positive X-axis in degrees
int angle(Point2f A, Point2f B) {
	double val;
	double deltaY = A.y - B.y;
	double deltaX = A.x - B.x;
	
	val = atan2(deltaY, deltaX);
	//val = val - pow(val, 3) / 3 + pow(val, 5) / 5; // find arc tan of the slope using taylor series approximation
	if (val<0) val += 2 * PI;
	val = ((int)(val * 180 / PI)); // Convert the angle in radians to degrees
	
	return 360-val;
}

int main(int, char** argv)
{
	// Creates an output file
	//ofstream fs("times.txt");
	VideoCapture cap(0);

	Mat img, imgGRAY, imgThresholded, imgCanny;
	bool detected;
	char key = '0';
	int counter = 0;

	if (!cap.isOpened()) {
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	namedWindow("Thresholded", CV_WINDOW_AUTOSIZE);
	namedWindow("Original", CV_WINDOW_AUTOSIZE);

	cvMoveWindow("Thresholded", 0, 525);
	cvMoveWindow("Original", 0, 0);

	while (key != 'q')
	{
		int64_t e1, e2;
		double t;
		cap.read(img);

		cvtColor(img, imgGRAY, CV_BGR2GRAY);
		//GaussianBlur(imgGRAY, imgGRAY, Size(3, 3), 0);

		adaptiveThreshold(imgGRAY, imgThresholded, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 9, 9);
		//adaptiveThreshold(imgGRAY, imgThresholded, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 9, 9);

		vector< vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		vector<Rect> boundRect(contours.size());
		vector<Point2f>center(contours.size());
		vector<float>radius(contours.size());
		vector<float>curvatureFact(contours.size());
		vector<float>variabilityMaxVal(contours.size());
	
		//e1 = getTickCount();
		for (size_t i = 0; i < contours.size(); i++)
		{
			boundRect[i] = boundingRect(Mat(contours[i]));
			minEnclosingCircle((Mat)contours[i], center[i], radius[i]);
		}

		//e1 = getTickCount();
		//Curvature factor computation algorithm
		//int totalPoints = 0;
		for (size_t i = 0; i < contours.size(); i++)
		{
			float add = 0, prev_val = 0, comp, varMax = 0, diff, average;
				
			for (size_t j = 0; j < contours[i].size(); j++)
			{
				//Average of the difference between the computed radius at each point and the radius of the surrounding circle. Much more accurate
				comp = sqrt(pow((contours[i][j].x - center[i].x), 2) + pow((contours[i][j].y - center[i].y), 2));
				diff = abs(radius[i] - comp);
				if (diff > varMax)
				{
					varMax = diff;
				}
				add = add + diff;
				average = add / (j + 1); 
			}
			curvatureFact[i] = average;
			variabilityMaxVal[i] = varMax;
			//totalPoints = totalPoints + contours[i].size();
		}

		for (size_t i = 0; i < contours.size(); i++)
		{
			detected = false;

				if (curvatureFact[i] < 8 && variabilityMaxVal[i] < 10)
				{

					if (hierarchy[i][2] != -1 && curvatureFact[hierarchy[i][2]] < 8 && variabilityMaxVal[hierarchy[i][2]] < 10 && (radius[hierarchy[i][2]] / radius[i]) < 0.70 && (hierarchy[hierarchy[i][2]][0] != -1 || hierarchy[hierarchy[i][2]][1] != -1))
					{
						int aux = hierarchy[hierarchy[i][2]][2];
						int ref_contour = hierarchy[i][2];
						while (aux != -1)
						{
							if (curvatureFact[aux] < 8 && variabilityMaxVal[aux] < 10 && (radius[aux] / radius[ref_contour]) < 0.75) {
								detected = true;
								break;
							}
							aux = hierarchy[aux][2];
						}

						if (detected)
						{
							int ins_contour = hierarchy[i][2];
							if ((radius[hierarchy[i][2]] / radius[i]) < 0.5)
							{
								if (hierarchy[hierarchy[i][2]][0] != -1 && (radius[hierarchy[hierarchy[i][2]][0]] > radius[hierarchy[i][2]]))
								{
									ins_contour = hierarchy[hierarchy[i][2]][0];
								}
							}

							double comp_angle = 0;
							rectangle(img, boundRect[i].tl(), boundRect[i].br(), CV_RGB(0, 255, 0), 2, 8, 0);
							ostringstream str3, str4;
							if ((hierarchy[ins_contour][0] != -1 && curvatureFact[hierarchy[ins_contour][0]] < 8 && variabilityMaxVal[hierarchy[ins_contour][0]]< 10 && contourArea(contours[hierarchy[ins_contour][0]]) > MIN_AREA) || (hierarchy[ins_contour][1] != -1 && curvatureFact[hierarchy[ins_contour][1]] < 8 && variabilityMaxVal[hierarchy[ins_contour][1]]< 10 && contourArea(contours[hierarchy[ins_contour][1]]) > MIN_AREA))
							{
								int relative;
								if (hierarchy[ins_contour][0] != -1)
								{
									relative = 0;
								}
								else if (hierarchy[ins_contour][1] != -1)
								{
									relative = 1;
								}

								line(img, center[ins_contour], center[hierarchy[ins_contour][relative]], Scalar(255, 0, 0), 2, 8, 0);

								Point A, B;
								if (radius[hierarchy[ins_contour][relative]] > radius[ins_contour])
								{
									A = center[hierarchy[ins_contour][relative]];
									B = center[ins_contour];

								}
								else if (radius[hierarchy[ins_contour][relative]] < radius[ins_contour])
								{

									B = center[hierarchy[ins_contour][relative]];
									A = center[ins_contour];
								}

								comp_angle = angle(A, B);

								str3 << "Point A";
								str4 << "Point B";

								putText(img, str3.str(), A, FONT_HERSHEY_COMPLEX_SMALL, 0.60, CV_RGB(255, 255, 0), 1, CV_AA);
								putText(img, str4.str(), B, FONT_HERSHEY_COMPLEX_SMALL, 0.60, CV_RGB(255, 255, 0), 1, CV_AA);


							}

							ostringstream str2;
							//line(img, center[i], Point((int)img.cols / 2, (int)img.rows / 2), CV_RGB(0, 255, 255), 2, 8, 0);

							if (comp_angle == 0)
							{
								str2 << (int)(center[i].x - (int)img.cols / 2) << ", " << (int)(-(center[i].y - (int)img.rows / 2)) << "  No angle";
								//cout << "X: " << (int)(center[i].x - (int)img.cols / 2) << "   Y: " << (int)(-(center[i].y - (int)img.rows / 2)) << "   H: "<<boundRect[i].height<<"   W: "<<boundRect[i].width<<"   No angle\n";
							}
							else {
								str2 << (int)(center[i].x - (int)img.cols / 2) << ", " << (int)(-(center[i].y - (int)img.rows / 2)) << "  Angle: " << comp_angle;
								//cout << "X: " << (int)(center[i].x - (int)img.cols / 2) << "   Y: " << (int)(-(center[i].y - (int)img.rows / 2)) << "   H: " << boundRect[i].height << "   W: " << boundRect[i].width << "   Angle: " << comp_angle<<"\n";
							}
							putText(img, str2.str(), Point(boundRect[i].x, boundRect[i].y - 13), FONT_HERSHEY_COMPLEX_SMALL, 0.60, CV_RGB(0, 255, 0), 1, CV_AA);
						}
					}
				}
		}
		line(img, Point((int)img.cols / 2 - 15, (int)img.rows / 2), Point((int)img.cols / 2 + 15, (int)img.rows / 2), Scalar(180, 180, 180), 2, 8, 0);
		line(img, Point((int)img.cols / 2, (int)img.rows / 2 - 15), Point((int)img.cols / 2, (int)img.rows / 2 + 15), Scalar(180, 180, 180), 2, 8, 0);
		/*e2 = getTickCount();
		t = (e2 - e1) / getTickFrequency() * 1000;
		cout << t << " " <<totalPoints << "\n";
		fs << t << " " << totalPoints << endl;*/

		imshow("Original", img); //show the original image
		imshow("Thresholded", imgThresholded); //show the thresholded image

		key = waitKey(1);
	}
	//fs.close();
	destroyAllWindows();
	return 0;
}
