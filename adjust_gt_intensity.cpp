/* ---------------------------------
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include "basefunction.h"
#include <pcl/features/normal_3d.h>
#include <fstream>
-----------------------------------*/
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

Mat g_gray, g_binary;
int g_thresh = 100;

void on_trackbar(int, void*)
{
    threshold(g_gray, g_binary, g_thresh, 255, THRESH_BINARY);
    vector< vector<Point> > contours;
    findContours(g_binary, contours, noArray(), RETR_TREE, CHAIN_APPROX_SIMPLE);
    g_binary = Scalar::all(0);
    drawContours( g_binary, contours, -1, Scalar::all(150), 1, CV_AA, noArray(), 1 );
    imshow( "Contours", g_binary );

}


int main(int argc, char** argv)
{
    if( argc != 2 || (g_gray = imread(argv[1], 0)).empty() )
    {
        cout << "Find threshold dependent contours\nUsage : " << argv[0] << "fruits.png" << endl;
        return -1;
    }
    namedWindow( "Contours", 1 );

    createTrackbar("Threshold", "Contours", &g_thresh, 255, on_trackbar);
    on_trackbar(0, 0);
    waitKey();
    
    return 0;
}
