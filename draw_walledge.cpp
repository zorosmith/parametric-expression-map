#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>

using namespace std;

struct AreaCmp {
  AreaCmp(const vector<float>& _areas) : areas(&_areas) {}
  bool operator()(int a, int b) const { return (*areas)[a] > (*areas)[b]; }
  const vector<float>* areas;
};

int main(int argc, char* argv[]) {

  cv::Mat img, img_edge, img_color;

  // load image or show help if no image was provided
  //
  if( argc != 2 || (img = cv::imread(argv[1],cv::IMREAD_GRAYSCALE)).empty() ) {
	  cout << "\nERROR: You need 2 parameters, you had " << argc << "\n" << endl;
    cout << "\nExample 14_2: Drawing Contours\nCall is:\n" << argv[0] << " <image>\n\n"
         << "Example:\n" << argv[0] << " ../box.png\n" << endl;
    return -1;
  }

  cv::threshold(img, img_edge, 210, 255, cv::THRESH_BINARY); //THRESH_BINARY：过门限的值设置为maxVal，其它值置零
  cv::imshow("Image after threshold", img_edge);
  vector< vector< cv::Point > > contours;

  vector< cv::Vec4i > hierarchy;

  cv::findContours(
    img_edge,
    contours,
    hierarchy,
    cv::RETR_LIST,
    cv::CHAIN_APPROX_SIMPLE
  );
  cout << "\n\nHit any key to draw the next contour, ESC to quit\n\n";
  cout << "Total Contours Detected: " << contours.size() << endl;

  vector<int> sortIdx(contours.size());
  vector<float> areas(contours.size());

  // containers for simplified contour
  vector< vector<cv::Point> > approx_contours(contours.size());
  vector<float> approx_areas(contours.size());

  for( int n = 0; n < (int)contours.size(); n++ ) {
    sortIdx[n] = n;
    areas[n] = contourArea(contours[n], false);
  }

  // sort contours so that the largest contours go first
  //
  std::sort( sortIdx.begin(), sortIdx.end(), AreaCmp(areas ));

  for( int n = 0; n < (int)sortIdx.size(); n++ )
  {
    int idx = sortIdx[n];
    cv::cvtColor( img, img_color, cv::COLOR_GRAY2BGR );
    cv::approxPolyDP(contours[idx], approx_contours[n], 10, true);
    approx_areas[n] = contourArea(approx_contours[n], false);    

    cv::drawContours(
      img_color, contours, idx,
      cv::Scalar(0,100,255), 2, 8, hierarchy,
      1 // Try different values of max_level, and see what happens
    );
    cv::drawContours(
      img_color, approx_contours, n,
      cv::Scalar(255, 0, 0), 2, 8, hierarchy,
      1 // Try different values of max_level, and see what happens
    );
    cout << "Contour #" << idx << ": area=" << areas[idx] <<
      ", nvertices=" << contours[idx].size() << endl;
    cout << "After approxing " << endl;
    cout << "approx_areas = " << approx_areas[n] << ", nvertices = " <<
      approx_contours[n].size() << endl;

    cout << "vertices of plane # " << n << endl;
    for(int i=0; i < approx_contours[n].size(); i++)
    {
        cout << "point " << i << " : " << approx_contours[n][i] << endl;
    }
    
    cv::imshow(argv[0], img_color);
    int k;
    if( (k = cv::waitKey()&255) == 27 )
      break;
  }
  cout << "Finished all contours\n";


  return 0;
}
