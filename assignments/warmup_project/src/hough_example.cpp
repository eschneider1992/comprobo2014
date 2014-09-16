// Written by Eric Schneider for the CompRobo warmup project
// 9/14/2014

// Stuff about how it works...

// In the code below I couldn't get the input_keys methods to apprpriately
// pass a class object. The sm_robot class was either not passed into the
// states, or it was passed as a constant. I left the code in there for
// documentation and in case I could get it working later

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>  

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

void help()
{
 cout << "\nThis program demonstrates line finding with the Hough transform.\n"
 "Usage:\n"
 "./houghlines <image_name>, Default is pic1.jpeg\n" << endl;
}

int main(int argc, char** argv)
{
    sensor_msgs::LaserScan scan;
    scan.ranges.resize(10);
    for (int i=0; i++; i<10)
        scan.ranges[i] = 5.1 * i;

    const char* filename = argc >= 2 ? argv[1] : "pic1.jpeg";

    Mat src = imread(filename, 0);
    if(src.empty())
    {
     help();
     cout << "can not open " << filename << endl;
     return -1;
 }

 Mat dst, cdst;
 Canny(src, dst, 50, 200, 3);
 cvtColor(dst, cdst, CV_GRAY2BGR);
 
 int width = 500;
 int width2 = width*width;
 uint8_t data[width2];
 for (int i = 0; i<width2; i++)
 {
    data[i] = i;
 }

ROS_WARN("Got here!");

 // Mat test(100,100, CV_8UC3, Scalar(0,0,0));
 Mat test(width,width, CV_8UC1, &data);
 // test[0] = 255; test[1] = 255; test[2] = 255;
 // test[3] = 255; test[6] = 255; test[9] = 255;

 #if 0
  vector<Vec2f> lines;
  HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

  for( size_t i = 0; i < lines.size(); i++ )
  {
     float rho = lines[i][0], theta = lines[i][1];
     Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 10*(-b));
     pt1.y = cvRound(y0 + 10*(a));
     pt2.x = cvRound(x0 - 10*(-b));
     pt2.y = cvRound(y0 - 10*(a));
     line( cdst, pt1, pt2, Scalar(0,0,255), 1, CV_AA);
  }
 #else
  vector<Vec4i> lines;
  HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
  // ROS_INFO("Lines.size: %d", lines.size());
  for( size_t i = 0; i < lines.size(); i++ )
  {
    // ROS_INFO("i: %d", i);
    Vec4i l = lines[i];
    line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, CV_AA);
}
 #endif
imshow("source", src);
imshow("test", test);
imshow("detected lines", cdst);

waitKey();

return 0;
}