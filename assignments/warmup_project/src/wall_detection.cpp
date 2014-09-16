// Written by Eric Schneider for the CompRobo warmup project
// 9/14/2014

// This node
//    1) Takes laser data and creates a binary image with it
//    2) Does the hough transform on the image to find lines
//    3) Analyzes those lines with reference to the center point to find
//       a Twist vector representing the wall
//    4) wall_publishing that vector

// Twist.linear is the distance to the nearest point on the infinite wall line
//   - Twist.linear.x is positive in front of bot, negative behind
//   - Twist.linear.y is positive to left, negative to right (right hand rule)
// Twist.anguler.z is radians the robot needs to turn in order to align w/ wall
//   - Example: if the wall is in front of the robot and slopes towards the bot
//   - as it goes to the right, angular.z would be ~PI/4 (positive) b/c turning
//   - PI/4 would bring the bot into alignment with the wall

// Areas of improvement
//    1) Be smarter about detecting overlapping segments as a single wall
//    2) wall_publish multiple wall segments at a time, perhaps as a Twist vector
//    3) Delineate where the wall starts/stops

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <math.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

ros::Publisher wall_pub;
const int edge_len = 500;
const int point_radius = 1;
const int pix_per_m = 100;
const int num_points = 360;
const float max_dist = edge_len / (pix_per_m * 2.1);
const int cen_x = edge_len/2.0;
const int cen_y = edge_len/2.0;

void laser_to_image(const sensor_msgs::LaserScanConstPtr& msg);
bool detect_walls(Mat blk);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wall_detection");
  ros::NodeHandle n;
  wall_pub = n.advertise<geometry_msgs::Twist>("/detected_walls", 10);
  ros::Subscriber sub = n.subscribe("/scan", 1, laser_to_image);
  ros::spin();
  return 0;
}

void laser_to_image(const sensor_msgs::LaserScanConstPtr& msg)
{
  ROS_INFO("Starting laser to image conversion");
  uint8_t data[edge_len * edge_len];
  for (int i = 0; i < (edge_len * edge_len); i++)
    data[i] = 0.0;

  float ang;  float dist;
  int x_pix;  int y_pix;

  for (int i=0; i<num_points; i++)
  {
    // The angle offset is to account for North = 0
    ang = (i * CV_PI / 180) + (CV_PI / 2);
    dist = msg->ranges[i];
    if (dist > 0.01 && dist < max_dist)
    {
      x_pix = (cos(ang) * dist * pix_per_m) + cen_x;
      y_pix = (sin(ang) * dist * pix_per_m) + cen_y;
      if (x_pix + point_radius > edge_len || x_pix - point_radius < 0
          || y_pix + point_radius > edge_len || y_pix - point_radius < 0)
        ROS_INFO("Point dropped, it was on an edge");
      else
      {
        for (int x = -point_radius; x <= point_radius; x++)
          for (int y = -point_radius; y <= point_radius; y++)
            data[((edge_len - (y_pix + y)) * edge_len + (x_pix + x))] = 255;
      }
    }
  }
  
  Mat blk(edge_len, edge_len, CV_8UC1, &data);
  // imshow("Black and white", blk);
  // waitKey(25);

  detect_walls(blk);
}

bool detect_walls(Mat blk)
{
  // Create a new image and copy the binary image onto it
  Mat clr;
  cvtColor(blk, clr, CV_GRAY2BGR);
  
  // Draw a robot representation
  int k = edge_len / 50;
  line(clr, Point(cen_x, cen_y - k), Point(cen_x + k, cen_y + k),
        Scalar(0, 255, 0), 4, CV_AA);
  line(clr, Point(cen_x, cen_y - k), Point(cen_x - k, cen_y + k),
        Scalar(0, 255, 0), 4, CV_AA);

  // Detect lines in the image
  vector<Vec4i> lines;
  // src, lines, res (pix), res (rad), points_in_line, minLinLength, maxLineGap
  HoughLinesP(blk, lines, 1, CV_PI/180, 50, 100, 80 );
  ROS_INFO("\tNumber of lines detected: %d", lines.size());
  
  vector<float> ang, length, nearest_dist;
  ang.assign(lines.size(), 0.0);
  length.assign(lines.size(), 0.0);
  nearest_dist.assign(lines.size(), 0.0);
  float max_len = 0.0;
  int max_i = 0;

  for( size_t i = 0; i < lines.size(); i++ )
  {
    // Plot wall lines on image
    // format is (x1, y1, x2, y2)
    Vec4i l = lines[i];
    line(clr, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,217,0), 2, CV_AA);
    
    // Find orientation of wall
    // passed in as atan2(-y2 - (-y1), x2 - x1) because the y's are flipped
    ang.at(i) = atan2(-l[3] + l[1], l[2] - l[0]);
    
    // Find length of wall
    length.at(i) = pow(pow(l[2] - l[0], 2) + pow(l[3] - l[1], 2), 0.5) / pix_per_m;
    
    // Find the shortest distance from robot to wall
    float m = (-l[3] + l[1]) / ((float) l[2] - l[0]);
    // y-intercept is b = y - mx (again, negated b/c y is flipped in images)
    float by = -l[1] - m*l[0];
    // equation found here:
    //      math.ucsd.edu/~wgarner/math4c/derivations/distance/distptline.htm
    // This assumes the bot is in the center of the picture
    nearest_dist.at(i) = abs(-cen_y - m * cen_x - by)
                          / ((float) (pow(pow(m, 2) + 1, 0.5)
                          * pix_per_m));
    
    // Plot debug messages
    // ROS_INFO("\tline %d - angle: %f", i, ang.at(i));
    // ROS_INFO("\tline %d - length (m): %f", i, length.at(i));
    // ROS_INFO("\tline %d - slope: %f", i, m);
    // ROS_INFO("\tline %d - y intercept: %f", i, by);
    // ROS_INFO("\tline %d - nearest_dist (m): %f", i, nearest_dist.at(i));

    if (length.at(i) > max_len)
    {
      max_len = length.at(i);
      max_i = i;
    }
  }

  imshow("Added lines", clr);
  waitKey(10);

  if (lines.size() > 0)
  {
    // TODO: Right now this only publishes the longest wall, could be smarter
    geometry_msgs::Twist wall;
    wall.linear.x = cos(ang.at(max_i)) * nearest_dist.at(max_i);
    wall.linear.y = sin(ang.at(max_i)) * nearest_dist.at(max_i);
    if (ang.at(max_i) < 0.0)
      wall.angular.z = ang.at(max_i) + CV_PI / 2.0;
    else
      wall.angular.z = ang.at(max_i) - CV_PI / 2.0;
    wall_pub.publish(wall);
    return true;
  }
  else
    return false;
}