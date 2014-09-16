// Written by Eric Schneider for the CompRobo warmup project
// 9/14/2014

// Stuff about how it works...
///////////////////////////////////////////////////////////////////////////////

#include "laser_imaging.h"
#include "ros/ros.h"
#include <ros/console.h>
using namespace laser_imaging;

///////////////////////////////////////////////////////////////////////////////

LaserImaging::LaserImaging(const int edge, const int point, const int ppm)
                        // vector<float> data):
                        // vector<int> indices,
{
  ROS_INFO("LaserImaging constructor");
  edge_length = edge;
  point_radius = point;
  pix_per_m = ppm;
  // num_points = vector_size;
  // angles = indices;
  // distances = data;
  ROS_INFO("Data acquired");
}

LaserImaging::~LaserImaging()
{
}

bool LaserImaging::start_publishing()
{
    return true;
}

bool LaserImaging::stop_publishing()
{
    return true;
}

// void ReflexHand::tx(const uint8_t *msg, 
//                     const uint16_t msg_len, 
//                     const uint16_t port)
// {
//   ROS_INFO("ReflexHand::tx %d bytes to port %d", msg_len, port);
//   mcast_addr_.sin_port = htons(port);
//   int nsent = sendto(tx_sock_, msg, msg_len, 0,
//                      (sockaddr *)&mcast_addr_, sizeof(mcast_addr_));
//   ROS_ERROR_COND(nsent < 0, "woah. sendto() returned %d", nsent);
// }