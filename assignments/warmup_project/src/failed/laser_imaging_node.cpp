// Written by Eric Schneider for the CompRobo warmup project
// 9/14/2014

// Stuff about how it works...
///////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include "laser_imaging.h"
using namespace laser_imaging;

int main(int argc, char **argv)
{
	int edge_length = 500;
	int point_radius = 10;
	int pix_per_m = 100;
	// int num_points = 360;
	// vector<int> distances;
	// distances.assign(num_points, 1.1);

	LaserImaging img(edge_length, point_radius, pix_per_m);
	// laser_imaging::LaserImaging *img =
	// new laser_imaging::LaserImaging(edge_length, point_radius, pix_per_m);
}