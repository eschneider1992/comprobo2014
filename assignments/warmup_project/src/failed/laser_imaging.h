// Written by Eric Schneider for the CompRobo warmup project
// 9/14/2014

// Stuff about how it works...

/////////////////////////////////////////////////////////////////////////////

#ifndef LASER_IMAGING_H
#define LASER_IMAGING_H

using namespace std;

/////////////////////////////////////////////////////////////////////////////

namespace laser_imaging
{
  class LaserImaging
  {
  public:
  LaserImaging(const int edge_length, const int point_radius,
             const int pix_per_m);
             // , const int num_points, vector<int> angles,
             // vector<float> distances);
  ~LaserImaging();
  bool start_publishing();
  bool stop_publishing();

  private:
    int edge_length;
    int point_radius;
    int pix_per_m;
    int num_points;
    // vector<int> angles;
    // vector<float> distances;
  };
}

#endif
