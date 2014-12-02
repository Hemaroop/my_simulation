#ifndef HEMU_30102014_1221_H
#define HEMU_30102014_1221_H

#include "warehouse_markers.h"
#include <geometry_msgs/Point.h>
#define LASER_DEFAULT_RANGE 6.0
#define BLOBFINDER_DEFAULT_RANGE 5.0

class marker_details {
 public:
  marker_details( char* map_file );
  ~marker_details();

  Warehouse_Markers marker_list;
  char* map_image;
  double map_resolution;
  geometry_msgs::Point map_origin;
  unsigned int x_dim;
  unsigned int y_dim;
  float laser_range;
  float blbfndr_range;
};

#endif
