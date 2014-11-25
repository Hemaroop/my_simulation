#ifndef HEMU_271014_1432_H
#define HEMU_271014_1432_H

#include <geometry_msgs/Point.h>
#include <Stage-4.1/stage.hh>
#include <vector>

typedef struct {
  std::vector< Stg::Color > marker_color;
  std::vector< geometry_msgs::Point > marker_location;
}Warehouse_Markers;

#endif
