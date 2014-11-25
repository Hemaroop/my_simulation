#include "my_simulation/marker_details.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <string.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <libgen.h>
#include <math.h>

void operator >> ( const YAML::Node& node, Warehouse_Markers& marker_lst ) {
  std::string marker_col;
  geometry_msgs::Point marker_loc;
  node["color"] >> marker_col;
  node["location"] [0] >> marker_loc.x;
  node["location"] [1] >> marker_loc.y;
  node["location"] [2] >> marker_loc.z;

  ROS_INFO( "Marker Color %s", marker_col.c_str() );
  marker_lst.marker_color.push_back( Stg::Color( marker_col ) );
  marker_lst.marker_location.push_back( marker_loc );
}

marker_details::marker_details( char* map_file )
{

  std::string image_name;
  std::ifstream ware_house( map_file );
  if ( ware_house.fail() )
    {
      ROS_ERROR( "Could not open %s.", map_file );
      exit (-1);
    }

  YAML::Parser file_parser( ware_house );
  YAML::Node doc;

  file_parser.GetNextDocument(doc);

  try {
    doc["image"] >> image_name;
  } catch ( YAML::InvalidScalar ) {
    ROS_ERROR("Image key not found in the document.");
    exit (-1);
  }

  try {
    doc["resolution"] >> map_resolution;
  } catch ( YAML::InvalidScalar ) {
    ROS_ERROR("Resolution key not available in the document.");
    exit (-1);
  }

  try {
    doc["origin"] [0] >> map_origin.x;
    doc["origin"] [1] >> map_origin.y;
    doc["origin"] [2] >> map_origin.z;
  } catch ( YAML::InvalidScalar ) {
    ROS_ERROR("Map Origin co-ordinates not available");
    exit (-1);
  }

  try  {
    const YAML::Node& markers = doc["markers"];
    for ( unsigned ii = 0; ii < markers.size(); ii++ )
      {
	markers[ii] >> marker_list;
      }
  } catch ( YAML::InvalidScalar ) {
    ROS_ERROR( "Markers not found in the map file." );
    exit (-1);
  }

  try {
    doc["laser_range"] >> laser_range;
  } catch ( YAML::InvalidScalar ) {
    ROS_WARN( "Laser range not specified. Using default value." );
    laser_range = LASER_DEFAULT_RANGE;
  }

  try {
    doc["blobfinder_range"] >>blbfndr_range;
  } catch ( YAML::InvalidScalar ) {
    ROS_WARN( "Blobfinder range not specified. Using default value." );
    blbfndr_range = BLOBFINDER_DEFAULT_RANGE;
  }

  x_dim = (fabs( map_origin.x ))/map_resolution;
  y_dim = (fabs( map_origin.y ))/map_resolution;
  ROS_INFO("%d %d %F", x_dim, y_dim, map_resolution);
  ROS_INFO("%F %F %F", map_origin.x, map_origin.y, map_origin.z);
  
  if ( image_name[0] != '/' )
    {
      std::string str1(map_file);
      char* ch_arr = strdup( str1.c_str() );
      ch_arr = dirname(ch_arr);
      ROS_INFO("Map directory : %s", ch_arr);
      std::string sep_str = "/"; 
      ch_arr = strcat( ch_arr, sep_str.c_str() );
      ch_arr = strcat( ch_arr, image_name.c_str() );
      image_name = std::string(ch_arr);
      ROS_INFO("Map image : %s", image_name.c_str());
      map_image = strdup( image_name.c_str() );
      delete[] ch_arr;
    }
  else 
    map_image = strdup( image_name.c_str() );

  ROS_INFO("Map image : %s", map_image);
  ware_house.close();
}

marker_details::~marker_details()
{
  delete[] map_image;
}
