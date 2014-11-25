#include <ros/ros.h>
#include <ros/console.h>
#include "my_simulation/marker_details.h"
//#include "my_simulation/temp_map.h"
#include <math.h>
#include <time.h>
#include <string.h>
#include <my_stage/Color.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

//std::string TEMP_MAP = "/tmp/marker_locality.png";
bool using_temp_map = false;
//unsigned int isolated_reg_max_dim;
//float iso_dim;
vector<Stg::Color> color_list;
vector<geometry_msgs::Point> point_list;
//my_simulation::temp_map temp_topic;
geometry_msgs::PoseWithCovarianceStamped possible_loc;
ros::Publisher initial_pose_pub_; //temp_map_pub_;
cv::Mat my_map;
//cv::Point temp_org;
geometry_msgs::Point warehouse_org;
double warehouse_res;
unsigned int x_dim, y_dim;

/*void convertTopixels( geometry_msgs::Point& mp_coord, unsigned int x_org, unsigned int y_org, double& m_res )
{
  temp_org.x = (int) (mp_coord.x / m_res);
  temp_org.y = (int) ((-1*mp_coord.y) / m_res);
  temp_org.x += x_org;
  temp_org.y += y_org;
}*/

void blob_detected( const my_stage::Color::ConstPtr& colorMsg )
{
  Stg::Color blob_color ( colorMsg->r, colorMsg->g, colorMsg->b, colorMsg->a );
  if ( colorMsg->blob_area > (uint32_t) 7000 ) //&& !using_temp_map )
    {
      for ( unsigned int ii = 0; ii < color_list.size(); ii++ )
	{
	  if ( color_list[ii] == blob_color )
	    {
	      possible_loc.header.stamp = ros::Time::now();
	      possible_loc.pose.pose.position.x = point_list[ii].x;
	      possible_loc.pose.pose.position.y = point_list[ii].y;
	      possible_loc.pose.pose.position.z = point_list[ii].z;
	      /*temp_topic.temp_origin.x = point_list[ii].x - iso_dim;
	      temp_topic.temp_origin.y = point_list[ii].y - iso_dim;
	      temp_topic.temp_origin.z = point_list[ii].z;
	      convertTopixels( temp_topic.temp_origin, x_dim, y_dim, warehouse_res );*/
	      double yaw = 0.0;
	      possible_loc.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( 0.0, 0.0, yaw );
	      initial_pose_pub_.publish( possible_loc );    
	    }
	}
      /*int strtrow = temp_org.x - isolated_reg_max_dim;
      int endrow = temp_org.x + isolated_reg_max_dim;
      int strtcol = temp_org.y - isolated_reg_max_dim;
      int endcol = temp_org.y + isolated_reg_max_dim;

      ROS_INFO( "Selected Region : [ %d %d ] [ %d %d ]", strtrow, endrow, strtcol, endcol );

      if ( strtrow < 0 )
	strtrow = 0;
      if ( strtcol < 0 )
	strtcol = 0;
      if ( endrow > my_map.cols )
	endrow = my_map.cols;
      if ( endcol > my_map.rows )
	endcol = my_map.rows;

      ROS_INFO( "Selected Region : [ %d %d ] [ %d %d ]", strtrow, endrow, strtcol, endcol );

      cv::Range r_range( strtrow, endrow );
      cv::Range c_range( strtcol, endcol ); 

      cv::Mat temp_map( my_map, c_range, r_range ); 

      if ( cv::imwrite( TEMP_MAP, temp_map ) )
	ROS_INFO("Temporary map saved.");     

      temp_topic.use_temp_map = true;
      temp_topic.map_name = TEMP_MAP;
      temp_map_pub_.publish( temp_topic );
      using_temp_map = true;*/
    }
}

void localized_pose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_msg )
{

}

void usage()
{
  ROS_INFO("USAGE: rosrun warehouse_marker_finder --map_file [PATH TO YAML MAP FILE]");
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "warehouse_marker_finder");

  char* map_file = NULL;
  ros::NodeHandle n;
  possible_loc.header.frame_id = "map";

  if ( argc < 3 )
    {
      usage();
      return -1;
    }

  for ( int jj = 0; jj < argc; jj++ )
    {
      if ( ! strcmp( argv[jj], (char*) "--map_file") )
        {
	  std::string my_str( argv[++jj] );
          map_file = strdup( my_str.c_str() );
        }
    }

  marker_details tmp_Mrkr_Lst( map_file );
  color_list = tmp_Mrkr_Lst.marker_list.marker_color;
  point_list = tmp_Mrkr_Lst.marker_list.marker_location;
  /*iso_dim = tmp_Mrkr_Lst.laser_range + tmp_Mrkr_Lst.blbfndr_range;
  isolated_reg_max_dim = (unsigned int) (iso_dim)/tmp_Mrkr_Lst.map_resolution;*/
  warehouse_res = tmp_Mrkr_Lst.map_resolution;
  x_dim =  tmp_Mrkr_Lst.x_dim;
  y_dim = tmp_Mrkr_Lst.y_dim;

  ROS_INFO( "Reading map image : %s", tmp_Mrkr_Lst.map_image);
  my_map = cv::imread( std::string( tmp_Mrkr_Lst.map_image ), 0 );

  ROS_INFO ( "Rows: %d \n Cols: %d", my_map.rows, my_map.cols );

  delete[] map_file;

  if ( ! my_map.data )
    {
      ROS_INFO( "Map does not exist." );
      return -1;
    }

  my_map.convertTo( my_map, CV_32FC1, 1.0/255.0, 0.0 );
  threshold( my_map, my_map, 0.5, 1, cv::THRESH_BINARY );
  my_map.convertTo( my_map, CV_8UC1, 255, 0.0 );

  initial_pose_pub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>( "initialpose", 10 );
  //temp_map_pub_ = n.advertise<my_simulation::temp_map>("tmp_map_specs", 10 );
  
  ros::Subscriber blob_sub_ = n.subscribe( "color_blob", 10, blob_detected );
  ros::Subscriber amcl_pose_sub_ = n.subscribe( "amcl_pose", 10, localized_pose );

  ros::MultiThreadedSpinner my_spinner(4);
  my_spinner.spin();  

  return 0;
}
