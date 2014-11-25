#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <fuzzy_oa/robo_vel.h>
#include <fuzzy_oa/my_vfh.h>

using namespace std;

ros::Publisher vel_pub;
geometry_msgs::Twist vel_obj;

MY_VFH laser_vfh;
int n, n_highres_sec, n_medres_sec, n_lowres_sec; // Count of laser scan points
double COSTMAP_OBST_RAD=0.40, calculated_costmap;
double linSpeed, rotSpeed;
int op_mode = 0;
vel_control wh_bot1( (char*) "BOT_CONTROL" );

void laser_to_vfh( const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  n = scan_msg->ranges.size();
  n_highres_sec = n/HIGHRES_SEC;
  n_medres_sec = HIGHRES_SEC/MEDRES_SEC;
  n_lowres_sec = n/LOWRES_SEC;
  //High resolution VFH
  for ( int ii = 0; ii < HIGHRES_SEC; ii++ )
    {
      double added_value = 0.0;
      for ( int jj= 0; jj < n_highres_sec; jj++ )
	{
	  added_value+=scan_msg->ranges[(ii*n_highres_sec)+jj];
	}
      laser_vfh.highres_vfh.push_back((added_value/n_highres_sec) - calculated_costmap);
    }
  
  //Medium resolution VFH
  for ( int ii = 0; ii < MEDRES_SEC; ii++ )
    {
      double added_value=0.0;
      for ( int jj= 0; jj < n_medres_sec; jj++ )
        {
	  added_value+=laser_vfh.highres_vfh[(ii*n_medres_sec)+jj];
        }
      laser_vfh.medres_vfh.push_back((added_value/n_medres_sec) - calculated_costmap);
    }

  //Low Resolution VFH
  for ( int ii = 0; ii < LOWRES_SEC; ii++ )
    {
      double added_value=0.0;
      for ( int jj= 0; jj < n_lowres_sec; jj++ )
        {
	  added_value+=scan_msg->ranges[(ii*n_lowres_sec)+jj];
        }
      laser_vfh.lowres_vfh.push_back((added_value/n_lowres_sec) - calculated_costmap);
    }

  wh_bot1.vel_ctrl_fis( laser_vfh, &linSpeed, &rotSpeed, op_mode);

  //ROS_INFO("LinSpeed %f \n RotSpeed %f", linSpeed, rotSpeed);

  vel_obj.linear.x = linSpeed;
  vel_obj.angular.z = rotSpeed;
  //ROS_INFO("CMD_Vel %f %f", vel_obj.linear.x, vel_obj.angular.z);

  vel_pub.publish(vel_obj);
}

int main( int argc, char** argv )
{

  ros::init(argc,argv,"player_obst_avoid");
  ros::NodeHandle n;
  
  for ( int argno = 0; argno < argc; argno++ )
    {
      if ( ! strcmp( argv[argno], (char*) "mapping" ) )
	{
	  ROS_INFO("Initiating mapping mode");
	  op_mode = 1;
	}
    }

  ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 10, laser_to_vfh);
  vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  calculated_costmap = (0.40*COSTMAP_OBST_RAD/2)+(0.70*COSTMAP_OBST_RAD);

  ros::spin();  
  
  return 0;
}
