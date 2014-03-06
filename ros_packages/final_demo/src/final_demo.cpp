#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <utility>
#include <math.h>
#include <vector>

void scanCallback(const sensor_msgs::LaserScan& scan)
{

  #pragma omp parallel for
  for(unsigned int i = 0; i < scan.ranges.size(); i+=20) { // for each ray
  
    if(isnan(scan.ranges[i]) || scan.ranges[i] > scan.range_max || scan.ranges[i] < scan.range_min)
      continue;

    double t  = theta + scan.angle_min + (i*scan.angle_increment);
    double px = x + scan.ranges[i]*cos(t);
    double py = y + scan.ranges[i]*sin(t);

    

  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
 
int main(int argc, char **argv)
{

    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;
 
    ros::Subscriber scan_sub = n.subscribe("/scan", 1, scanCallback);
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, odomCallback);
    
    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    ros::spin();

    return 0;
}
