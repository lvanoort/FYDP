#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <utility>
#include <math.h>
#include <vector>

class SubscribeThenPublish {

public:

SubscribeThenPublish(ros::NodeHandle &n) : 
  n_(n)
{
    scan_sub = n_.subscribe("/scan", 1, &SubscribeThePublish::scanCallback, this);
    velocity_publisher = n_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    //map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    odom_sub = n_.subscribe("/odom", 1,  &SubscribeThePublish::odomCallback, this);
}



void odomCallback(const nav_msgs::Odometry msg){

}

void scanCallback(const sensor_msgs::LaserScan& scan)
{

  #pragma omp parallel for
  bool obstructed = false;
  for(unsigned int i = 0; i < scan.ranges.size(); i+=20) { // for each ray
  
    if(isnan(scan.ranges[i]) || scan.ranges[i] > scan.range_max || scan.ranges[i] < scan.range_min)
      continue;

    double t  = scan.angle_min + (i*scan.angle_increment);
    double px = x + scan.ranges[i]*cos(t);
    double py = y + scan.ranges[i]*sin(t);
    obstructed = (abs(px) < 0.5 && abs(py) < 2.0);
    

  }
  
  geometry_msgs::Twist cmd;
  if(obstructed) {
    cmd.linear.x  = 0.0;
    cmd.angular.z = 0.6 * 3.14 / 180.0 * 1.0;
  }
  ROS_INFO("clear");
  velocity_publisher.publish(cmd);
}

protected:
   ros::NodeHandle n_;
   ros::Subscriber odom_sub;
   ros::Subscriber scan_sub;
   ros::Publisher velocity_publisher;

   //ros::Publisher map_publisher;

}
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
 
int main(int argc, char **argv)
{

    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;
    SubscribeThenPublish(n);
    
    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate
    ros::spin();
    return 0;
}
