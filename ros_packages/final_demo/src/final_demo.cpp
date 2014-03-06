#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <utility>
#include <math.h>
#include <vector>

#define OBS_THRESHOLD 5

class SubscribeThenPublish {

public:

SubscribeThenPublish(ros::NodeHandle &n) : 
  n_(n)
{
    scan_sub = n_.subscribe("/scan", 1, &SubscribeThenPublish::scanCallback, this);
    velocity_publisher = n_.advertise<geometry_msgs::Twist>("/command", 1);
    //map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    odom_sub = n_.subscribe("/odom", 1,  &SubscribeThenPublish::odomCallback, this);
}



void odomCallback(const nav_msgs::Odometry msg){

}

void scanCallback(const sensor_msgs::LaserScan& scan)
{

  int obstructed = 0;

  //#pragma omp parallel for
  //for(unsigned int i = 0; i < scan.ranges.size(); i+=5) { // for each ray
  for(unsigned int i = 100; i < 540; i+=5) { // for each ray
  
    if(isnan(scan.ranges[i]) || scan.ranges[i] > scan.range_max || scan.ranges[i] < scan.range_min)
      continue;

    double t  = scan.angle_min + (i*scan.angle_increment);
    double px = 0 + scan.ranges[i]*cos(t);
    double py = 0 + scan.ranges[i]*sin(t);
    obstructed += (abs(py) < 0.2 && abs(px) < 1.5) ? 1:0;
    //ROS_INFO("ANGLE IS %f", t);
  }

  geometry_msgs::Twist cmd;
  if(obstructed > OBS_THRESHOLD) {
    ROS_INFO("OBSTRUCTED: %d", obstructed);
    cmd.linear.x  = 0.0;
    cmd.angular.z = 0; //0.6 * 3.14 / 180.0 * 1.0;
  }
  else {
    ROS_INFO("CLEAR: %d", obstructed);
    cmd.linear.x  = 0.4;
    cmd.angular.z = 0; //0.6 * 3.14 / 180.0 * 1.0;
  }
  velocity_publisher.publish(cmd);
}

protected:
   ros::NodeHandle n_;
   ros::Subscriber odom_sub;
   ros::Subscriber scan_sub;
   ros::Publisher velocity_publisher;

   //ros::Publisher map_publisher;

};
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
 
int main(int argc, char **argv)
{

    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle nh;
    SubscribeThenPublish stp(nh);
    
    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate
    ros::spin();
    return 0;
}
