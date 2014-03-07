#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <hark_msgs/HarkSource.h>
#include <utility>
#include <math.h>
#include <vector>

#define OBS_THRESHOLD 5

enum RobotModes {
  INITIAL_SEARCH = 0,
  DRIVING        = 1,
  DODGING_TURN   = 2,
  DODGING_GO     = 3,
  TRACKING       = 4,
};

class SubscribeThenPublish {

public:

SubscribeThenPublish(ros::NodeHandle &n) : 
  n_(n)
{
    scan_sub = n_.subscribe("/scan", 1, &SubscribeThenPublish::scanCallback, this);
    velocity_publisher = n_.advertise<geometry_msgs::Twist>("/command", 1);
    //map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    odom_sub = n_.subscribe("/robo_odo/odometry", 1,  &SubscribeThenPublish::odomCallback, this);
    hark_sub = n_.subscribe("/HarkSource", 1, &SubscribeThenPublish::harkCallback, this);
    state = DRIVING;
}

void odomCallback(const nav_msgs::Odometry msg) 
{
  double turn_rate = -msg.twist.twist.angular.y;
  double forward_rate = msg.twist.twist.linear.z;
  
  static double turn = 0;
  static double forward = 0;

  static bool first_message = true;
  static ros::Time last_time = ros::Time::now();

  if(first_message) {
    first_message = false;
    last_time = ros::Time::now();
    return;
  }

  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  last_time = current_time;

   geometry_msgs::Twist cmd;


   switch (state) {
      case DODGING_TURN:
         turn += turn_rate*dt;
         cmd.angular.z = -2.0;
         if(turn < -3.1415/6) {
           ROS_INFO("Moving to: DODGING_GO");
           state = DODGING_GO;
         }
         
         velocity_publisher.publish(cmd);
         break;
      case DODGING_GO:
         forward += forward_rate*dt;
         cmd.linear.x  = 0.7;
         if(forward > 1.8) {
           ROS_INFO("Moving to: TRACKING");
           state = TRACKING;
            cmd.linear.x = 0.0;
         }

         velocity_publisher.publish(cmd);
         break;
      default:
         break;
   }


}

void harkCallback(const hark_msgs::HarkSourceConstPtr& msg)
{
  //ROS_INFO("Received Num of Src [%d]", msg->exist_src_num);
  if(msg->exist_src_num < 1) {
    //pub_.publish(geometry_msgs::Twist());    
  } else {
    int minid = 10000;
    int minidelem = 0;
    //geometry_msgs::Twist cmd;
    for(int num = 0; num < msg->exist_src_num; num++) {
      if(msg->src[num].id > minid ) 
        continue;
      currentTarget = msg->src[num].theta;
    }
    //ROS_INFO("Received [ID,Azimuth] [%d,%f]", msg->src[minidelem].id, msg->src[minidelem].theta);
   // pub_.publish(cmd);
  }

   switch (state) {
      case INITIAL_SEARCH:
         track();
         if( currentTarget < 5.0*3.1415/180 ) {
           state = DRIVING;
           ROS_INFO("Moving to: DRIVING");
         }
         break;
      case TRACKING:
         track();
         break;
      default:
         break;
   }

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
    obstructed += (abs(py) < 0.2 && abs(px) < 0.75) ? 1:0;
  }

  bool currentlyObstructed = (obstructed > OBS_THRESHOLD);

   geometry_msgs::Twist cmd;
   switch (state) {
      case DRIVING:
         cmd.linear.x  = 0.7;
         cmd.angular.z = 0;
         velocity_publisher.publish(cmd);
         if (currentlyObstructed) {
           ROS_INFO("Moving to: DODGING_TURN");
           state = DODGING_TURN;
         }
         break;
      default:
         break;
   }
}

void search() 
{
  geometry_msgs::Twist cmd;
  velocity_publisher.publish(cmd);
}

void track() 
{
  geometry_msgs::Twist cmd;
  cmd.linear.x  = 0.1;
  cmd.angular.z = currentTarget * 3.14 / 180.0 * 1.0;
  velocity_publisher.publish(cmd);
}

protected:
   ros::NodeHandle n_;
   ros::Subscriber odom_sub;
   ros::Subscriber scan_sub;
   ros::Subscriber hark_sub;
   ros::Publisher velocity_publisher;
   double currentTarget;
   RobotModes state;

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
