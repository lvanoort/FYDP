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

//states
#define SEARCHING 0;
#define TRACKING 1;
#define DODGING2;

class SubscribeThenPublish {

public:

SubscribeThenPublish(ros::NodeHandle &n) : 
  n_(n)
{
    scan_sub = n_.subscribe("/scan", 1, &SubscribeThenPublish::scanCallback, this);
    velocity_publisher = n_.advertise<geometry_msgs::Twist>("/command", 1);
    //map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    odom_sub = n_.subscribe("/odom", 1,  &SubscribeThePublish::odomCallback, this);
    hark_sub = n_.subscribe("HarkSource", 1, &SubscribeThenPublish::subCallback, this);
    state = SEARCHING;
}



void odomCallback(const nav_msgs::Odometry msg){

}

void harkCallback(const hark_msgs::HarkSourceConstPtr& msg)
{
  ROS_INFO("Received Num of Src [%d]", msg->exist_src_num);
  currentlyTracking = false;
  if(msg->exist_src_num < 1) {
    //pub_.publish(geometry_msgs::Twist());    
  } else {
    int minid = 10000;
    int minidelem = 0;
    //geometry_msgs::Twist cmd;
    for(int num = 0; num < msg->exist_src_num; num++) {
      if(msg->src[num].id > minid ) continue;
      //minidelem = num;
      //minid = msg->src[num].id;
      //cmd.linear.x  = 0.1;
      //cmd.angular.z = msg->src[num].theta * 3.14 / 180.0 * 1.0;
      currentTarget = msg->src[num].theta
      currentlyTracking = true;
    }
    ROS_INFO("Received [ID,Azimuth] [%d,%f]", msg->src[minidelem].id, msg->src[minidelem].theta);
   // pub_.publish(cmd);
  }
  
  updateState();
  doStuff();
}

void scanCallback(const sensor_msgs::LaserScan& scan)
{

  int obstructed = 0;

  //#pragma omp parallel for
  //for(unsigned int i = 0; i < scan.ranges.size(); i+=5) { // for each ray
  for(unsigned int i = 100; i < 540; i+=5) { // for each ray
  #pragma omp parallel for
  for(unsigned int i = 0; i < scan.ranges.size(); i+=20) { // for each ray
  
    if(isnan(scan.ranges[i]) || scan.ranges[i] > scan.range_max || scan.ranges[i] < scan.range_min)
      continue;

    double t  = scan.angle_min + (i*scan.angle_increment);
    double px = 0 + scan.ranges[i]*cos(t);
    double py = 0 + scan.ranges[i]*sin(t);
    obstructed += (abs(py) < 0.2 && abs(px) < 1.5) ? 1:0;
    //ROS_INFO("ANGLE IS %f", t);
  }

  geometry_msgs::Twist cmd;
  currentlyObstructed = (obstructed > OBS_THRESHOLD);
  
  updateState();
  doStuff();
}

void updateState() {
   switch (state) {
      case SEARCHING:
         if(currentlyTracking && currentlyObstructed) state = 2;
         else if(currentlyTracking) state = 1;
         break;
      case TRACKING:
         if(!currentlyTracking) state = 0;
         else if(currentlyObstructed) state = 2;
         break;
      case DODGING:
         if(doneDodge && currentlyTracking) state = 1;
         else if (doneDodge) state = 0;
         break;
   }
}

void doStuff() {
   switch (state){
      case SEARCHING:
         search();
         break;
      case TRACKING:
         track();
         break;
      case DODGING:
         dodge();
         break;

   }
}

void search() {
    geometry_msgs::Twist cmd;

    velocity_publisher.publish(cmd);
}

void track() {
    geometry_msgs::Twist cmd;

    cmd.linear.x  = 0.1;
    cmd.angular.z = currentTarget * 3.14 / 180.0 * 1.0;
   
    velocity_publisher.publish(cmd);
}

void dodge() {
    geometry_msgs::Twist cmd;

    velocity_publisher.publish(cmd);
}

protected:
   ros::NodeHandle n_;
   ros::Subscriber odom_sub;
   ros::Subscriber scan_sub;
   ros::Subscriber hark_sub;
   ros::Publisher velocity_publisher;
   static bool currentlyObstructed;
   static bool currentlyTracking;
   static double currentTarget;
   static bool doneDodge;
   static int state;
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
