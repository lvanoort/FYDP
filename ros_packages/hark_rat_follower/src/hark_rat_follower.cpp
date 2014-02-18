#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <hark_msgs/HarkSource.h>
#include <sstream>

class SubscribeThenPublish {

public:

SubscribeThenPublish(ros::NodeHandle &n) : 
  n_(n)
{
  pub_ = n_.advertise<geometry_msgs::Twist>("turtlebot_node/cmd_vel",1);
  sub_ = n_.subscribe("HarkSource", 1, &SubscribeThenPublish::subCallback, this);
}

void subCallback(const hark_msgs::HarkSourceConstPtr& msg)
{
  ROS_INFO("Received Num of Src [%d]", msg->exist_src_num);

  if(msg->exist_src_num < 1) {
    pub_.publish(geometry_msgs::Twist());    
  } else {
    int minid = 10000;
    int minidelem = 0;
    geometry_msgs::Twist cmd;
    for(int num = 0; num < msg->exist_src_num; num++) {
      if(msg->src[num].id > minid ) continue;
      minidelem = num;
      minid = msg->src[num].id;
      cmd.linear.x  = 0.1;
      cmd.angular.z = msg->src[num].theta * 3.14 / 180.0 * 1.0;
    }
    ROS_INFO("Received [ID,Azimuth] [%d,%f]", msg->src[minidelem].id, msg->src[minidelem].theta);
    pub_.publish(cmd);
  }

}

protected:
ros::NodeHandle n_;
ros::Subscriber sub_;
ros::Publisher pub_;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hark_turtlebot_follower");
  ros::NodeHandle npub;
  SubscribeThenPublish stp(npub);
  ros::spin();
  return 0;
}
