#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist input;

void joy_callback(const sensor_msgs::Joy& msg)
{
  input.linear.x = msg.axes[1]*(0.6271+msg.buttons[7]);
  input.angular.z = msg.axes[0]*(2.5);
}

int main(int argc, char* argv [])
{
  ros::init(argc,argv,"joycontrol");
  ros::NodeHandle n;
  
  ros::Subscriber pose_sub = n.subscribe("/joy", 1, joy_callback);  
  ros::Publisher inputpub = n.advertise<geometry_msgs::Twist>("/command", 1);  
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    inputpub.publish(input);
    ros::spinOnce();
    loop_rate.sleep();
  }
	
  return 0;
}
