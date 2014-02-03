#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

//sensor_msgs::Joy sensor;
geometry_msgs::Twist input;



void joy_callback(const sensor_msgs::Joy& msg)
{
  input.linear.x = msg.axes[1]*12;
  input.linear.y = msg.axes[3]*12;
  //ROS_INFO ("hello");
}



int main(int argc, char* argv [])
{
  ros::init(argc,argv,"joycontrol");
  ros::NodeHandle n;
  
  ros::Subscriber pose_sub = n.subscribe("/joy", 1, joy_callback);
  //ros::Timer timer = n.createTimer(ros::Duration(0.02), timerCallback);
  
  ros::Publisher inputpub = n.advertise<geometry_msgs::Twist>("/command", 1);
  
  ros::Rate loop_rate(10);



  while (ros::ok())
  {

    inputpub.publish(input);

	ros::spinOnce();

    loop_rate.sleep();

  }
	
  
  return 0;
}
