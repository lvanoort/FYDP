#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

typedef struct State_ {
  double x; // x position
  double y; // y position
  double t; // yaw
} State;

State current_state;

void encoder_callback(const nav_msgs::Odometry& msg)
{
  static bool first = true;
  static ros::Time last_integrate;
  if(first)
  {
    last_integrate = ros::Time::now();
    first = false;
    return;
  }

  ros::Time current_time = ros::Time::now();

  double dt = (current_time - last_integrate).toSec();

  State new_state = current_state;
  //fovis convention: x right, y down, z forward
  new_state.x += msg.twist.twist.linear.z*cos(current_state.t)*dt + 
                 msg.twist.twist.linear.x*sin(current_state.t)*dt;
  new_state.y += msg.twist.twist.linear.z*sin(current_state.t)*dt -
                 msg.twist.twist.linear.x*cos(current_state.t)*dt;
  new_state.t += -msg.twist.twist.angular.y*dt;
  if(new_state.t > M_PI)
    new_state.t -= 2*M_PI;
  else if(new_state.t < -M_PI)
    new_state.t += 2*M_PI;
    
  current_state = new_state;
  last_integrate = current_time;
  
  //ROS_INFO("dt=%f",dt);
  ROS_INFO("x=%f y=%f t=%f", current_state.x, current_state.y, current_state.t);
}

int main(int argc, char* argv [])
{
  ros::init(argc,argv,"encoder_integration");
  ros::NodeHandle n;

  current_state.x = 0;
  current_state.y = 0;
  current_state.t = 0;
  
  ros::Subscriber enc_sub = n.subscribe("/mono_depth_odometer/odometry", 1, encoder_callback);

  ros::spin();

  return 0;
}
