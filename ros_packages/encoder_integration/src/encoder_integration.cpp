#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

typedef struct State_ {
  double x; // x position
  double y; // y position
  double t; // yaw
} State;

State current_state;

void encoder_callback(const geometry_msgs::Twist& msg)
{
  static bool first = true;
  static ros::Time last_integrate;
  if(first)
  {
    last_integrate = ros::Time::now();
    first = false;
  }

  ros::Time current_time = ros::Time::now();

  double dt = (current_time - last_integrate).toSec();

  State new_state;
  new_state.x += msg.linear.x*cos(current_state.t)*dt;
  new_state.y += msg.linear.x*sin(current_state.t)*dt;
  new_state.t += msg.angular.z*dt;
  if(new_state.t > M_PI)
    new_state.t -= 2*M_PI;
  if(new_state.t < -M_PI)
    new_state.t += 2*M_PI;
    
  current_state = new_state;
  last_integrate = current_time;
  
  ROS_INFO("x=%f y=%f t=%f", current_state.x, current_state.y, current_state.t);
}

int main(int argc, char* argv [])
{
  ros::init(argc,argv,"encoder_integration");
  ros::NodeHandle n;

  current_state.x = 0;
  current_state.y = 0;
  current_state.t = 0;
  
  ros::Subscriber enc_sub = n.subscribe("/odom", 1, encoder_callback);

  ros::spin();

  return 0;
}
