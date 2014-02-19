#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <hark_msgs/HarkSource.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

ros::Publisher bearing_publisher;

void publishVisualization(double angle)
{ 
  double distance = 1.5;
  
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/camera_depth_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = cos(angle)*distance;
  marker.pose.position.y = sin(angle)*distance;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.7;

  marker.lifetime = ros::Duration(5);
	
	bearing_publisher.publish(marker);
}

void harkCallback(const hark_msgs::HarkSourceConstPtr& msg)
{
  //ROS_INFO("Received Num of Src [%d]", msg->exist_src_num);

  if(msg->exist_src_num < 1) {
    //nothing 
  } else {
    int minid = 10000;
    int minidelem = 0;
    for(int num = 0; num < msg->exist_src_num; num++) {

      if(msg->src[num].id > minid )
        continue;
      if(msg->src[num].power < 28 )
        continue;
      

      
      minidelem = num;
      minid = msg->src[num].id;
      
      double angle = msg->src[num].theta * M_PI/180;
      publishVisualization(angle);
    }

  }

}

double forward = 0;
double turn = 0;

void odomCallback(const geometry_msgs::Twist& msg)
{
  forward = msg.linear.x;
  turn = msg.angular.z;
}

int main(int argc, char **argv)
{
  //Initialize the ROS framework
  ros::init(argc,argv,"main_control");
  ros::NodeHandle n;
  
  //ros::Subscriber hark_sub = n.subscribe("HarkSource", 1, &harkCallback);
  ros::Subscriber hark_sub = n.subscribe("/encoder_odom", 1, &odomCallback);
  bearing_publisher = n.advertise<visualization_msgs::Marker>("robot_goal",1);
  ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("/command",1);
  
  ros::Rate loop_rate(10);
  double angle;
  while(angle < M_PI/2) {
    angle += turn*0.1;
    geometry_msgs::Twist command;
    command.linear.x = 0;
    command.linear.y = 0;
    command.linear.z = 0;
    command.angular.x = 0;
    command.angular.y = 0;
    command.angular.z = 0.3;
    command_pub.publish(command);
    loop_rate.sleep();
    ros::spinOnce();
  }
  
  for( int i = 0; i < 20; i++) {
    geometry_msgs::Twist command;
    command.linear.x = 0;
    command.linear.y = 0;
    command.linear.z = 0;
    command.angular.x = 0;
    command.angular.y = 0;
    command.angular.z = 0.0;
    command_pub.publish(command);
    loop_rate.sleep();
    ros::spinOnce();
  }
  // turn left
  
  
  return 0;
  
}
