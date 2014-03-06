#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <utility>
#include <math.h>
#include <vector>
 
#define DEBUG_VISUALIZATIONS 1
#define MAP_SIZE 200

typedef struct _ScanInfo {
  geometry_msgs::Pose2D pose;
  sensor_msgs::LaserScan scan;
} ScanInfo;

geometry_msgs::Pose2D current_position;
sensor_msgs::LaserScan current_scan;
nav_msgs::OccupancyGrid map;
std::vector<ScanInfo> last_infos;
bool last_info_received = false;

ros::Publisher viz_publisher;
ros::Publisher map_publisher;

static inline double wraparound(double angle) {
  if(angle > M_PI)
    angle -= 2*M_PI;
  else if(angle < -M_PI)
    angle += 2*M_PI;
    
  return angle;
}

static inline bool inMap(int x, int y)
{
  return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

std::pair<int,int> getIndexes(double x, double y)
{
  int yi = (int)((y - map.info.origin.position.y)/(map.info.resolution));
  int xi = (int)((x - map.info.origin.position.x)/(map.info.resolution));
  
  return std::pair<int,int>(xi,yi);
}

// fovis specific callback
void vodomCallback(const nav_msgs::Odometry msg)
{
  static bool first = true;
  static ros::Time last_update;
  if(first) {
    first = false;
    last_update = ros::Time::now();
    return;
  }
  
  ros::Time current_update = ros::Time::now();
  double dt = (current_update - last_update).toSec();
  last_update = current_update;

  //update position; fovis uses opencv standard camera coordinates to express odometry
  current_position.x     += msg.twist.twist.linear.z*cos(current_position.theta)*dt + msg.twist.twist.linear.x*sin(current_position.theta)*dt;
  current_position.y     += msg.twist.twist.linear.z*sin(current_position.theta)*dt - msg.twist.twist.linear.x*cos(current_position.theta)*dt;
  current_position.theta += -msg.twist.twist.angular.y*dt;
  
  current_position.theta = wraparound(current_position.theta);
}

void odomCallback(const nav_msgs::Odometry msg)
{
  static bool first = true;
  static ros::Time last_update;
  if(first) {
    first = false;
    last_update = ros::Time::now();
    return;
  }
  
  ros::Time current_update = ros::Time::now();
  double dt = (current_update - last_update).toSec();
  last_update = current_update;

  //update position
  current_position.x     += msg.twist.twist.linear.x*cos(current_position.theta)*dt;
  current_position.y     += msg.twist.twist.linear.x*sin(current_position.theta)*dt;
  current_position.theta += msg.twist.twist.angular.z*dt;
  
  current_position.theta = wraparound(current_position.theta);
}

void draw_map(const sensor_msgs::LaserScan& scan, double x, double y, double theta)
{
  std::pair<int,int> robot = getIndexes(x, y);
  map.data[robot.second*map.info.width + robot.first] = 0; 
  
  #pragma omp parallel for
  for(unsigned int i = 0; i < scan.ranges.size(); i+=20) { // for each ray
  
    if(isnan(scan.ranges[i]) || scan.ranges[i] > scan.range_max || scan.ranges[i] < scan.range_min)
      continue;
      
    double t  = theta + scan.angle_min + (i*scan.angle_increment);
    double px = x + scan.ranges[i]*cos(t);
    double py = y + scan.ranges[i]*sin(t);
    std::pair<int,int> point = getIndexes(px, py);

    // Bresenham's line algorithm, translated from wikipedia psuedocode
    double x0 = robot.first; double y0 = robot.second;
    double x1 = point.first; double y1 = point.second;
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
      std::swap(x0, y0);
      std::swap(x1, y1);
    }
    if (x0 > x1) {
      std::swap(x0, x1);
      std::swap(y0, y1);
    }
    int deltax = x1 - x0;
    int deltay = abs(y1 - y0);
    int error = deltax / 2;
    int ystep;
    int y = y0;
    if (y0 < y1) 
      ystep = 1;
    else 
      ystep = -1;
       
    for (int j = x0; j <x1; j++) { // from x0 to x1
      //update map with empty values
      if (steep) {
        if(inMap(y,j))
          map.data[j*map.info.width + y] = 0;
      }
      else {
        if(inMap(j,y))
	        map.data[y*map.info.width + j] = 0;
      }
         
      error = error - deltay;
      if (error < 0) {
        y = y + ystep;
        error = error + deltax;
      }
    }
    if(inMap(point.first,point.second)) { //end point
      map.data[point.second*map.info.width + point.first] = 100;
    }
  }

}

void scanCallback(const sensor_msgs::LaserScan& scan)
{
  
  //update map
  for (int i = 0; i < MAP_SIZE*MAP_SIZE; i++) {
    map.data[i] = 50;
  }
  
  draw_map(scan, 0, 0, 0);
  if(last_info_received) {
    geometry_msgs::Pose2D key_pose;
    key_pose.x = -(cos(current_position.theta)*current_position.x + sin(current_position.theta)*current_position.y);
    key_pose.y = -(-sin(current_position.theta)*current_position.x + cos(current_position.theta)*current_position.y);
    key_pose.theta = -current_position.theta;
		key_pose.theta = wraparound(key_pose.theta);
    
    draw_map(current_scan, key_pose.x, key_pose.y, key_pose.theta);

    for(int i = last_infos.size()-1; i >= 0; i--) {
      geometry_msgs::Pose2D last_pose;
      last_pose.x = cos(last_infos[i].pose.theta)*(key_pose.x - last_infos[i].pose.x) + sin(last_infos[i].pose.theta)*(key_pose.y - last_infos[i].pose.y);
      last_pose.y = cos(last_infos[i].pose.theta)*(key_pose.y - last_infos[i].pose.y) - sin(last_infos[i].pose.theta)*(key_pose.x - last_infos[i].pose.x);
      last_pose.theta = key_pose.theta - last_infos[i].pose.theta;
      key_pose = last_pose;
			key_pose.theta = wraparound(key_pose.theta);
			
      draw_map(last_infos[i].scan, key_pose.x, key_pose.y, key_pose.theta);
    }
  }
 
  map_publisher.publish(map);
  
  // Look up current pose to check for saving
  bool save_scan_and_pose = false;
  if(current_position.x*current_position.x + 
     current_position.y*current_position.y > 0.4*0.4) // if moved more than 25cm
    save_scan_and_pose = true;
  if(current_position.theta*current_position.theta > (20*M_PI/180)*(20*M_PI/180))
    save_scan_and_pose = true;
    
  if (save_scan_and_pose)
  {
    ROS_INFO("New keyframe");
    ROS_INFO("%f %f %f", current_position.x, current_position.y, current_position.theta);
    
    if(last_info_received) {
      ScanInfo last_info;
      last_info.scan = current_scan;
		  last_info.pose = current_position;
			  
			last_infos.push_back(last_info);
			//keep only n elements
			if(last_infos.size() > 9)
			  last_infos.erase(last_infos.begin(),last_infos.begin()+1);
    }
    
    current_scan = scan;
    current_position.x = 0;
    current_position.y = 0;
    current_position.theta = 0;
    
    last_info_received = true;
  }
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
 
int main(int argc, char **argv)
{

    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;
 
    ros::Subscriber scan_sub = n.subscribe("/scan", 1, scanCallback);
    //ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    ros::Subscriber odom_sub = n.subscribe("/mono_depth_odometer/odometry", 1, vodomCallback);
    
    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    //Map setup
    map.header.frame_id = "/camera_depth_frame"; //map will be local to robot
    map.info.resolution = 0.1; //10 cm grid sizes for speed
    map.info.width = MAP_SIZE; //200*0.1 = 6 meters
    map.info.height = MAP_SIZE;
    map.info.origin.position.x = -10; //map is centered about robot
    map.info.origin.position.y = -10;
    map.data.resize(MAP_SIZE*MAP_SIZE); //for visualization, use logit map otherwise

    ros::spin();

    return 0;
}
