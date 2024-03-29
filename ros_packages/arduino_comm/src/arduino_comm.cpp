#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <boost/algorithm/string.hpp>



int fd;
geometry_msgs::Twist cmd;
ros::Publisher encoder_pub;
ros::Time last_arduino_message;

// Opens serial port
// returns -1 if error
int open_port(void)
{

  fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
  sleep(4);
  if (fd == -1) {
    perror("open_port: Unable to open /dev/ttyf1 - ");
    return fd;
  }

  //115200 bps, 8N1
  struct termios options;
  tcgetattr(fd, &options);
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_lflag |= ICANON;
  options.c_cflag &= ~CRTSCTS;
  tcsetattr(fd, TCSANOW, &options);

  return (fd);
}

void write_commands(int left, int right)
{
  if (left > 90)
    left = 90;
  else if (left < -90)
    left = -90;

  if (right > 90)
    right = 90;
  else if (right < -90)
    right = -90;

  unsigned char buf[10];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0;
  buf[3] = 40;
  buf[4] = (0xFF00 & left) >> 8;
  buf[5] = 0xFF & left;
  buf[6] = (0xFF00 & right) >> 8;
  buf[7] = 0xFF & right;
  int checksum = left + right + 40;
  buf[8] = (0xFF00 & checksum) >> 8;
  buf[9] = 0xFF & checksum;
  
  int n = write(fd, buf, 10);
  if (n != 10)
    printf("Error\n");
}

void command_callback(const geometry_msgs::Twist& msg)
{
  cmd = msg;
}

void timerCallback(const ros::TimerEvent& e)
{
  // Saturate commands to stop crazy thing from happening
  if (cmd.angular.z > 2.0)
    cmd.angular.z = 2.0;
  else if (cmd.angular.z < -2.0)
    cmd.angular.z = -2.0;

/*  if (cmd.angular.z > 2.0)
    cmd.angular.z = 2.0;
  else if (cmd.angular.z < -2.0)
    cmd.angular.z = -2.0;*/

  // Map control inputs and outputs and fix sending stuff out
  double x_cmd = (cmd.linear.x*12.0/0.6271) - (cmd.angular.z*12.0/0.92);
  double y_cmd = (cmd.linear.x*12.0/0.6271) + (cmd.angular.z*12.0/0.92);
  
  
  write_commands(-x_cmd, -y_cmd);
  
  //Health check
  static int ticks = 0;
  ticks++;
  if(ticks > 50) {
    //ROS_INFO("ALIVE");
    ticks = 0;

    if( (ros::Time::now() - last_arduino_message).toSec() > 2)
      ROS_INFO("Arduino appears to be dead");
  }

}

void parse_string(std::string s)
{
  if(!s.c_str())
    return;

  std::string delimiters("RL");
  std::vector<std::string> tokens;
  boost::split(tokens,s,boost::is_any_of(delimiters));
  
  // First token is blank
  // Second is right
  // Third is left
  if (tokens.size() != 3) {
    //ROS_ERROR("Parsing error # of tokens = %d", tokens.size());
    return;
  } else {
    //ROS_INFO("Sup");
  }
  
  int r = atoi(tokens[1].c_str());
  int l = atoi(tokens[2].c_str());

  double right_speed = 0.0256*r; //m/s
  double left_speed = 0.0256*l;

  geometry_msgs::Twist msg;
  msg.linear.x = (right_speed + left_speed)/2.0;
  msg.linear.y = right_speed;
  msg.linear.z = left_speed;
  msg.angular.z = (right_speed - left_speed)*2.318;
  msg.angular.x = 0;
  msg.angular.y = 0;

  encoder_pub.publish(msg);

  last_arduino_message = ros::Time::now();
}

int main(int argc, char* argv [])
{
  ros::init(argc,argv,"arduino_comm");
  ros::NodeHandle n;
  
  int port = open_port();
  if (port == -1)
    return -1; //TODO: report error
  
  ROS_INFO("Serial port openned");
  
  ros::Subscriber pose_sub = n.subscribe("/command", 1, command_callback);
  encoder_pub = n.advertise<geometry_msgs::Twist>("/encoder_odom", 1);
  ros::Timer timer = n.createTimer(ros::Duration(0.02), timerCallback);
  
  last_arduino_message = ros::Time::now();
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  char buffer[32];
  int buf_pos = 0;
  while(ros::ok())
  {
    while(ros::ok()) {
      read(fd, buffer+buf_pos, 1);//TODO: non blocking checks and what not
      if( buffer[buf_pos] == '\n' || buf_pos == 31) break;
      buf_pos++;
    }
    
    if(buf_pos == 31)
    {
      //TODO: report error, this is odd
      buf_pos = 0;
    }
    else if(buffer[buf_pos] == '\n')
    {
      std::string s = std::string(buffer, buf_pos + 1);
      parse_string(s);
      buf_pos = 0;
    }
    
  }
  
  spinner.stop();
  return 0;
}
