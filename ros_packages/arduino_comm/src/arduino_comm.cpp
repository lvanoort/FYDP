#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int fd;
geometry_msgs::Twist cmd;

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
  //TODO: Map control inputs and outputs and fix sending stuff out
  write_commands(cmd.linear.x, cmd.linear.y);
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
	ros::Timer timer = n.createTimer(ros::Duration(0.02), timerCallback);
	
	ros::spin();
  
  return 0;
}
