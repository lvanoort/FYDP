#include "controller.h"

#define MOTOR_LIMIT 60.0
#define MOTOR_MAX 90.0

//#define KI 0.6666
#define KI 0.2;


double limit_command(double command, double limit) {
	if(command < (-limit)) 
		return (-limit);
	if(command > limit)
		return limit;
   return command;
}
int limit_command(int command, int limit) {
	if(command < (-limit)) 
		return (-limit);
	if(command > limit)
		return limit;
   return command;
}

int p_control(int recCommand, double speed, double Kp) {
	double error = ((recCommand/(12.0 / 0.6271)) - speed);
	double correction = error*Kp;
	double correctionAngle = correction * (12.0/.6271);
	double command = correctionAngle+recCommand;

   return (int)limit_command(command,MOTOR_MAX);
}

int controller_r(int recCommand, double speed) {
   double error = ((recCommand/(12.0 / 0.6271)) - speed);
   static double int_err_r = 0;
   int_err_r += error*0.02;
   double correction = error*KI;
   double correction_angle = correction * (12.0/.6271);
   double commandAngle = recCommand + correction_angle;

	return (int)(limit_command(commandAngle,MOTOR_LIMIT));
}

int controller_l(int recCommand, double speed) {
   double error = ((recCommand/(12.0 / 0.6271)) - speed);
   static double int_err_l = 0;
   int_err_l += error*0.02;
   double correction = error*KI;
   double correction_angle = correction * (12.0/.6271);
   double commandAngle = recCommand + correction_angle;

   return (int)(limit_command(commandAngle,MOTOR_LIMIT));
}
