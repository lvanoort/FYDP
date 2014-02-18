#include "controller.h"

#define MOTOR_LIMIT 60.0

//#define KI 0.6666
#define KI 10


int proportional(int recCommand, double speed, double Kp) {
	double error = ((recCommand/(12.0 / 0.6271)) - speed);
	double commandVel = error*Kp;
	double commandAngle = commandVel * (12.0/.6271);
	if(commandAngle < -MOTOR_LIMIT) 
		commandAngle = -MOTOR_LIMIT;
	if(commandAngle > MOTOR_LIMIT)
		commandAngle = MOTOR_LIMIT;
	return (int)(commandAngle);
}

int controller_r(int recCommand, double speed) {
	double error = ((recCommand/(12.0 / 0.6271)) - speed);
  static double int_err_r = 0;
  int_err_r += error*0.02;
  double correction = error*KI;
  double correction_angle = correction * (12.0/.6271);
  double commandAngle = recCommand + correction_angle;

	if(commandAngle < -MOTOR_LIMIT) 
		commandAngle = -MOTOR_LIMIT;
	if(commandAngle > MOTOR_LIMIT)
		commandAngle = MOTOR_LIMIT;
	return (int)(commandAngle);
}

int controller_l(int recCommand, double speed) {
	double error = ((recCommand/(12.0 / 0.6271)) - speed);
  static double int_err_l = 0;
  int_err_l += error*0.02;
  double correction = error*KI;
  double correction_angle = correction * (12.0/.6271);
  double commandAngle = recCommand + correction_angle;

	if(commandAngle < -MOTOR_LIMIT) 
		commandAngle = -MOTOR_LIMIT;
	if(commandAngle > MOTOR_LIMIT)
		commandAngle = MOTOR_LIMIT;
	return (int)(commandAngle);
}
