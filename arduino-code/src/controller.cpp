#include "controller.h"

int proportional(int recCommand, double speed, double Kp) {
	double error = ((recCommand/(12.0 / 0.6271)) - speed);
	double commandVel = error*Kp;
	double commandAngle = commandVel * (12.0/.6271);
	if(commandAngle < -90.0) 
		commandAngle = -90.0;
	if(commandAngle > 90.0)
		commandAngle = 90.0;
	return (int)(commandAngle);
}
