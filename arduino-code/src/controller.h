#ifndef CONTROLLER_H
#define CONTROLLER_H

double limit_command(double command, double limit);
int limit_command(int command, int limit);

int p_control(int recCommand, double speed, double Kp);

int controller_r(int recCommand, double speed);

int controller_l(int recCommand, double speed);

#endif
