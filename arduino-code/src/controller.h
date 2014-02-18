#ifndef CONTROLLER_H
#define CONTROLLER_H

int proportional(int recCommand, double speed, double Kp);

int controller_r(int recCommand, double speed);

int controller_l(int recCommand, double speed);

#endif
