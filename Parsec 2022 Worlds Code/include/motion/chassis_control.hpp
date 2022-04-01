#ifndef CHASSIS_CONTROL_HPP
#define CHASSIS_CONTROL_HPP

#include "motion/odometry.hpp"

extern double xTargetLocation;
extern double yTargetLocation;
extern double targetFacingAngle;

extern void driveTo();
extern void turnTo();
extern void turnToPoint();
void setDrivePower(double theta);
void drivePID();
void turnPID();

int chassis_control();

#endif