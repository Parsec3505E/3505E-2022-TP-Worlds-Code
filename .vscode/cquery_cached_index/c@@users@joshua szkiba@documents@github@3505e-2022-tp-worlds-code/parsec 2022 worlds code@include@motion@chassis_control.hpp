#ifndef CHASSIS_CONTROL_HPP
#define CHASSIS_CONTROL_HPP

#include "motion/odometry.hpp"

extern double xTargetLocation;
extern double yTargetLocation;
extern double targetFacingAngle;

extern void driveTo(double xTarget, double yTarget, double targetAngle);
extern void turnTo(double targetAngle);
extern double turnToPoint();
void setDrivePower(double theta);
void drivePID();
void turnPID();

int chassis_control();

#endif