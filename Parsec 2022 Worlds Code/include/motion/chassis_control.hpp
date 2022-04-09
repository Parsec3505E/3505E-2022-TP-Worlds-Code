#ifndef CHASSIS_CONTROL_HPP
#define CHASSIS_CONTROL_HPP

#include "motion/odometry.hpp"

void locationTracking(Drivetrain* drivetrain);

extern double xTargetLocation;
extern double yTargetLocation;
extern double targetFacingAngle;

extern void odomDriveTo(double xTarget, double yTarget, double targetAngle);
extern void odomTurnTo(double targetAngle);
extern double odomTurnToPoint();
void setDrivePower(double theta);
void odomDrivePID();
void odomTurnPID();

int odomChassisControl(Drivetrain drivetrain);

extern void driveToPID(double inches);
extern void turnToPID();

void drivePID(Drivetrain drivetrain);
void turnPID();

int PIDControl(Drivetrain drivetrain);


#endif