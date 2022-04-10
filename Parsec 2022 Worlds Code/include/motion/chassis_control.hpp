#ifndef CHASSIS_CONTROL_HPP
#define CHASSIS_CONTROL_HPP

#include "motion/odometry.hpp"

extern bool runChassisControl;
extern bool runPIDChassisControl;

extern double xTargetLocation;
extern double yTargetLocation;
extern double targetFacingAngle;

extern double drivePowerFactor;

extern void odomDriveTo(double xTarget, double yTarget, double targetAngle);
extern void odomTurnTo(double xTarget, double yTarget);
extern double odomTurnToPoint();
void setDrivePower(double theta);
void odomDrivePID();
void odomTurnPID();

void odomChassisControl(void* arg);

extern void driveToPID(double inches);
extern void turnToPID();

void drivePID(Drivetrain drivetrain);
void turnPID();

void PIDControl(void* arg);

void driveSeconds(Drivetrain drivetrain, int seconds, int vel);

//Structs for tasks
typedef struct{
	Drivetrain drivetrain;
} drive_arg;

#endif