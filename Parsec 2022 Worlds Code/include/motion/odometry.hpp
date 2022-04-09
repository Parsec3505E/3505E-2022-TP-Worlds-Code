#ifndef PRIMARY_HPP
#define PRIMARY_HPP

#include "main.h"

extern const double PI;

extern double heading;

extern double xPoseGlobal;
extern double yPoseGlobal;

int poseTracking(Drivetrain* drivetrain);

#endif