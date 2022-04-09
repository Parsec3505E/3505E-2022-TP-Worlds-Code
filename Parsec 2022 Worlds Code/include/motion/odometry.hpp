#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "main.h"

extern const double PI;

extern double heading;

extern double xPoseGlobal;
extern double yPoseGlobal;

void poseTracking(void* arg);

#endif