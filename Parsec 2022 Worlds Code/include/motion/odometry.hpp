#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "main.h"

extern const double PI;

extern double heading;
extern double prevHeading;

extern double xPoseGlobal;
extern double yPoseGlobal;

extern bool runOdomTracking;

void resetTracking();

void poseTracking(void* arg);


#endif