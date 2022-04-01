#include "motion/chassis_control.hpp"

double xTargetLocation = xPoseGlobal;
double yTargetLocation = yPoseGlobal;
double targetFacingAngle = 0;

double xDistToTarget = 0;
double yDistToTarget = 0;

double hypotenuseAngle = 0;

double robotRelativeAngle = 0;

void driveTo(double xTarget, double yTarget, double targetAngle)
{
    xTargetLocation = xTarget;
    yTargetLocation = yTarget;
    targetFacingAngle = targetAngle;
}

void turnTo(double targetAngle)
{
    targetFacingAngle = targetAngle;
    xTargetLocation = xPoseGlobal;
    yTargetLocation = yPoseGlobal;
}

double turnToPoint(double xCoordTarget, double yCoordTarget)
{

    targetFacingAngle = atan2(yCoordTarget - yPoseGlobal, xCoordTarget - xPoseGlobal);

    if(targetFacingAngle < 0)
    {
        targetFacingAngle += 2 * PI;   
    }

    double angle = fmod((targetFacingAngle - heading + (3 * PI)), (2 * PI) - PI);
    return angle;
}
