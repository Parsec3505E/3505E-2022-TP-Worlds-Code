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

double getAngleToTarget()
{

    targetFacingAngle = atan2(yTargetLocation - yPoseGlobal, xTargetLocation - xPoseGlobal);

    if(targetFacingAngle < 0)
    {
        targetFacingAngle += 2 * PI;   
    }

    double angle = fmod((targetFacingAngle - heading + (3 * PI)), (2 * PI) - PI);
    return angle;
}

double getDistToTarget(){
    double dist = sqrt(pow((xPoseGlobal - xTargetLocation), 2) + pow((yPoseGlobal - yTargetLocation), 2));
    return dist;
}


// Drive PID variables/gains

double driveError = 0;
double drivePrevError = 0;

double driveMaxError = 0.1;

double driveIntegral = 0;
double driveIntegralBound = 1.5;

double driveDerivative = 0;

double drivekP = 0;
double drivekI = 0;
double drivekD = 0;

// The output power of the PID to the motors
double drivePIDPower = 0;

void drivePID()
{
    // Proportional
    driveError = getDistToTarget();
  
    // Integral
    if(fabs(driveError) < driveIntegralBound)
    {
    driveIntegral += driveError;
    }
    else
    {
    driveIntegral = 0;
    }

    // Derivative
    driveDerivative = driveError - drivePrevError;
    drivePrevError = driveError;

    // Calculating the power coming out of the PID
    drivePIDPower = (driveError * drivekP + driveIntegral * drivekI + driveDerivative * drivekD);

    //Limit power output to 12V
    if(drivePIDPower > 12)
    {
    drivePIDPower = 12;
    }

    if(fabs(driveError) < driveMaxError)
    {
    drivePIDPower = 0;
    }
}

// Turn PID variables/gains

double turnError = 0;
double turnPrevError = 0;

double turnMaxError = 0.01;

double turnIntegral = 0;
double turnIntegralBound = 0.09;

double turnDerivative = 0;

double turnkP = 13.00;
double turnkI = 1.00;
double turnkD = 10.00;

double turnPIDPower = 0;


void turnPID()
{
  turnError = getAngleToTarget();

  if(fabs(turnError) > PI) {
    turnError = (turnError/fabs(turnError)) * -1 * fabs(2 * PI - turnError);
  }

  //only use integral if close enough to target
  if(fabs(turnError) < turnIntegralBound) {
    turnIntegral += turnError;
  }
  else {
    turnIntegral = 0;
  }

  //reset integral if we pass the target
  if(turnError * turnPrevError < 0) {
    turnIntegral = 0;
  } 

  turnDerivative = turnError - turnPrevError;

  turnPrevError = turnError;

  turnPIDPower = (turnError * turnkP + turnIntegral * turnkI + turnDerivative * turnkD);

  //Limit power output to 12V
  if(turnPIDPower > 12) {
    turnPIDPower = 12;
  }

  if(fabs(turnError) < turnMaxError) {
    turnPIDPower = 0;
  }
}


// Motor Powers
double rightSidePower = 0;
double leftSidePower = 0;


int chassis_control()
{
    while(true)
    {
        // Distances to target point in both axis
        xDistToTarget = xTargetLocation - xPoseGlobal;
        yDistToTarget = yTargetLocation - yPoseGlobal;

        // Angle of the resultant/hypotenuse vector
        hypotenuseAngle = atan2(yDistToTarget, xDistToTarget);

        if(hypotenuseAngle < 0 )
        {
            hypotenuseAngle += 2 * PI;
        }

        // The angle the robot needs to travel in order to move toward the target
        robotRelativeAngle = hypotenuseAngle - heading + (2 * PI);

        if(robotRelativeAngle > 2)
        {
            robotRelativeAngle -= 2 * PI;
        }
        else if(robotRelativeAngle < 0)
        {
            robotRelativeAngle += 2 * PI;
        }

        // Get PID drive and turn powers
        drivePID();
        turnPID();
        

        rightSidePower = drivePIDPower - turnPIDPower;
        leftSidePower = drivePIDPower + turnPIDPower;

        
        drivetrain.runRightDrive(rightSidePower);
        drivetrain.runLeftDrive(leftSidePower); 

        pros::delay(20);
 
    }
    return 1;
}
