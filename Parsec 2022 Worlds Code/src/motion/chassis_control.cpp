#include "motion/chassis_control.hpp"

// ########################### USING ODOMETRY ###########################

double xTargetLocation = xPoseGlobal;
double yTargetLocation = yPoseGlobal;
double targetFacingAngle = 0;

double xDistToTarget = 0;
double yDistToTarget = 0;

double hypotenuseAngle = 0;

double robotRelativeAngle = 0;

// Drive PID variables/gains
//double odomError = 0;
double odomPrevError = 0;
double odomMaxError = 0;

double odomError = 0.1;

double odomIntegral = 0;
double odomIntegralBound = 1.5;

double odomDerivative = 0;

double odomkP = 1;
double odomkI = 1;
double odomkD = 1;

// The output power of the PID to the motors
double odomPIDPower = 0;

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

//Decide whether drive or just turn odom command
bool justTurn = false;

void odomDriveTo(double xTarget, double yTarget)
{
  //Reset some PID variables
  odomPrevError = 0;
  odomIntegral = 0;

  xTargetLocation = xTarget;
  yTargetLocation = yTarget;
  justTurn = false;
}

void odomTurnTo(double targetAngle)
{
  //Reset some PID variables
  turnIntegral = 0;
  turnPrevError = 0;

  targetFacingAngle = targetAngle;
  justTurn = true;
}

double getAngleToTarget()
{

  targetFacingAngle = atan2(yTargetLocation - yPoseGlobal, xTargetLocation - xPoseGlobal);

  if(targetFacingAngle < 0)
  {
    targetFacingAngle += 2 * PI;
  }

  double angle = targetFacingAngle - heading;

  if(angle >= 2 * PI)
  {
    angle -= 2 * PI;
  }
  else if(angle <= -2 * PI)
  {
    angle += 2 * PI;
  }
  else if(angle < 0)
  {
    angle += 2 * PI;
  }

  //double angle = fmod((targetFacingAngle - heading + (3 * PI)), (2 * PI) - PI);
  return angle;
}

double getDistToTarget(){
    double dist = sqrt(pow((xPoseGlobal - xTargetLocation), 2) + pow((yPoseGlobal - yTargetLocation), 2));
    return dist;
}


void odomDrivePID()
{
    // Proportional
    odomError = getDistToTarget();
  
    // Integral
    if(fabs(odomError) < odomIntegralBound)
    {
    odomIntegral += odomError;
    }
    else
    {
    odomIntegral = 0;
    }

    // Derivative
    odomDerivative = odomError - odomPrevError;
    odomPrevError = odomError;

    // Calculating the power coming out of the PID
    odomPIDPower = (odomError * odomkP + odomIntegral * odomkI + odomDerivative * odomkD);

    /*
    //Limit power output to 127
    if(odomPIDPower > 127)
    {
      odomPIDPower = 127;
    }
    else if(odomPIDPower < -127)
    {
      odomPIDPower = -127;
    }
    */

    if(fabs(odomError) < odomMaxError)
    {
      odomPIDPower = 0;
    }
}

void odomTurnPID()
{
  if(justTurn == false)
  {
    turnError = getAngleToTarget();
  }
  else{
    turnError = targetFacingAngle;
  }

  //WHEN WILL turnError EVER BE NEGATIVE????

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

  /*
  //Limit power output to 12V
  if(turnPIDPower > 127) {
    turnPIDPower = 127;
  }
  else if(turnPIDPower < -127)
  {
    turnPIDPower = -127;
  }
  */

  if(fabs(turnError) < turnMaxError) {
    turnPIDPower = 0;
  }
}


// Motor Powers
double odomRightSidePower = 0;
double odomLeftSidePower = 0;

// The output power of the PID to the motors
double drivePIDPower = 0;

//  Adjust Motor Power Manually
double drivePowerFactor = 1;

void odomChassisControl(void* arg)
{
  Drivetrain drive_temp = ((drive_arg*)arg)->drivetrain;
  pros::Controller driver(pros::E_CONTROLLER_MASTER);
    while(true)
    {
      /*
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

      if(robotRelativeAngle >= 2 * PI)
      {
        robotRelativeAngle -= 2 * PI;
      }
      else if(robotRelativeAngle <= -2 * PI)
      {
        robotRelativeAngle += 2 * PI;
      }
      else if(robotRelativeAngle < 0)
      {
        robotRelativeAngle += 2 * PI;
      }
      */

      // Get PID drive and turn powers
      /*
      if(justTurn == false)
      {
        odomDrivePID();
      }
      else{
        odomPIDPower = 0;
      }
      */

      odomDrivePID();
      odomTurnPID();
      

      odomRightSidePower = odomPIDPower;
      odomLeftSidePower = odomPIDPower;

      if(odomRightSidePower > 127)
      {
        odomRightSidePower = 127;
      }
      else if(odomRightSidePower < -127)
      {
        odomRightSidePower = -127;
      }

      if(odomLeftSidePower > 127)
      {
        odomLeftSidePower = 127;
      }
      else if(odomLeftSidePower < -127)
      {
        odomLeftSidePower = -127;
      }

      drive_temp.runRightDrive(odomRightSidePower);
      drive_temp.runLeftDrive(odomLeftSidePower); 
      printf("%d", (int)odomRightSidePower);
      //driver.print(2,2,"%d", (int)odomRightSidePower);
      pros::delay(20);
 
    }
}


// ########################### ONLY USING PIDs ###########################



double driveTarget = 0;

void driveToPID(double inches)
{
  driveTarget = inches;
}


double driveError = 0;
double drivePrevError = 0;

double driveMaxError = 0.1;

double driveIntegral = 0;
double driveIntegralBound = 1.5;

double driveDerivative = 0;

double drivekP = 3;
double drivekI = 0;
double drivekD = 0;



void drivePID(Drivetrain drivetrain){

    driveError = driveTarget - drivetrain.getEncoderInchesAverage();
  
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

double rightSidePower = 0;
double leftSidePower =0;


int PIDControl(Drivetrain drivetrain)
{
  while(true){
    drivePID(drivetrain);

    rightSidePower = drivePIDPower - turnPIDPower;
    leftSidePower = drivePIDPower + turnPIDPower;

    
    drivetrain.runRightDrive(rightSidePower);
    drivetrain.runLeftDrive(leftSidePower); 
  }
  

}


