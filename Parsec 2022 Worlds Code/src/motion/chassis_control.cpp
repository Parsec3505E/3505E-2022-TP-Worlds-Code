#include "motion/chassis_control.hpp"

// ########################### USING ODOMETRY ###########################

bool runChassisControl = false;

double xTargetLocation = xPoseGlobal;
double yTargetLocation = yPoseGlobal;
double targetFacingAngle = 0.0;

double xDistToTarget = 0.0;
double yDistToTarget = 0.0;

double hypotenuseAngle = 0.0;

double robotRelativeAngle = 0.0;

// Drive PID variables/gains
//double odomError = 0.0;
double odomPrevError = 0.0;
double odomMaxError = 1.2;

double odomDriveError = 0.1;

double odomIntegral = 0.0;
double odomIntegralBound = 1.5;

double odomDerivative = 0.0;

double odomkP = 1;
double odomkI = 1;
double odomkD = 1;

// The output power of the PID to the motors
double odomPIDPower = 0.0;

// Turn PID variables/gains
double turnError = 0.0;
double turnPrevError = 0.0;

double turnMaxError = 0.1;

double turnIntegral = 0.0;
double turnIntegralBound = 0.09;

double turnDerivative = 0.0;

double turnkP = 200.00;
double turnkI = 2.00;
double turnkD = 0.00;

double turnPIDPower = 0.0;

//Decide whether drive or just turn odom command
bool justTurn = false;

double getAngleToTarget()
{

  double angle = atan2(yTargetLocation - yPoseGlobal, xTargetLocation - xPoseGlobal);

  /*
  while(angle < 0)
  {
    angle += 2 * PI;
  }
  */

  angle = (PI/2.0) - angle - heading;
  /*
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
  */

  //double angle = fmod((targetFacingAngle - heading + (3 * PI)), (2 * PI) - PI);
  return angle;
}

double getDistToTarget(){
    double dist = sqrt(pow((xPoseGlobal - xTargetLocation), 2) + pow((yPoseGlobal - yTargetLocation), 2));
    return dist;
}

void odomDriveTo(double xTarget, double yTarget)
{
  //Reset some PID variables
  odomPrevError = 0.0;
  odomIntegral = 0.0;
  turnIntegral = 0.0;
  turnPrevError = 0.0;

  xTargetLocation = xTarget;
  yTargetLocation = yTarget;
  justTurn = false;
  runChassisControl = true;
}

void odomTurnToPos(double xTarget, double yTarget)
{
  //Reset some PID variables
  turnIntegral = 0.0;
  turnPrevError = 0.0;
  xTargetLocation = xTarget;
  yTargetLocation = yTarget;
  targetFacingAngle = getAngleToTarget();
  justTurn = true;
  runChassisControl = true;
}

void odomTurnToHeading(double targetAngle)
{
  //Reset some PID variables
  turnIntegral = 0.0;
  turnPrevError = 0.0;

  justTurn = true;
  targetFacingAngle = targetAngle;
  runChassisControl = true;
}


void odomDrivePID()
{
    // Proportional
    odomDriveError = getDistToTarget();
  
    // Integral
    if(fabs(odomDriveError) < odomIntegralBound)
    {
    odomIntegral += odomDriveError;
    }
    else
    {
    odomIntegral = 0.0;
    }

    // Derivative
    odomDerivative = odomDriveError - odomPrevError;
    odomPrevError = odomDriveError;

    // Calculating the power coming out of the PID
    odomPIDPower = (odomDriveError * odomkP + odomIntegral * odomkI + odomDerivative * odomkD);

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

    if(fabs(odomDriveError) < odomMaxError)
    {
      odomPIDPower = 0.0;
    }
}

void odomTurnPID()
{
  turnError = targetFacingAngle - heading;

  //WHEN WILL turnError EVER BE NEGATIVE????

  if(fabs(turnError) > PI) {
    turnError = (turnError/fabs(turnError)) * -1 * fabs(2 * PI - turnError);
  }

  //only use integral if close enough to target
  if(fabs(turnError) < turnIntegralBound) {
    turnIntegral += turnError;
  }
  else {
    turnIntegral = 0.0;
  }

  //reset integral if we pass the target
  if(turnError * turnPrevError < 0) {
    turnIntegral = 0.0;
  } 

  turnDerivative = turnError - turnPrevError;

  turnPrevError = turnError;

  turnPIDPower = (turnError * turnkP + turnIntegral * turnkI + turnDerivative * turnkD);

  
  //Limit power output to 12V
  if(turnPIDPower > 127) {
    turnPIDPower = 127;
  }
  else if(turnPIDPower < -127)
  {
    turnPIDPower = -127;
  }
  

  if(fabs(turnError) < turnMaxError) {
    turnPIDPower = 0.0;
  }
}


// Motor Powers
double odomRightSidePower = 0.0;
double odomLeftSidePower = 0.0;

// The output power of the PID to the motors
double drivePIDPower = 0.0;

//Wheel Offset of actual powered wheels
double wheelOffset = 7.5;

//Drive Target Speed
double driveTargetSpeed = 0.0;

//Turn Factor for DriveToPoint()
double driveTurnFactor = 0.0;

void odomDriveTo(double xTarget, double yTarget, double speed, double turnFactor)
{
  /*
  //Reset some PID variables
  odomPrevError = 0.0;
  odomIntegral = 0.0;
  turnIntegral = 0.0;
  turnPrevError = 0.0;
  */
  driveTargetSpeed = speed;
  driveTurnFactor = turnFactor;

  xTargetLocation = xTarget;
  yTargetLocation = yTarget;
  justTurn = false;
  runChassisControl = true;
}

double driveToPointPower()
{
  return (2.0/getDistToTarget()) * driveTargetSpeed * wheelOffset * cos((PI/2.0) - getAngleToTarget());
}


void odomChassisControl(void* arg)
{
  Drivetrain drive_temp = ((drive_arg*)arg)->drivetrain;
  pros::Controller driver(pros::E_CONTROLLER_MASTER);
    while(true)
    {
      if(runChassisControl)
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
        
        if(justTurn == true)
        {
          odomTurnPID();
          odomRightSidePower = -turnPIDPower;
          odomLeftSidePower = turnPIDPower;
        }
        else{
          odomRightSidePower = driveTargetSpeed - (driveTurnFactor * driveToPointPower());
          odomLeftSidePower = driveTargetSpeed + (driveTurnFactor * driveToPointPower());
        }

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

        if(getDistToTarget() < odomMaxError && justTurn == false)
        {
          runChassisControl = false;
        }
        else if(fabs(turnError) < turnMaxError && justTurn == true)
        {
          runChassisControl = false; 
        }

        drive_temp.runRightDrive(odomRightSidePower);
        drive_temp.runLeftDrive(odomLeftSidePower); 
        //driver.print(2,2,"%.1f", (atan2(yTargetLocation - yPoseGlobal, xTargetLocation - xPoseGlobal)) * (180.0/PI));
        driver.print(2,2,"%.1f, %.1f", xPoseGlobal, yPoseGlobal);
        }
      pros::delay(10);
    }
}


// ########################### ONLY USING PIDs ###########################

bool runPIDChassisControl = false;

double driveTarget = 0.0;

double driveError = 0.0;
double drivePrevError = 0.0;

double driveIntegral = 0.0;

void driveToPID(double inches)
{
  drivePrevError = 0.0;
  driveIntegral = 0.0;
  driveTarget = inches;
  runPIDChassisControl = true;
}

double driveMaxError = 5;


double driveIntegralBound = 1.5;

double driveDerivative = 0.0;

double drivekP = 7;
double drivekI = 0.2;
double drivekD = 1;

void drivePID(Drivetrain drivetrain){

    driveError = driveTarget - drivetrain.getEncoderInchesAverage();
  
    // Integral
    if(fabs(driveError) < driveIntegralBound)
    {
    driveIntegral += driveError;
    }
    else
    {
    driveIntegral = 0.0;
    }

    // Derivative 
    driveDerivative = driveError - drivePrevError;
    drivePrevError = driveError;

    // Calculating the power coming out of the PID
    drivePIDPower = (driveError * drivekP + driveIntegral * drivekI + driveDerivative * drivekD);

    //Limit power output to 127
    if(drivePIDPower > 127)
    {
      drivePIDPower = 127;
    }
    else if(drivePIDPower < -127)
    {
      drivePIDPower = -127;
    }

    if(fabs(driveError) < driveMaxError)
    {
    drivePIDPower = 0.0;
    }

}

double rightSidePower = 0.0;
double leftSidePower =0.0;


void PIDControl(void* arg)
{
  Drivetrain drive_temp = ((drive_arg*)arg)->drivetrain;
  //pros::Controller driver(pros::E_CONTROLLER_MASTER);
  while(true){
    if(runPIDChassisControl)
    {
      drivePID(drive_temp);

      rightSidePower = drivePIDPower;
      leftSidePower = drivePIDPower;
      //driver.print(2,2,"%.2f", drivePIDPower);
      
      drive_temp.runRightDrive(rightSidePower);
      drive_temp.runLeftDrive(leftSidePower); 
      if(fabs(drivePIDPower) < driveMaxError)
      {
        runPIDChassisControl = false;
      }
    }
    pros::delay(10);
  }
}

//MOVE FOR SECONDS
void driveSeconds(Drivetrain drivetrain, int ms, int vel)
{
  drivetrain.runRightDrive(vel);
  drivetrain.runLeftDrive(vel);
  pros::delay(ms);
  drivetrain.stop();
}

