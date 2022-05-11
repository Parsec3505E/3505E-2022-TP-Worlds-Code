#include "autonomous.hpp"

const double TALL_NEUTRAL_X = 70.3;
const double TALL_NEUTRAL_Y = 70.3;

const double RIGHT_ALLIANCE_X = 0;
const double LEFT_ALLIANCE_Y = 0;

intake_arg* intake_task_arg = new intake_arg;
pros::Task intakeTask(moveIntakeFor, intake_task_arg, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_MIN);

arm_arg* arm_task_arg = new arm_arg;
pros::Task armTask(moveArmFor, arm_task_arg, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_MIN);

stick_arg* stick_task_arg = new stick_arg;
pros::Task stickTask(moveStickFor, stick_task_arg, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_MIN);

drive_arg* track_task_arg = new drive_arg;
pros::Task odomTracking(poseTracking, track_task_arg, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_MIN);
pros::Task chassisControl(odomChassisControl, track_task_arg, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_MIN);




void skills()
{
    Intake intake = Intake();
    Drivetrain drive = Drivetrain();
    Arm arm = Arm();
    Stick stick = Stick();
    track_task_arg->drivetrain = drive;
    intake.resetEncoder();
    arm.resetEncoder();
    stick.resetEncoder();
    intake_task_arg->intake = intake;
    arm_task_arg->arm = arm;
    stick_task_arg->stick = stick;
    drive.resetEncoders();
    runChassisControl = true;
    runOdomTracking = true;
    /*
    odomDriveTo(70.3, 70.3, 100.0, 10.0);
    while(runChassisControl)
    {
        pros::delay(1);
    }
    drive.stop();
    */
}


void endAllTasks()
{
    stickTask.remove();
    intakeTask.remove();
    armTask.remove();
    odomTracking.remove();
    chassisControl.remove();
}


void odomPrint()
{
    Drivetrain drive = Drivetrain(); 
    track_task_arg->drivetrain = drive;

    drive.resetEncoders();
    resetTracking();
    runOdomTracking = true;
}


void highNeutralWinPoint()
{
    //Set-up initial heading (change 9 to the offset of stand-off from center of robot)
    //heading = 360 + atan2(TALL_NEUTRAL_Y + 9 - yPoseGlobal, TALL_NEUTRAL_X - xPoseGlobal);
    pros::Controller driver(pros::E_CONTROLLER_MASTER);

    Intake intake = Intake();
    Drivetrain drive = Drivetrain();
    Arm arm = Arm();
    Stick stick = Stick();

    //Reset Encoders (drive resetted later)
    intake.resetEncoder();
    arm.resetEncoder();
    stick.resetEncoder();



    intake_task_arg->intake = intake;
    arm_task_arg->arm = arm;
    stick_task_arg->stick = stick;
    track_task_arg->drivetrain = drive;
    runOdomTracking = false;

    double curEncoderValue = 0.0;
    stick.setHold();
    
    //Change Numbers
    setTargetIntake(1000, 100);
    drive.resetEncoders();
    setStartingPosition(104.0, 8.75, 0.5);
    runOdomTracking = true;
    runChassisControl = false;
    while(drive.getEncoderInchesAverage() < 35.0)
    {
        drive.runLeftDrive(127);
        drive.runRightDrive(127);
    }   
    //driver.print(2,2,"%.1f", drive.getEncoderInchesAverage());
    drive.stop();
    //HOOK TALL GOAL
    setTargetIntake(1300+500, 100);
    pros::delay(600);
    drive.triggerPincer(true);
    //Change Turn Factor
    
    odomDriveTo(106.0, 9.0, -127, 0.5, 0);
    runChassisControl = true;
    while(yPoseGlobal > 11.0)
    {
        pros::delay(5);
    }
    runChassisControl = false;
    drive.stop();
    /*
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() > curEncoderValue - 40.0)
    {
        drive.runLeftDrive(-127);
        drive.runRightDrive(-80);
    }
    drive.stop();
    */

   //LET GO OF TALL GOAL
    drive.triggerPincer(false);
    pros::delay(100);
    
    setTargetIntake(900, 100);
    pros::delay(500);
    drive.runRightDrive(-100);
    pros::delay(750);
    drive.stop();

    pros::delay(500);
    driveSeconds(drive, 100, -80);
    driveSeconds(drive, 250, 50);
    
    curEncoderValue = drive.getLeftEncoderInches();
    while(drive.getLeftEncoderInches() < curEncoderValue + 4.0)
    {
        drive.runLeftDrive(100);
        drive.runRightDrive(-100);
    }
    drive.stop();

    /* ODOM ATTEMP (DONT USE)
    double xTemp = xPoseGlobal;
    double yTemp = yPoseGlobal;
    double headingTemp = heading;
    
    //Recalibrate odom
    
    runOdomTracking = false;
   
    pros::delay(50);
    drive.resetEncoders();
    resetTracking();
    yPoseGlobal = 9.0;
    xPoseGlobal = fabs(xTemp*cos(headingTemp) + yTemp*sin(headingTemp)) + 104.0;
    //xPoseGlobal = 113.0;
    pros::delay(50);
    //ODOM BRICKED
    runOdomTracking = true;
    if(xPoseGlobal > 120.0)
    {
        xPoseGlobal -= 20;
    }
    odomTurnToPos(129.2, 35.0);
    driver.print(2,2,"Hi: %.1f", xPoseGlobal);
    pros::delay(50);
    while(runChassisControl)
    {
        pros::delay(10);
    }
    */
    
   
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() < curEncoderValue + 17.0)
    {
        drive.runLeftDrive(100);
        drive.runRightDrive(100);
    }   
    drive.stop();
    pros::delay(100);
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() > curEncoderValue - 10.0)
    {
        drive.runLeftDrive(-60);
        drive.runRightDrive(-75);
    }   
    drive.stop();
    pros::delay(100);
    setTargetIntake(200 + 900, 100);

    //DRIVE INTO ALLIANCE
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() < curEncoderValue + 10.0)
    {
        drive.runLeftDrive(100);
        drive.runRightDrive(100);
    }   
    drive.stop();
  
    curEncoderValue = drive.getLeftEncoderInches();
    
    //TURN AWAY FROM WALL
    while(drive.getLeftEncoderInches() > curEncoderValue - 1.5)
    {
        drive.runLeftDrive(-70);
        drive.runRightDrive(70);
    }  
    drive.stop();

    pros::delay(250);
    driveSeconds(drive, 500, 90);
    pros::delay(250);

    //PICK UP ALLIANCE
    setTargetIntake(2240 + 900, 100); 
    setTargetArm(720.0, 100);

    // while(drive.getEncoderInchesAverage() > curEncoderValue - 0.25)
    // {
    //     drive.runLeftDrive(-20);
    //     drive.runRightDrive(-20);
    // }  
    // drive.stop();
    /*
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() < curEncoderValue + 2.3)
    {
        drive.runLeftDrive(40);
        drive.runRightDrive(40);
    }  
    drive.stop();
    */
    //718 just for margin
    while(arm.getEncoderRaw() < 718.0)
    {
        //driver.print(2,2,"%.2f", arm.getEncoderRaw());
        pros::delay(10);
    }
    // multiplied by 2 for torque cartirdge
    setTargetStick(-115.2 * 2, 20);
    pros::delay(2000);

    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() > curEncoderValue - 2.5)
    {
        drive.runLeftDrive(-100);
        drive.runRightDrive(-100);
    }  
    drive.stop();

     pros::delay(500);

    /* NON ODOM TURN (INCONSISTENT)
    drive.resetEncoders();
    curEncoderValue = drive.getEncoderInchesAverage();
    setTargetIntake(1500, 100);
    while(drive.getEncoderInchesAverage() > curEncoderValue - 3.0)
    {
        drive.runLeftDrive(40);
        drive.runRightDrive(-100);
    }
    drive.stop();
    */

    //ODOM USED HERE  
    drive.resetEncoders();
    resetTracking();
    runChassisControl = false;
    runOdomTracking = true;
    
    //TURN TO TALL GOAL
    odomTurnToHeading(90.0 * (PI/180.0));
    while(runChassisControl)
    {
        pros::delay(5);
    }
    odomTracking.suspend();
    chassisControl.suspend();
    curEncoderValue = drive.getEncoderInchesAverage();
    setTargetIntake(1000, 100);
    while(drive.getEncoderInchesAverage() > curEncoderValue - 5.0)
    {
        drive.runLeftDrive(-100);
        drive.runRightDrive(-100);
    }  
    drive.stop();
    intakeTask.suspend();
    stickTask.suspend();
    armTask.suspend();

}

void leftSideAuton()
{
    //Set-up initial heading (change 9 to the offset of stand-off from center of robot)
    //heading = 360 + atan2(TALL_NEUTRAL_Y + 9 - yPoseGlobal, TALL_NEUTRAL_X - xPoseGlobal);

    Intake intake = Intake();
    Drivetrain drive = Drivetrain();
    Arm arm = Arm();
    Stick stick = Stick();

    //Reset Encoders (drive resetted later)
    intake.resetEncoder();
    arm.resetEncoder();
    stick.resetEncoder();

    intake_task_arg->intake = intake;
    arm_task_arg->arm = arm;
    stick_task_arg->stick = stick;
    track_task_arg->drivetrain = drive;
    setStartingPosition(23.2, 12.0, 0.0);
    runOdomTracking = false;

    double curEncoderValue = 0.0;
    stick.setHold();
    
    //------MOVE FORWARD------
    setTargetIntake(1000, 100);
    pros::delay(200);
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() < curEncoderValue + 25.0)
    {
        drive.runLeftDrive(127);
        drive.runRightDrive(127);
    } 
    drive.stop();
    
    //------PICKUP + MOVE BACK------
    setTargetIntake(1800, 100);
    pros::delay(300);
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getLeftEncoderInches() >  curEncoderValue - 23.0)
    {
        drive.runLeftDrive(-100);
        drive.runRightDrive(-100);
    }
    drive.stop();
    pros::delay(150);

    //------MANUAL TURN------
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getLeftEncoderInches() < curEncoderValue+ 4.5)
    {
        drive.runLeftDrive(100);
        drive.runRightDrive(-100);
    }
    drive.stop();
    pros::delay(500);

    //--------DROP SECONDARY--------
    setTargetIntake(-550, 100);
    pros::delay(2000);

    //------SMALL BACKUP------
    // curEncoderValue = drive.getEncoderInchesAverage();
    // while(drive.getLeftEncoderInches() > curEncoderValue- 2.0)
    // {
    //     drive.runLeftDrive(-80);
    //     drive.runRightDrive(-80);
    // }
    // drive.stop();
    // pros::delay(500);

    //-----ODOM TURN-----
    runOdomTracking = true;
    odomTurnToHeading(58.0 * (PI/180.0));
    while(runChassisControl)
    {
        pros::delay(5);
    }
    runOdomTracking = false;
    pros::delay(200);
    
    //-----WALL SQR-----
    driveSeconds(drive, 1000, -100);
    drive.runRightDrive(-127);
    pros::delay(500);




    
    //SHIMMY
    driveSeconds(drive, 1600, 100);
    driveSeconds(drive, 500, -100);

    //GO IN TO SCOOP BLUE
    curEncoderValue = drive.getLeftEncoderInches();
    while(drive.getLeftEncoderInches() < curEncoderValue + 15.0)
    {
        drive.runLeftDrive(80);
        drive.runRightDrive(80);
    }
    drive.stop();
   
   //SCOOP BLUE GOAL
    setTargetIntake(2000, 100);
    pros::delay(1000);

    //MOVE BACK W/ GOAL
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() > curEncoderValue - 10.0)
    {
        drive.runLeftDrive(-75);
        drive.runRightDrive(-75);
    }   
    drive.stop(); 

    //LIFT BLUE HIGHER AND LIFT ARM
    curEncoderValue = drive.getEncoderInchesAverage();
    setTargetIntake(3200, 100); 
    setTargetArm(815.0, 100);
    while(drive.getEncoderInchesAverage() > curEncoderValue - 3.0)
    {
        drive.runLeftDrive(-100);
        drive.runRightDrive(-100);
    }  
    drive.stop();
    
    // multiplied by 2 for torque cartirdge
    setTargetStick(-115.2 * 2, 20);
    pros::delay(2000);

    
    intakeTask.suspend();
    stickTask.suspend();
    armTask.suspend();
    pros::delay(3000);

}
