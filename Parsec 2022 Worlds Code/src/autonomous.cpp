#include "autonomous.hpp"

const double TALL_NEUTRAL_X = 70.3;
const double TALL_NEUTRAL_Y = 70.3;


const double RIGHT_ALLIANCE_X = 0;
const double LEFT_ALLIANCE_Y = 0;


void highNeutralWinPoint()
{
    
    //Set-up initial heading (change 9 to the offset of stand-off from center of robot)
    //heading = 360 + atan2(TALL_NEUTRAL_Y + 9 - yPoseGlobal, TALL_NEUTRAL_X - xPoseGlobal);
    pros::Controller driver(pros::E_CONTROLLER_MASTER);

    Drivetrain drive = Drivetrain();
    Intake intake = Intake();
    Arm arm = Arm();
    Stick stick = Stick();

    //Reset Encoders
    drive.resetEncoders();
    
    intake.resetEncoder();
    arm.resetEncoder();
    stick.resetEncoder();

    double curEncoderValue = 0;

    //drive_arg* track_task_arg = new drive_arg;  
    //track_task_arg->drivetrain = drive;

    intake_arg* intake_task_arg = new intake_arg;
    intake_task_arg->intake = intake;

    arm_arg* arm_task_arg = new arm_arg;
    arm_task_arg->arm = arm;

    stick_arg* stick_task_arg = new stick_arg;
    stick_task_arg->stick = stick;
    
    //pros::Task odomTracking(poseTracking, track_task_arg);
    //pros::Task chassisControl(odomChassisControl, track_task_arg);
    //runChassisControl = false;

    pros::Task intakeTask(moveIntakeFor, intake_task_arg);
    pros::Task armTask(moveArmFor, arm_task_arg);
    pros::Task stickTask(moveStickFor, stick_task_arg);
    stick.setHold();
    
    //Change Numbers
    setTargetIntake(1000, 100);
    drive.resetEncoders();
    while(drive.getEncoderInchesAverage() < 35.0)
    {
        drive.runLeftDrive(127);
        drive.runRightDrive(127);
    }   
    //driver.print(2,2,"%.1f", drive.getEncoderInchesAverage());
    drive.stop();
    setTargetIntake(1300, 100);
    pros::delay(600);
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() > curEncoderValue - 40.0)
    {
        drive.runLeftDrive(-127);
        drive.runRightDrive(-80);
    }
    drive.stop();
    setTargetIntake(900, 100);
    pros::delay(500);
    drive.runRightDrive(-100);
    pros::delay(750);
    drive.stop();

    pros::delay(500);
    driveSeconds(drive, 100, -80);
    driveSeconds(drive, 250, 50);
    
    curEncoderValue = drive.getLeftEncoderInches();
    while(drive.getLeftEncoderInches() < curEncoderValue + 3.5)
    {
        drive.runLeftDrive(100);
        drive.runRightDrive(-100);
    }
    drive.stop();
    
    /*
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
        drive.runLeftDrive(-75);
        drive.runRightDrive(-75);
    }   
    drive.stop();
    pros::delay(100);
    setTargetIntake(200 + 900, 100);
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() < curEncoderValue + 10.0)
    {
        drive.runLeftDrive(100);
        drive.runRightDrive(100);
    }   
    drive.stop();
    curEncoderValue = drive.getEncoderInchesAverage();
    setTargetIntake(2240 + 900, 100); 
    setTargetArm(720.0, 100);
    while(drive.getEncoderInchesAverage() > curEncoderValue - 3.0)
    {
        drive.runLeftDrive(-100);
        drive.runRightDrive(-100);
    }  
    drive.stop();
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() < curEncoderValue + 2.3)
    {
        drive.runLeftDrive(40);
        drive.runRightDrive(40);
    }  
    drive.stop();
    //718 just for margin
    while(arm.getEncoderRaw() < 718.0)
    {
        //driver.print(2,2,"%.2f", arm.getEncoderRaw());
        pros::delay(10);
    }
    setTargetStick(-115.2, 20);
    pros::delay(1500);

    drive.resetEncoders();
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() > curEncoderValue - 6)
    {
        drive.runLeftDrive(40);
        drive.runRightDrive(-100);
    }
    drive.stop();
    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() > curEncoderValue - 5.0)
    {
        drive.runLeftDrive(-100);
        drive.runRightDrive(-100);
    }  
    drive.stop();
    setTargetIntake(1000, 100);
    intakeTask.suspend();
    stickTask.suspend();
    armTask.suspend();
    pros::delay(2000);
    //odomTracking.suspend();
    //chassisControl.suspend();
    
    
    
    


    /*
    while(drive.getLeftEncoderInches() - drive.getRightEncoderInches() < 4.5)
    {
        drive.runLeftDrive(100);
        drive.runRightDrive(-30);
    }
    drive.stop();
    */
    pros::delay(3000);
    /*
    setTargetIntake(1500, 100);
    driveToPID(-27);
    waitWhile(runPIDChassisControl);
    pros::Task chassisControl(odomChassisControl, track_task_arg);
    pros::Task PIDChassiControl(PIDControl, track_task_arg);
    */
    /*
    odomDriveTo(140.7 - 9, 0 + 9, 0);
    waitWhile(runChassisControl);
    driveSeconds(drive, 1, 100);
    odomDriveTo(129.2, 35 - 10, 0);
    waitWhile(runChassisControl);
    */

    //KILL TASKS
}
