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
    setTargetIntake(1300, 100);
    pros::delay(600);
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
    // multiplied by 2 for torque cartirdge
    setTargetStick(-115.2 * 2, 20);
    pros::delay(2000);

    curEncoderValue = drive.getEncoderInchesAverage();
    while(drive.getEncoderInchesAverage() > curEncoderValue - 5.0)
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
    
    odomTurnToHeading(70.0 * (PI/180.0));
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
    
    //Change Numbers
    setTargetIntake(1000, 100);
    pros::delay(200);
    drive.resetEncoders();
    while(drive.getEncoderInchesAverage() < 25.0)
    {
        drive.runLeftDrive(127);
        drive.runRightDrive(127);
    } 
    //driver.print(2,2,"%.1f", drive.getEncoderInchesAverage());
    drive.stop();
    //pros::delay(600);

    setTargetIntake(1500, 100);
    pros::delay(300);

    drive.resetEncoders();
    while(drive.getLeftEncoderInches() > -15.0)
    {
        drive.runLeftDrive(-100);
        drive.runRightDrive(-100);
    }
    drive.stop();
    //pros::delay(750);


    /*drive.resetEncoders();
    while(drive.getLeftEncoderInches() < 5.0)
    {
        drive.runLeftDrive(100);
        drive.runRightDrive(-100);
    }
    drive.stop();
    pros::delay(500); */

    //odomTurnToPos(82.3, 35.0);
    runOdomTracking = true;
    odomTurnToHeading(38.0 * (PI/180.0));
    while(runChassisControl)
    {
        pros::delay(5);
    }
    runOdomTracking = false;

    pros::delay(100);

    drive.resetEncoders();
    while(drive.getLeftEncoderInches() > -4.0)
    {
        drive.runLeftDrive(-100);
        drive.runRightDrive(-100);
    }
    drive.stop();

    pros::delay(100);

    setTargetIntake(-800, 100);
    pros::delay(500);

    drive.resetEncoders();
    
    /*while(drive.getLeftEncoderInches() > -15.0)
    {
        drive.runLeftDrive(-100);
        drive.runRightDrive(-100);
    }
    drive.stop();
    pros::delay(750);*/
    driveSeconds(drive, 2000, -100);


    // drive.runRightDrive(-100);
    // pros::delay(750);





    // driveSeconds(drive, 100, -80);
    // driveSeconds(drive, 250, 50);
    
    // curEncoderValue = drive.getLeftEncoderInches();
    // while(drive.getLeftEncoderInches() < curEncoderValue + 4.0)
    // {
    //     drive.runLeftDrive(80);
    //     drive.runRightDrive(80);
    // }
    // drive.stop();
    
    // pros::delay(100);
   

    // curEncoderValue = drive.getEncoderInchesAverage();
    // while(drive.getEncoderInchesAverage() > curEncoderValue - 12.0)
    // {
    //     drive.runLeftDrive(75);
    //     drive.runRightDrive(-75);
    // }   
    // drive.stop();

    // pros::delay(100);

    // setTargetIntake(200 + 900, 100);

    // curEncoderValue = drive.getEncoderInchesAverage();
    // while(drive.getEncoderInchesAverage() < curEncoderValue + 10.0)
    // {
    //     drive.runLeftDrive(100);
    //     drive.runRightDrive(100);
    // }   
    // drive.stop();
    // curEncoderValue = drive.getEncoderInchesAverage();
    // setTargetIntake(2240 + 900, 100); 
    // setTargetArm(720.0, 100);
    // while(drive.getEncoderInchesAverage() > curEncoderValue - 3.0)
    // {
    //     drive.runLeftDrive(-100);
    //     drive.runRightDrive(-100);
    // }  
    // drive.stop();
    // curEncoderValue = drive.getEncoderInchesAverage();
    // while(drive.getEncoderInchesAverage() < curEncoderValue + 2.3)
    // {
    //     drive.runLeftDrive(40);
    //     drive.runRightDrive(40);
    // }  
    // drive.stop();
    // //718 just for margin
    // while(arm.getEncoderRaw() < 718.0)
    // {
    //     //driver.print(2,2,"%.2f", arm.getEncoderRaw());
    //     pros::delay(10);
    // }
    // // multiplied by 2 for torque cartirdge
    // setTargetStick(-115.2 * 2, 20);
    // pros::delay(2000);


    

    // curEncoderValue = drive.getEncoderInchesAverage();
    // setTargetIntake(1000, 100);
    // while(drive.getEncoderInchesAverage() > curEncoderValue - 5.0)
    // {
    //     drive.runLeftDrive(-100);
    //     drive.runRightDrive(-100);
    // }  
    // drive.stop();
    intakeTask.suspend();
    stickTask.suspend();
    armTask.suspend();
    pros::delay(3000);

}
