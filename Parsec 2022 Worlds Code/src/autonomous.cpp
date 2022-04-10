#include "autonomous.hpp"

const double TALL_NEUTRAL_X = 0;
const double TALL_NEUTRAL_Y = 0;

//SAVE ME
void waitWhile(bool someBool)
{
    while(someBool)
    {
        pros::delay(10);
    }
}


const double RIGHT_ALLIANCE_X = 0;
const double LEFT_ALLIANCE_Y = 0;


void highNeutralWinPoint()
{
    Drivetrain drive = Drivetrain();
    Intake intake = Intake();
    Arm arm = Arm();
    Stick stick = Stick();

    //Reset Encoders
    drive.resetEncoders();
    intake.resetEncoder();
    arm.resetEncoder();
    stick.resetEncoder();

    drive_arg* track_task_arg = new drive_arg;
    track_task_arg->drivetrain = drive;

    intake_arg* intake_task_arg = new intake_arg;
    intake_task_arg->intake = intake;

    arm_arg* arm_task_arg = new arm_arg;
    arm_task_arg->arm = arm;

    stick_arg* stick_task_arg = new stick_arg;
    stick_task_arg->stick = stick;

    pros::Task odomTracking(poseTracking, track_task_arg);
    pros::Task chassisControl(odomChassisControl, track_task_arg);
    pros::Task PIDChassiControl(PIDControl, track_task_arg);
    pros::Task intakeTask(moveIntakeFor, intake_task_arg);
    pros::Task armTask(moveArmFor, arm_task_arg);
    pros::Task stickTask(moveStickFor, stick_task_arg);

    //Change Numbers
    setTargetIntake(-300, 100);
    driveToPID(54);
    waitWhile(runPIDChassisControl);
    driveToPID(27);
    waitWhile(runPIDChassisControl);
    odomDriveTo(140.7 - 9, 0 + 9, 0);
    waitWhile(runChassisControl);
    driveSeconds(drive, 1, 100);
    odomDriveTo(129.2, 35 - 10, 0);
    waitWhile(runChassisControl);

}
