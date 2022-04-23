#include "main.h"

int intakeTicksToMove = 0;
int intakeVelToMove = 0;

double armTicksToMove = 0.0;
int armVelToMove = 0;

double stickTicksToMove = 0.0;
int stickVelToMove = 0;

void setTargetIntake(int ticks, int vel)
{
    intakeTicksToMove = ticks;
    intakeVelToMove = vel;
}

void setTargetStick(double degrees, int vel)
{
    stickTicksToMove = degrees;
    stickVelToMove = vel;
}

void setTargetArm(double degrees, int vel)
{
    armTicksToMove = degrees;
    armVelToMove = vel;
}

void moveIntakeFor(void* arg)
{
    Intake intake = ((intake_arg*)arg)->intake;
    while(true)
    {
        intake.intakeMotor->move_absolute(intakeTicksToMove, intakeVelToMove);
    }
    pros::delay(20);
}

void moveArmFor(void* arg)
{
    Arm arm = ((arm_arg*)arg)->arm;
    while(true)
    {
        arm.armMotor->move_absolute(armTicksToMove, armVelToMove);
    }
    pros::delay(20);
}

void moveStickFor(void* arg)
{
    Stick stick = ((stick_arg*)arg)->stick;
    while(true)
    {
        stick.stickMotor->move_absolute(stickTicksToMove, stickVelToMove);
    }
    pros::delay(20);
}