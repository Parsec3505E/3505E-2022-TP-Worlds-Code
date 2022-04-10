#include "main.h"

int intakeTicksToMove = 0;
int intakeVelToMove = 0;

int armTicksToMove = 0;
int armVelToMove = 0;

int stickTicksToMove = 0;
int stickVelToMove = 0;

void setTargetIntake(int ticks, int vel)
{
    intakeTicksToMove = ticks;
    intakeVelToMove = vel;
}

void setTargetStick(int ticks, int vel)
{
    stickTicksToMove = ticks;
    stickVelToMove = vel;
}

void setTargetArm(int ticks, int vel)
{
    armTicksToMove = ticks;
    armVelToMove = vel;
}

void moveIntakeFor(void* arg)
{
    Intake intake = ((intake_arg*)arg)->intake;
    intake.intakeMotor->move_absolute(intakeTicksToMove, intakeVelToMove);
}

void moveArmFor(void* arg)
{
    Arm arm = ((arm_arg*)arg)->arm;
    arm.armMotor->move_absolute(armTicksToMove, armVelToMove);
}

void moveStickFor(void* arg)
{
    Stick stick = ((stick_arg*)arg)->stick;
    stick.stickMotor->move_absolute(stickTicksToMove, stickVelToMove);
}