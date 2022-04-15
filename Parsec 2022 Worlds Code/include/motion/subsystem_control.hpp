#ifndef SUBSYSTEM_CONTROL_HPP
#define SUBSYSTEM_CONTROL_HPP

#include "main.h"

extern int intakeTicksToMove;
extern int intakeVelToMove;

extern double armTicksToMove;
extern int armVelToMove;

extern double stickTicksToMove;
extern int stickVelToMove;

void setTargetIntake(int ticks, int vel);

void setTargetStick(double degrees, int vel);

void setTargetArm(double degrees, int vel);

void moveIntakeFor(void* arg);

void moveArmFor(void* arg);

void moveStickFor(void* arg);

//Structs for tasks
typedef struct{
	Intake intake;
} intake_arg;

typedef struct{
	Arm arm;
} arm_arg;

typedef struct{
	Stick stick;
} stick_arg;


#endif