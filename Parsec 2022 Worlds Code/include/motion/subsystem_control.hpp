#ifndef SUBSYSTEM_CONTROL_HPP
#define SUBSYSTEM_CONTROL_HPP

#include "main.h"

extern int intakeTicksToMove;
extern int intakeVelToMove;

extern int armTicksToMove;
extern int armVelToMove;

extern int stickTicksToMove;
extern int stickVelToMove;

void setTargetIntake(int ticks, int vel);

void setTargetStick(int ticks, int vel);

void setTargetArm(int ticks, int vel);

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