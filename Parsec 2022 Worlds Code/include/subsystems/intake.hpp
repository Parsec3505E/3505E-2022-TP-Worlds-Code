#ifndef INTAKE_HPP
#define INTAKE_HPP

#include "main.h"

class Intake{

    public:
        pros::ADIDigitalOut* ptoLock;
        pros::Motor* intakeMotor;
        Intake();

        // MOTOR METHODS
        void stop();
        void intake(int voltage);
        void outtake(int voltage);

        // PNUEMATIC METHODS
        void triggerPTOLock(bool trigger);
    
        // MOTOR MODES
        void setCoast();
        void setBrake();

        double getIntakeVelocity();

        // ENCODER METHODS
        int getEncoderRaw();

        void resetEncoder();

        //AUTON METHODS
        void moveFor(void* arg);

};

#endif