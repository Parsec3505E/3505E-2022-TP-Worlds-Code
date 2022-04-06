#ifndef INTAKE_HPP
#define INTAKE_HPP

#include "main.h"

class Intake{

    pros::Motor* intakeMotor;
    pros::ADIDigitalOut* ptoLock;

    public:
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

};

#endif