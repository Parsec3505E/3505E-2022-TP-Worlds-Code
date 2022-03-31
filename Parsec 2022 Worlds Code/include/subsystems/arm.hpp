#ifndef ARM_HPP
#define ARM_HPP

#include "main.h"

class Arm{

    pros::Motor* armMotor;

    public:
        Arm();

        // MOTOR METHODS
        void stop();
        void runArm(int voltage);

        void runArmVelocity(int Velocity);
    
        // MOTOR MODES
        void setCoast();
        void setBrake();
        void setHold();

        double getArmVelocity();


        // ENCODER METHODS
        int getEncoderRaw();

        void resetEncoder();

};

#endif