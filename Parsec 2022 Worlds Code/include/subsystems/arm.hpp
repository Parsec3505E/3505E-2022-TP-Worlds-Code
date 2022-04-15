#ifndef ARM_HPP
#define ARM_HPP

#include "main.h"

class Arm{

    public:
        pros::Motor* armMotor;
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
        double getEncoderRaw();

        void resetEncoder();

};

#endif