#ifndef Stick_HPP
#define Stick_HPP

#include "main.h"

class Stick{

    public:
        pros::Motor* stickMotor;
        Stick();

        // MOTOR METHODS
        void stop();
        void runStick(int voltage);

        void runStickVelocity(int Velocity);
    
        // MOTOR MODES
        void setCoast();
        void setBrake();
        void setHold();

        double getStickVelocity();


        // ENCODER METHODS
        double getEncoderRaw();

        void resetEncoder();

};

#endif