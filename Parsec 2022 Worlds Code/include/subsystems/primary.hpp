#ifndef PRIMARY_HPP
#define PRIMARY_HPP

#include "main.h"

class Primary{

    pros::Motor* spinner;
    pros::ADIDigitalOut* mogoClamp;

    // pros::c::ext_adi_ultrasonic_t rightUltrasonic;
    // pros::c::ext_adi_ultrasonic_t leftUltrasonic;


    public:
        Primary();

        // MOTOR METHODS
        void stop();
        void spin(int voltage);

        // PNUEMATIC METHODS
        void triggerMogoClamp(bool trigger);
    
        // MOTOR MODES
        void setCoast();
        void setBrake();
        void setHold();

        double getIntakeVelocity();

        // ENCODER METHODS
        int getEncoderRaw();

        void resetEncoder();

        // ULTRASONIC METHODS
        // int getDistance(pros::c::ext_adi_ultrasonic_t ult);


};

#endif