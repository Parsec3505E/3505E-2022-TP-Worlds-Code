#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "main.h"

extern const double PI;

class Drivetrain{

    pros::Motor* rightFront;
    pros::Motor* leftFront;
    pros::Motor* rightBack;
    pros::Motor* leftBack;

    pros::ADIEncoder* rightEncoder;
    pros::ADIEncoder* leftEncoder;
    pros::ADIEncoder* backEncoder;

    // pros::c::ext_adi_encoder_t rightEncoder;
    // pros::c::ext_adi_encoder_t leftEncoder;
    // pros::c::ext_adi_encoder_t backEncoder;

    // pros::c::ext_adi_ultrasonic_t driveUltrasonic;

    public:
        Drivetrain();

        // MOTOR METHODS
        void stop();
        void runRightDrive(int voltage);
        void runLeftDrive(int voltage);

        void runRightDriveVelocity(int Velocity);
        void runLeftDriveVelocity(int Velocity);

        // MOTOR MODES
        void setCoast();
        void setBrake();
        void setHold();

        double getRightVelocity();
        double getLeftVelocity();


        // ENCODER METHODS
        int getRightEncoderRaw();
        int getLeftEncoderRaw();
        int getBackEncoderRaw();

        int getAverageEncoderRaw();

        double getRightEncoderInches();
        double getLeftEncoderInches();
        // double getBackEncoderInches();

        double getEncoderInchesAverage();

        double ticksToInches(int ticks);

        void resetEncoders();

        // ULTRASONIC METHODS

        // int getDistance();
};

#endif