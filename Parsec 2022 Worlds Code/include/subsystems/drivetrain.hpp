#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "main.h"

class Drivetrain{

    pros::Motor* rightFront;
    pros::Motor* leftFront;
    pros::Motor* rightBack;
    pros::Motor* leftBack;

    pros::ADIEncoder* rightEncoder;
    pros::ADIEncoder* leftEncoder;
    pros::ADIEncoder* backEncoder;

    protected:
        Drivetrain();

        // MOTOR METHODS
        void stop();
        void runRightDrive(int voltage);
        void runLeftDrive(int voltage);

        void runRightDriveVelocity(int Velocity);
        void runLeftDriveVelocity(int Velocity);
    
    public:
        // MOTOR MODES
        void setCoast();
        void setBrake();
        void setHold();

        double getRightVelocity();
        double getLeftVelocity();


        // ENCODER METHODS
        int getRightEncoderRaw();
        int getLeftEncorderRaw();
        int getBackEncorderRaw();

        int getAverageEncorderRaw();

        double getRightEncorderInches();
        double getLeftEncorderInches();
        double getBackEncorderInches();

        double getEncoderInchesAverage();

        double ticksToInches(int ticks);
        double inchesToTicks(int inches);

        void resetEncoders();

        // JOYSTICK THROTTLING 
        double exponentialDrive(int joystickVal);
};

#endif