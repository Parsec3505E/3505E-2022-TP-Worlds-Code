#include "main.h"

Drivetrain::Drivetrain()
{

    // Motors
    rightFront = new pros::Motor(11);
    rightFront->set_reversed(true);
    rightFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    rightBack = new pros::Motor(6);
    rightBack->set_reversed(true);
    rightBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	leftFront = new pros::Motor(4);
    leftFront->set_reversed(false);
    leftFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	leftBack = new pros::Motor(17);
    leftBack->set_reversed(false);
    leftBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // Encoder Objects
    rightEncoder = new pros::ADIEncoder('A', 'B', true);
    leftEncoder = new pros::ADIEncoder('C', 'D');
    backEncoder = new pros::ADIEncoder('E', 'F');

    driveUltrasonic = pros::c::ext_adi_ultrasonic_init(2, 'A', 'B');

}

void Drivetrain::stop()
{
    rightFront->move(0);
    rightBack->move(0);
    leftFront->move(0);
    leftBack->move(0);
}

void Drivetrain::runRightDrive(int voltage)
{
    rightFront->move(voltage);
    rightBack->move(voltage);
}

void Drivetrain::runLeftDrive(int voltage)
{
    leftFront->move(voltage);
    leftBack->move(voltage);
}


void Drivetrain::setBrake()
{
    rightFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void Drivetrain::setCoast()
{
    rightFront->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightBack->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftFront->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftBack->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void Drivetrain::setHold()
{
    rightFront->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rightBack->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    leftFront->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    leftBack->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

double Drivetrain::getRightVelocity()
{
   return (double)(rightFront->get_actual_velocity() + rightBack->get_actual_velocity()) / 2;
}

double Drivetrain::getLeftVelocity()
{
   return (double)(leftFront->get_actual_velocity() + leftBack->get_actual_velocity()) / 2;
}

int Drivetrain::getRightEncoderRaw()
{
    return rightEncoder->get_value();
}

int Drivetrain::getLeftEncoderRaw()
{
    return leftEncoder->get_value();
}

int Drivetrain::getBackEncoderRaw()
{
    return backEncoder->get_value();
}

int Drivetrain::getAverageEncorderRaw()
{
    return (getRightEncoderRaw() + getLeftEncorderInches()) / 2;
}

double Drivetrain::getRightEncorderInches()
{
    return ticksToInches(getRightEncoderRaw());
}

double Drivetrain::getLeftEncorderInches()
{
    return ticksToInches(getLeftEncoderRaw());
}

double Drivetrain::getBackEncorderInches()
{
    return ticksToInches(getBackEncoderRaw());
}

double Drivetrain::getEncoderInchesAverage()
{
    return (getRightEncorderInches() + getLeftEncorderInches()) / 2;
}

void Drivetrain::resetEncoders()
{
    rightEncoder->reset();
    leftEncoder->reset();
    backEncoder->reset();
}

int Drivetrain::getDistance()
{
    return pros::c::ext_adi_ultrasonic_get(driveUltrasonic);
}








