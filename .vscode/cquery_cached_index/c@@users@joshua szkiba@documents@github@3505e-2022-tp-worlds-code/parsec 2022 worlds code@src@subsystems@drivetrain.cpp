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
    // rightEncoder = new pros::ADIEncoder('A', 'B', true);
    // leftEncoder = new pros::ADIEncoder('C', 'D');
    // backEncoder = new pros::ADIEncoder('E', 'F');

    rightEncoder = pros::c::ext_adi_encoder_init(2, 'A', 'B', true);
    leftEncoder = pros::c::ext_adi_encoder_init(2, 'A', 'B', false);
    backEncoder = pros::c::ext_adi_encoder_init(2, 'A', 'B', false);

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
    return pros::c::ext_adi_encoder_get(rightEncoder);
}

int Drivetrain::getLeftEncoderRaw()
{
    return pros::c::ext_adi_encoder_get(leftEncoder);
}

int Drivetrain::getBackEncoderRaw()
{
    return pros::c::ext_adi_encoder_get(backEncoder);
}

int Drivetrain::getAverageEncorderRaw()
{
    return (getRightEncoderRaw() + getLeftEncoderRaw()) / 2;
}

// double Drivetrain::getRightEncoderInches()
// {
//     return ticksToInches(getRightEncoderRaw());
// }

// double Drivetrain::getLeftEncoderInches()
// {
//     return ticksToInches(getLeftEncoderRaw());
// }

// double Drivetrain::getBackEncoderInches()
// {
//     return ticksToInches(getBackEncoderRaw());
// }

// double Drivetrain::getEncoderInchesAverage()
// {
//     return (getRightEncoderInches() + getLeftEncoderInches()) / 2;
// }

void Drivetrain::resetEncoders()
{
    pros::c::ext_adi_encoder_reset(rightEncoder);
    pros::c::ext_adi_encoder_reset(leftEncoder); 
    pros::c::ext_adi_encoder_reset(backEncoder);
}

int Drivetrain::getDistance()
{
    return pros::c::ext_adi_ultrasonic_get(driveUltrasonic);
}








