#include "main.h"

const double PI = 3.14159265358979323846;

Drivetrain::Drivetrain()
{

    // Motors
    rightFront = new pros::Motor(11);
    rightFront->set_reversed(true);
    rightFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    rightBack = new pros::Motor(10);
    rightBack->set_reversed(true);
    rightBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	leftFront = new pros::Motor(3);
    leftFront->set_reversed(false);
    leftFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	leftBack = new pros::Motor(6);
    leftBack->set_reversed(false);
    leftBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // Encoder Objects
    rightEncoder = new pros::ADIEncoder({{15, 'a', 'b'}});
    leftEncoder = new pros::ADIEncoder({{15, 'c', 'd'}});
    backEncoder = new pros::ADIEncoder({{15, 'e', 'f'}});

    // rightEncoder = pros::c::ext_adi_encoder_init(13, 'A', 'B', true);
    // leftEncoder = pros::c::ext_adi_encoder_init(13, 'C', 'D', false);
    // backEncoder = pros::c::ext_adi_encoder_init(13, 'E', 'F', false);

    // driveUltrasonic = pros::c::ext_adi_ultrasonic_init(2, 'A', 'B');

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

void Drivetrain::runRightDriveVelocity(int velocity)
{
    rightFront->move_velocity(velocity);
    rightBack->move_velocity(velocity);
}

void Drivetrain::runLeftDriveVelocity(int velocity)
{
    leftFront->move_velocity(velocity);
    leftBack->move_velocity(velocity);
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
   return (double)(rightFront->get_actual_velocity() + rightBack->get_actual_velocity()) / 2.0;
}

double Drivetrain::getLeftVelocity()
{
   return (double)(leftFront->get_actual_velocity() + leftBack->get_actual_velocity()) / 2.0;
}

int Drivetrain::getRightEncoderRaw()
{
    return (-1 * rightEncoder->get_value());
}

int Drivetrain::getLeftEncoderRaw()
{
    return leftEncoder->get_value();
}

int Drivetrain::getBackEncoderRaw()
{
    return backEncoder->get_value();
}

int Drivetrain::getAverageEncoderRaw()
{
    return (getRightEncoderRaw() + getLeftEncoderRaw()) / 2.0;
}

double Drivetrain::getRightEncoderInches()
{
    return ticksToInches(getRightEncoderRaw());
}

double Drivetrain::getLeftEncoderInches()
{
    return ticksToInches(getLeftEncoderRaw());
}

// double Drivetrain::getBackEncoderInches()
// {
//     return ticksToInches(getBackEncoderRaw());
// }

double Drivetrain::getEncoderInchesAverage()
{
    return (getRightEncoderInches() + getLeftEncoderInches()) / 2.0;
}

void Drivetrain::resetEncoders()
{
    pros::delay(100);
    while(getEncoderInchesAverage() > 10 || getEncoderInchesAverage() < -10)
    {
        rightEncoder->reset();
        leftEncoder->reset();
        backEncoder->reset();
        pros::delay(100);
    }
    rightFront->tare_position();
    rightBack->tare_position();
    leftFront->tare_position();
    leftBack->tare_position();
}

double Drivetrain::ticksToInches(int ticks)
{
    return (((double)ticks) * (2.75*PI)/360.0);
}

// int Drivetrain::getDistance()
// {
//     return pros::c::ext_adi_ultrasonic_get(driveUltrasonic);
// }








