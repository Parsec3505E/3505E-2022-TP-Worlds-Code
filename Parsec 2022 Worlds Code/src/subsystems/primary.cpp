#include "main.h"

Primary::Primary()
{

    // Motors
    spinner = new pros::Motor(20);
    spinner->set_reversed(true);
    spinner->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    mogoClamp = new pros::ADIDigitalOut('A');

}

void Primary::stop()
{
    spinner->move(0);
}

void Primary::spin(int voltage)
{
    spinner->move(voltage);
}

void Primary::triggerMogoClamp(bool trigger)
{
    mogoClamp->set_value(trigger);
}

void Primary::setBrake()
{
    spinner->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void Primary::setCoast()
{
    spinner->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void Primary::setHold()
{
    spinner->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

double Primary::getIntakeVelocity()
{
    return (double)spinner->get_actual_velocity();
}

int Primary::getEncoderRaw()
{
    return spinner->get_position();
}

void Primary::resetEncoder()
{
    spinner->tare_position();
}