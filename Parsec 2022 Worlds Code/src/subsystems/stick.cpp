#include "main.h"

Stick::Stick()
{

    // Motors
    stickMotor = new pros::Motor(7);
    stickMotor->set_reversed(true);
    stickMotor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

}

void Stick::stop()
{
    stickMotor->move(0);
}

void Stick::runStick(int voltage)
{
    stickMotor->move(voltage);
}

void Stick::runStickVelocity(int velocity)
{
    stickMotor->move_velocity(velocity); 
}


void Stick::setBrake()
{
    stickMotor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void Stick::setCoast()
{
    stickMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

}

void Stick::setHold()
{
    stickMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

}

double Stick::getStickVelocity()
{
   return (double)stickMotor->get_actual_velocity();
}

int Stick::getEncoderRaw()
{
    return stickMotor->get_position();
}

void Stick::resetEncoder()
{
    stickMotor->tare_position();
}
