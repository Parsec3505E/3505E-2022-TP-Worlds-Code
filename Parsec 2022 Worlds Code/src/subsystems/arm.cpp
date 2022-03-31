#include "main.h"

Arm::Arm()
{

    // Motors
    armMotor = new pros::Motor(11);
    armMotor->set_reversed(true);
    armMotor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

}

void Arm::stop()
{
    armMotor->move(0);
}

void Arm::runArm(int voltage)
{
    armMotor->move(voltage);
}

void Arm::runArmVelocity(int velocity)
{
    armMotor->move_velocity(velocity); 
}


void Arm::setBrake()
{
    armMotor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void Arm::setCoast()
{
    armMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

}

void Arm::setHold()
{
    armMotor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

}

double Arm::getArmVelocity()
{
   return (double)armMotor->get_actual_velocity();
}

int Arm::getEncoderRaw()
{
    return armMotor->get_position();
}

void Arm::resetEncoder()
{
    armMotor->tare_position();
}
