#include "main.h"

Arm::Arm()
{

    // Motors
    armMotor = new pros::Motor(4);
    armMotor->set_reversed(true);
    armMotor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    armMotor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

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

double Arm::getEncoderRaw()
{
    return armMotor->get_position();
}

void Arm::resetEncoder()
{
    armMotor->tare_position();
}
