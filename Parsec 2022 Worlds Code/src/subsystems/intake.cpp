#include "main.h"

Intake::Intake()
{

    // Motors
    intakeMotor = new pros::Motor(19);
    intakeMotor->set_reversed(true);
    intakeMotor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    ptoLock = new pros::ADIDigitalOut('B');

}

void Intake::stop()
{
    intakeMotor->move(0);
}

void Intake::intake(int voltage)
{
    intakeMotor->move(voltage);
}

void Intake::outtake(int voltage)
{
    intakeMotor->move(voltage * -1);
}

void Intake::triggerPTOLock(bool trigger)
{
    ptoLock->set_value(trigger);
}

void Intake::setBrake()
{
    intakeMotor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void Intake::setCoast()
{
    intakeMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

double Intake::getIntakeVelocity()
{
    return (double)intakeMotor->get_actual_velocity();
}

int Intake::getEncoderRaw()
{
    return intakeMotor->get_position();
}

void Intake::resetEncoder()
{
    intakeMotor->tare_position();
}



