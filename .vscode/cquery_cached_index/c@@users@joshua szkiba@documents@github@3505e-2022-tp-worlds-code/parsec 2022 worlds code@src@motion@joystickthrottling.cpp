#include "main.h"

double joystickThrottle(double joystick_val)
{
    return pow(joystick_val , 3) / pow(100, 2);
}