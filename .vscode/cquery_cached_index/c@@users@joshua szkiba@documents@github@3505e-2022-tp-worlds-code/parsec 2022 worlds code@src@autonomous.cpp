#include "autonomous.hpp"

const double TALL_NEUTRAL_X = 0;
const double TALL_NEUTRAL_Y = 0;


const double RIGHT_ALLIANCE_X = 0;
const double LEFT_ALLIANCE_Y = 0;


void highNeutralWinPoint()
{
    drivetrain.resetEncoders();
    driveTo(TALL_NEUTRAL_X, TALL_NEUTRAL_Y, PI / 2);
}