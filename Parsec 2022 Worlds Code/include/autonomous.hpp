#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP

#include "motion/chassis_control.hpp"

extern const double TALL_NEUTRAL_X;
extern const double TALL_NEUTRAL_Y;

extern const double RIGHT_ALLIANCE_X;
extern const double LEFT_ALLIANCE_Y;

void odomPrint();

void skills();

void endAllTasks();

void highNeutralWinPoint();

void leftSideAuton();

#endif