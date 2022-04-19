#include "main.h"

//Drivetrain drivetrain = Drivetrain();

//const double PI = 3.14159265358979323846;

//Diameter of tracking wheels in inches
double WHEEL_DIAMETER = 2.75;

//Starting angle (relative to field) (RADIANS)
double THETA_START = 0.5;

//Starting Position For Auton
double X_START = 104.0; 
double Y_START = 8.75;

//Distances of tracking wheels from tracking center (INCHES)
double rTrackRadius = 4.75;
double lTrackRadius = 4.75;
//Might need to be negative
double bTrackRadius = 1.04;

//Calculated Values (every loop)
//Angles (DEGREES) *NEEDS TO BE CONVERTED TO RADIANS FOR MATH*
double lEncoderPose = 0.0;
double rEncoderPose = 0.0;
double bEncoderPose = 0.0;

double rPrevPose = 0.0;
double lPrevPose = 0.0;
double bPrevPose = 0.0;

//Distances traveled by tracking wheels each loop (INCHES)
double deltaDistR = 0.0;
double deltaDistL = 0.0;
double deltaDistB = 0.0;

//Distance summations (since last reset)
double totalDeltaDistL = 0.0;
double totalDeltaDistR = 0.0;

//The current angle of the bot (RADIANS)
double heading = THETA_START;
double heading_raw = THETA_START;
//The previous angle of the bot (RADIANS)
double prevHeading_raw = THETA_START;

//The change in heading each loop (RADIANS)
double deltaHeading = 0.0;

//The changes in the X and Y positions (INCHES)
/*These are calculated on a local basis each loop,
then converted to global position changes */
double deltaXLocal = 0.0;
double deltaYLocal = 0.0;

//The X and Y offsets converted from their local frame to global frame (INCHES)
double deltaXGlobal = 0.0;
double deltaYGlobal = 0.0;

//The global position of the bot (INCHES)
double xPoseGlobal = X_START;
double yPoseGlobal = Y_START;

//Do tracking boolean
bool runOdomTracking = false;

void resetTracking()
{
    lEncoderPose = 0.0;
    rEncoderPose = 0.0;
    bEncoderPose = 0.0;
    rPrevPose = 0.0;
    lPrevPose = 0.0;
    bPrevPose = 0.0;
    deltaDistR = 0.0;
    deltaDistL = 0.0;
    deltaDistB = 0.0;
    totalDeltaDistL = 0.0;
    totalDeltaDistR = 0.0;
    deltaHeading = 0.0;
    deltaXLocal = 0.0;
    deltaYLocal = 0.0;
    deltaXGlobal = 0.0;
    deltaYGlobal = 0.0;
    heading_raw = 0.0;
    heading = 0.0;
    xPoseGlobal = 0.0;
    yPoseGlobal = 0.0;
    prevHeading_raw = 0.0;
}

void poseTracking(void* arg){
    Drivetrain drive_temp = ((drive_arg*)arg)->drivetrain;
    pros::Controller driver(pros::E_CONTROLLER_MASTER);
    while(true){
        if(runOdomTracking)
        {
            rEncoderPose = (double)drive_temp.getRightEncoderRaw();
            lEncoderPose = (double)drive_temp.getLeftEncoderRaw();
            bEncoderPose = (double)drive_temp.getBackEncoderRaw();

            deltaDistL = ((lEncoderPose - lPrevPose)/360.0) * PI * WHEEL_DIAMETER;
            deltaDistR = ((rEncoderPose - rPrevPose)/360.0) * PI * WHEEL_DIAMETER;
            deltaDistB = ((bEncoderPose - bPrevPose)/360.0) * PI * WHEEL_DIAMETER;

            totalDeltaDistL += deltaDistL;
            totalDeltaDistR += deltaDistR;

            heading_raw = (((lEncoderPose - rEncoderPose)/360.0) * PI * WHEEL_DIAMETER) / (lTrackRadius + rTrackRadius);

            deltaHeading = heading_raw - prevHeading_raw;

            if(deltaHeading == 0.0 ){
                deltaXLocal = deltaDistB;
                deltaYLocal = (deltaDistL + deltaDistR)/2.0;
            }else{
                deltaYLocal = 2.0*sin(deltaHeading/2.0) * ((deltaDistL / deltaHeading) - lTrackRadius);
                deltaXLocal = 2.0*sin(deltaHeading/2.0) * ((deltaDistB / deltaHeading) + bTrackRadius);
            }
            //When encoder moves left it should be negative

            deltaXGlobal = cos(heading_raw + (deltaHeading/2.0))*deltaXLocal + sin(heading_raw + (deltaHeading/2.0))*deltaYLocal;
            deltaYGlobal = -sin(heading_raw + (deltaHeading/2.0))*deltaXLocal + cos(heading_raw + (deltaHeading/2.0))*deltaYLocal;

            /*
            while(heading >= 2 * PI) {
                heading -= 2 * PI;
            }

            
            while(heading < 0) {
                heading += 2 * PI;
            }
            */

            //Update global positions
            xPoseGlobal += deltaXGlobal;
            yPoseGlobal += deltaYGlobal;

            rPrevPose = rEncoderPose;
            lPrevPose = lEncoderPose;
            bPrevPose = bEncoderPose;
            prevHeading_raw = heading_raw;

            heading = heading_raw;

            while (heading > 2.0 * PI)
            {
                heading -= 2.0 * PI;
            }
            while (heading < 0)
            {
                heading += 2.0 * PI;
            }

            //pros::Controller driver(pros::E_CONTROLLER_MASTER);
            //driver.print(2,2,"%.1f, %.1f, %.1f", xPoseGlobal, yPoseGlobal, heading_raw);
        }
        pros::delay(10);

    }

}