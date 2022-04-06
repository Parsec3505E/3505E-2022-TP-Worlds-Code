#include "main.h"

Drivetrain drivetrain = Drivetrain();

const double PI = 3.14159265358979323846;

//Radius of tracking wheels in inches
double WHEEL_RADIUS = 2.75;

//Starting angle (relative to field) (RADIANS)
double THETA_START = PI / 2; //imagine the field is a unit circle

double X_START = 115; 
double Y_START = 9;

//Distances of tracking wheels from tracking center (INCHES)
double rTrackRadius = 4.75;
double lTrackRadius = 4.75;
double bTrackRadius = 1.04;

//Calculated Values (every loop)
//Angles (DEGREES) *NEEDS TO BE CONVERTED TO RADIANS FOR MATH*
double lEncoderPose = 0;
double rEncoderPose = 0;
double bEncoderPose = 0;

double rPrevPose = 0;
double lPrevPose = 0;
double bPrevPose = 0;

//Distances traveled by tracking wheels each loop (INCHES)
double deltaDistR = 0;
double deltaDistL = 0;
double deltaDistB = 0;

//Distance summations (since last reset)
double totalDeltaDistL = 0;
double totalDeltaDistR = 0;

//The current angle of the bot (RADIANS)
double heading = THETA_START;
//The previous angle of the bot (RADIANS)
double prevHeading = THETA_START;

//The change in heading each loop (RADIANS)
double deltaHeading = 0;

//The changes in the X and Y positions (INCHES)
/*These are calculated on a local basis each loop,
then converted to global position changes */
double deltaXLocal = 0;
double deltaYLocal = 0;

//The X and Y offsets converted from their local frame to global frame (INCHES)
double deltaXGlobal = 0;
double deltaYGlobal = 0;

//The global position of the bot (INCHES)
double xPoseGlobal = X_START;
double yPoseGlobal = Y_START;

int poseTracking(){

    while(true){
        rEncoderPose = drivetrain.getRightEncoderRaw();
        lEncoderPose = drivetrain.getLeftEncoderRaw();
        bEncoderPose = drivetrain.getBackEncoderRaw();

        deltaDistL = ((lEncoderPose - lPrevPose) * PI / 180) * WHEEL_RADIUS;
        deltaDistR = ((rEncoderPose - rPrevPose) * PI / 180) * WHEEL_RADIUS;
        deltaDistB = ((bEncoderPose - bPrevPose) * PI / 180) * WHEEL_RADIUS;

        rEncoderPose = rPrevPose;
        lEncoderPose = lPrevPose;
        bEncoderPose = bPrevPose;

        totalDeltaDistL += deltaDistL;
        totalDeltaDistR += deltaDistR;

        heading = THETA_START - (deltaDistL - deltaDistR) / (lTrackRadius + rTrackRadius);
        deltaHeading = heading - prevHeading;

        prevHeading = heading;

        if(deltaHeading == 0){
            deltaXLocal = deltaDistB;
            deltaYLocal = (deltaDistL + deltaDistR)/2;
        }else{
            deltaYLocal = 2*sin(deltaHeading/2) * (deltaDistR / deltaHeading) - rTrackRadius;
            deltaXLocal = 2*sin(deltaHeading/2) * (deltaDistB / deltaHeading) + bTrackRadius;
        }

        deltaXGlobal = sin(deltaHeading/2)*deltaXLocal + cos(deltaHeading/2)*deltaYLocal;
        deltaYGlobal = cos(deltaHeading/2)*deltaXLocal - sin(deltaHeading/2)*deltaYLocal;

        while(heading >= 2 * PI) {
            heading -= 2 * PI;
        }

        while(heading < 0) {
            heading += 2 * PI;
        }

        //Update global positions
        xPoseGlobal += deltaXGlobal;
        yPoseGlobal += deltaYGlobal;

        pros::delay(10);

    }

 return 1;

}