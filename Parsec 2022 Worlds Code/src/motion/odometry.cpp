#include "main.h"

//Drivetrain drivetrain = Drivetrain();

//const double PI = 3.14159265358979323846;

//Diameter of tracking wheels in inches
double WHEEL_RADIUS = 2.75;

//Starting angle (relative to field) (RADIANS)
double THETA_START = 0; //imagine the field is a unit circle

double X_START = 0; 
double Y_START = 0;

//Distances of tracking wheels from tracking center (INCHES)
double rTrackRadius = 4.75;
double lTrackRadius = 4.75;
//Might need to be negative
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

void poseTracking(void* arg){
    Drivetrain drive_temp = ((drive_arg*)arg)->drivetrain;
    while(true){
        rEncoderPose = drive_temp.getRightEncoderRaw();
        lEncoderPose = drive_temp.getLeftEncoderRaw();
        bEncoderPose = drive_temp.getBackEncoderRaw();

        deltaDistL = ((lEncoderPose - lPrevPose)/360) * PI * WHEEL_RADIUS;
        deltaDistR = ((rEncoderPose - rPrevPose)/360) * PI * WHEEL_RADIUS;
        deltaDistB = ((bEncoderPose - bPrevPose)/360) * PI * WHEEL_RADIUS;

        totalDeltaDistL += deltaDistL;
        totalDeltaDistR += deltaDistR;

        deltaHeading = (deltaDistL - deltaDistR) / (lTrackRadius + rTrackRadius);

        heading += deltaHeading;

        if(heading >= 2 * PI)
        {
            heading -= 2 * PI;
        }
        else if (heading <= -2 * PI)
        {
            heading += 2 * PI;
        }

        if(deltaHeading == 0 ){
            deltaXLocal = deltaDistB;
            deltaYLocal = (deltaDistL + deltaDistR)/2;
        }else{
            deltaYLocal = 2*sin(deltaHeading/2) * ((deltaDistR / deltaHeading) - rTrackRadius);
            deltaXLocal = 2*sin(deltaHeading/2) * ((deltaDistB / deltaHeading) + bTrackRadius);
        }
        //When encoder moves left it should be negative

        deltaXGlobal = cos(heading + (deltaHeading/2))*deltaXLocal + sin(heading + (deltaHeading/2))*deltaYLocal;
        deltaYGlobal = - sin(heading + (deltaHeading/2))*deltaXLocal + cos(heading + (deltaHeading/2))*deltaYLocal;

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

        pros::Controller driver(pros::E_CONTROLLER_MASTER);
        driver.print(2,2,"%.1f %.1f %.1f\n", xPoseGlobal, yPoseGlobal, heading);

        pros::delay(20);

    }

}