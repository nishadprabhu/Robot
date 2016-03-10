


//Including FEH libraries
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <math.h>
#include <FEHServo.h>
#include "locations.h"
//Defining states for following lines
#define ON_LINE 2.9
#define CENTER 0
#define LEFT -1
#define FAR_LEFT -2
#define RIGHT 1
#define FAR_RIGHT 2
#define OFF_LINE 4
//Defining constants to convert counts to inches or degrees
#define COUNTS_PER_INCH 33.74
#define LEFT_COUNTS_PER_DEGREE 1.99
#define RIGHT_COUNTS_PER_DEGREE 1.99
//Define thresholds for line following/start light
#define START_LIGHT_ON 1.5
#define BLUE_LIGHT_ON 1
//Tuning constant
#define TUNING_CONSTANT 0.07
//PI
# define M_PI           3.14159265358979323846

//Declarations for encoders & motors
ButtonBoard buttons(FEHIO::Bank3);
DigitalEncoder right_encoder(FEHIO::P0_1);
DigitalEncoder left_encoder(FEHIO::P0_0);
FEHMotor right_motor(FEHMotor::Motor2,12.0);
FEHMotor left_motor(FEHMotor::Motor3,12.0);
FEHServo arm(FEHServo::Servo0);

AnalogInputPin right(FEHIO::P1_2);
AnalogInputPin middle(FEHIO::P1_4);
AnalogInputPin left(FEHIO::P1_6);
AnalogInputPin cds(FEHIO::P1_0);

DigitalInputPin frontLeftBump(FEHIO::P2_0);
DigitalInputPin frontRightBump(FEHIO::P2_1);
DigitalInputPin backLeftBump(FEHIO::P2_3);
DigitalInputPin backRightBump(FEHIO::P2_4);
/** move_forward
    Moves the robot forward

    @param percent Motor percent
    @param inches Distance robot needs to travel

*/
void move_forward(int percent, float inches) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    float counts = inches*COUNTS_PER_INCH;
    //Set both motors to desired percent
    right_motor.SetPercent(percent+2);
    left_motor.SetPercent(percent);
    int mp = percent;

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts) {
        mp = TUNING_CONSTANT*(left_encoder.Counts()-right_encoder.Counts())+(percent);
        right_motor.SetPercent(mp);
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/** move_forward_timed
    Moves the robot forward, stopping when a certain time is reached or a distance is met

    @param percent Motor percent
    @param inches Distance robot needs to travel
    @param time Time robot should be moving (in seconds)

*/
void move_forward_timed(int percent, float inches, int time) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    float counts = inches*COUNTS_PER_INCH;
    //Set both motors to desired percent
    right_motor.SetPercent(percent+1);
    left_motor.SetPercent(percent);
    int mp = percent;

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    int start_time = TimeNow();
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts && TimeNow() - start_time < time);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/** move_backwards
    Moves the robot backwards

    @param percent Motor percent
    @param inches Distance robot needs to travel

*/
void move_backwards(int percent, float inches) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    float counts = inches*COUNTS_PER_INCH;
    //Set both motors to desired percent
    right_motor.SetPercent(-1*percent);
    left_motor.SetPercent(-1*percent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors
     while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}
/** move_backwards_timed
    Moves the robot backwards, stopping when a certain time is reached or a distance is met

    @param percent Motor percent
    @param inches Distance robot needs to travel
    @param time Time robot should be moving (in seconds)

*/
void move_backwards_timed(int percent, float inches, int time) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    float counts = inches*COUNTS_PER_INCH;
    //Set both motors to desired percent
    right_motor.SetPercent(-1 * percent);
    left_motor.SetPercent(-1 * percent);
    int mp = percent;

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    int start_time = TimeNow();
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts && TimeNow() - start_time < time) {

    }


    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/** driveToWall
    Moves the robot forward, stopping when it hits a wall

    @param percent Motor percent

*/
void driveToWall(int percent) {

    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);
    int mp = percent+5;
    while(frontLeftBump.Value() || frontRightBump.Value()) {
        mp = TUNING_CONSTANT*(left_encoder.Counts()-right_encoder.Counts())+(percent);
        right_motor.SetPercent(mp);
        LCD.WriteLine(frontLeftBump.Value());
        LCD.WriteLine(frontRightBump.Value());
        if(!frontRightBump.Value()) {
            left_motor.SetPercent(percent+5);
            right_motor.SetPercent(percent-10);
        }
        else if(!frontLeftBump.Value()) {
            right_motor.SetPercent(percent+5);
            left_motor.SetPercent(percent-10);
        }
    }
    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/** followLine
    Makes the robot follow a line.

    @param speed Motor percent

*/
void followLine(float speed) {
        int leftValue, rightValue, midValue, state;
        while(frontLeftBump.Value() || frontRightBump.Value())
        {
            leftValue = left.Value();
            rightValue = right.Value();
            midValue = middle.Value();

            if(midValue <= ON_LINE && rightValue >= ON_LINE && leftValue >= ON_LINE) {
                state = CENTER;
            }
            else if(midValue <= ON_LINE && rightValue<= ON_LINE && leftValue >= ON_LINE) {
                state = LEFT;
            }
            else if(midValue >= ON_LINE && rightValue<= ON_LINE && leftValue >= ON_LINE) {
                state = FAR_LEFT;
            }
            else if(midValue <= ON_LINE && rightValue>= ON_LINE && leftValue <= ON_LINE) {
                state = RIGHT;
            }
            else if(midValue >= ON_LINE && rightValue>= ON_LINE && leftValue <= ON_LINE) {
                state = FAR_RIGHT;
            }
            else {
                state = OFF_LINE;
            }

            switch(state) {
                case CENTER:
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(speed);
                    break;
                case LEFT:
                    left_motor.SetPercent(1.5 *speed);
                    right_motor.SetPercent(.5 *speed);
                    break;
                case FAR_LEFT:
                    left_motor.SetPercent(2 *speed);
                    right_motor.SetPercent(.5*speed);
                    break;
                case RIGHT:
                    left_motor.SetPercent(.75*speed);
                    right_motor.SetPercent(speed);
                    break;
                case FAR_RIGHT:
                    left_motor.SetPercent(.5*speed);
                    right_motor.SetPercent(speed);
                    break;
                case OFF_LINE:
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(-1 * speed);
                    break;
            }
        }

}

/** turn_left
    Turns the robot to the left for a certain amount of degrees

    @param percent Motor percent
    @param degrees Amount for robot to turn

*/
void turn_left(int percent, float degrees) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    float counts = degrees * LEFT_COUNTS_PER_DEGREE;
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-1 * percent);
    int mp = percent;
    while((right_encoder.Counts() + left_encoder.Counts())/2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/** turn_right
    Turns the robot to the right for a certain amount of degrees

    @param percent Motor percent
    @param degrees Amount for robot to turn

*/
void turn_right(int percent, float degrees) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    float counts = degrees * RIGHT_COUNTS_PER_DEGREE;
    right_motor.SetPercent(-1 * percent);
    left_motor.SetPercent(percent);
    while((right_encoder.Counts() + left_encoder.Counts())/2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}
/** angleBetween
    Gets the smaller angle between two unit vectors

    @param degree1 Degree of first vector
    @param degree2 Degree of second vector
    @return The angle between the two vectors (angle < 180)

*/
float angleBetween(float degree1, float degree2) {
    float vect1x = cos(degree1 * M_PI/180);
    float vect1y = sin(degree1 * M_PI/180);
    float vect2x = cos(degree2 * M_PI/180);
    float vect2y = sin(degree2 * M_PI/180);
    //get dot product of vectors
    float dot = vect1x*vect2x + vect1y*vect2y;
    //use dot product definition to get angle between
    return acos(dot) * 180/M_PI;
}

/** faceDegree
    Uses RPS to face the robot to a certain degree
    @param degree Degree robot should face


*/
void faceDegree(float degree) {
    if(RPS.Heading() < 0) {
        return;
    }
    float headingToZero = 0;
    float degreeToZero = degree - RPS.Heading();
    if(degreeToZero < 0) {
        degreeToZero+=360;
    }
    float deltaTheta = angleBetween(headingToZero, degreeToZero);
    while(deltaTheta > 0.7) {
        LCD.WriteLine(RPS.Heading());
        headingToZero = 0;
        degreeToZero = degree-RPS.Heading();
        if(degreeToZero < 0) {
            degreeToZero+=360;
        }
        deltaTheta = angleBetween(headingToZero, degreeToZero);
        if(degreeToZero > 180) {
               turn_right(10,0.5);
               Sleep(100);

        }
        else {
               turn_left(10,0.5);
               Sleep(100);
            }
        }



 }


/** check_x_plus
    Moves the robot to a certain x coordinate while it is facing the positive x direction
    @param x_coordinate The coordinate the robot should go

*/

bool check_x_plus(float x_coordinate) //using RPS while robot is in the +x direction
{
    bool condition = true;
    //check whether the robot is within an acceptable range
    while((RPS.X() < x_coordinate - 1 || RPS.X() > x_coordinate + 1) && (frontLeftBump.Value() || frontRightBump.Value()))
    {
        if(RPS.X() > x_coordinate)
        {
            move_backwards(20,0.5);
        }
        else if(RPS.X() < x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            move_forward(20,0.5);
        }


    }
    return condition;
}
/** check_x_minus
    Moves the robot to a certain x coordinate while it is facing the negative x direction
    @param x_coordinate The coordinate the robot should go

*/

bool check_x_minus(float x_coordinate) //using RPS while robot is in the +x direction
{
    bool condition = true;
    //check whether the robot is within an acceptable range
    while(RPS.X() < x_coordinate - 1 || RPS.X() > x_coordinate + 1)
    {

       if(RPS.X() > x_coordinate)
        {
            move_forward(20,0.5);
        }
        else if(RPS.X() < x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            move_backwards(20,0.5);
        }


    }
    return condition;
}
/** check_y_minus
    Moves the robot to a certain y coordinate while it is facing the negative y direction
    @param y_coordinate The coordinate the robot should go

*/

bool check_y_minus(float y_coordinate) //using RPS while robot is in the -y direction
{
    bool condition = true;
    //check whether the robot is within an acceptable range
    while(RPS.Y() < y_coordinate - 1 || RPS.Y() > y_coordinate + 1)
    {

        if(RPS.Y() > y_coordinate)
        {
            move_forward(20,0.5);
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            move_backwards(20,0.5);
        }

    }
    return condition;
}
/** check_y_plus
    Moves the robot to a certain y coordinate while it is facing the positive y direction
    @param y_coordinate The coordinate the robot should go

*/
bool check_y_plus(float y_coordinate) //using RPS while robot is in the +y direction
{
    bool condition = true;
    //check whether the robot is within an acceptable range
    while(RPS.Y() < y_coordinate - 1 || RPS.Y() > y_coordinate + 1)
    {

        if(RPS.Y() > y_coordinate)
        {
            move_backwards(20,1);
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            move_forward(20,1);
        }

    }
    return condition;
}
/** moveTo
    Moves the robot to a certain coordinate.

    @param x The x coordinate the robot should go to
    @param y The y coordinate the robot should go to

*/
void moveTo(float x, float y) {
    float robotX = RPS.X();
    float robotY = RPS.Y();
    double deltaX = x - robotX;
    double deltaY = y - robotY;
    if(deltaX >=0 && deltaY >= 0) {
        faceDegree(0);
        bool check = check_x_plus(x);
        faceDegree(90);
        check = check_y_plus(y);
    }
    else if(deltaX <= 0 && deltaY >= 0 ) {

        faceDegree(90);
        bool check = check_y_plus(y);
        faceDegree(180);
        check = check_x_minus(x);

    }
    else if(deltaX <= 0 && deltaY <= 0 ) {
        faceDegree(270);
        bool check = check_y_minus(y);
        faceDegree(180);
        check = check_x_minus(x);
    }
    else if(deltaX >= 0 && deltaY <= 0 ) {
        faceDegree(0);
        bool check = check_x_plus(x);
        turn_right(20, 90);
        faceDegree(270);
        check = check_y_minus(y);
    }
}
/** waitForStart
    Initializes menu, waits for start light to go on.



*/
void waitForStart() {
    RPS.InitializeTouchMenu();
    LCD.Clear();
    while(cds.Value() > START_LIGHT_ON);
}
/** getLightColor
    Returns the color of the fuel light.

    @return 1 if light is blue, 2 if light is red
*/
int getLightColor() {
    if(cds.Value() > BLUE_LIGHT_ON) {
        return 1;
    }
    else {
        return 0;
    }
}
/** detectingLight
    Finds out whether the robot is detecting a light or not

    @return true if robot is detecting light, false otherwise
*/
bool detectingLight() {
    return (cds.Value() < 2.75);
}


/** setServo
    Sets the servo thresholds.

*/
void setServo() {
    arm.SetMin(818);
    arm.SetMax(2355);
}
void moveArm(float nextDegree, float currentDegree) {
    if(currentDegree < nextDegree) {
        while(currentDegree < nextDegree) {
            arm.SetDegree(currentDegree);
            currentDegree++;
            Sleep(10);
        }
    }
    else {
        while(currentDegree > nextDegree) {
            arm.SetDegree(currentDegree);
            currentDegree--;
            Sleep(10);
        }
    }
}

/** pullSwitch
    pulls a switch in front of the robot

*/
void pullSwitch() {
    driveToWall(20);
    Sleep(500);
    move_backwards(10, 2.7);
    arm.SetDegree(25);
    Sleep(2.0);
    move_backwards_timed(10, 1, 2);
    Sleep(2.0);
    arm.SetDegree(120);

}
/** pushSwitch
    pushes a switch in front of the robot

*/
void pushSwitch() {
    driveToWall(20);
    Sleep(500);
    move_backwards(10, 4.25);
    arm.SetDegree(25);
    Sleep(1.0);
    move_forward_timed(10,1, 1);
    Sleep(500);
    arm.SetDegree(120);
}
/** goUpSideRamp
    Assuming robot is facing ramp, moves up the side ramp, stopping when robot is completely on top level.

*/
void goUpSideRamp() {
    faceDegree(0);
    Sleep(500);
    driveToWall(20);
    Sleep(500);
    move_backwards(20, 0.1);
    Sleep(500);
    turn_left(20, 90);
    Sleep(500);
    driveToWall(30);
    Sleep(500);
    move_backwards(20,0.5);
    Sleep(500);
    turn_left(20, 90);
    Sleep(500);
    move_forward(30, 12.5);
    right_motor.Stop();
    left_motor.Stop();
    //end with robot facing left
    faceDegree(180);
}
/** flipSwitches
    Flips all 3 switches to their correct orientation

    @param red The direction for the red switch to go
    @param white The direction for the white switch to go
    @param blue The direction for the blue switch to go

*/
void flipSwitches(int red, int white, int blue) {
    //Starting at middle switch
    if(white == 1) {
        pushSwitch();
    }
    else {
        pullSwitch();
    }
    faceDegree(225);
    if(red == 1) {
        pushSwitch();
    }
    else {
        pullSwitch();
    }
    faceDegree(315);
    if(blue == 1) {
        pushSwitch();
    }
    else {
        pullSwitch();
    }
}
/** completeSwitches
    moves to switches and flips them

*/
void completeSwitches() {
    moveTo(Location::MID_SWITCH_X, Location::MID_SWITCH_Y);
    flipSwitches(RPS.RedSwitchDirection(), RPS.WhiteSwitchDirection(), RPS.BlueSwitchDirection());
}
/** pushButton
    pushes correct fuel button

*/
void pushButton() {
    moveTo(Location::FUEL_LIGHT_X, Location::FUEL_LIGHT_Y);
    int correctButton = getLightColor();
    if(correctButton == 1) {
        arm.SetDegree(90);
        move_forward_timed(10, 2, 2);
        Sleep(5.0);
        move_backwards_timed(10, 3, 2);
    }
    else {
        arm.SetDegree(120);
        move_forward_timed(10, 2, 2);
        Sleep(5.0);
        move_backwards_timed(10, 3, 2);
    }
}
void pickUpSupplies() {
    move_backwards(10, 2.75);
    faceDegree(270);
    arm.SetDegree(0);
    Sleep(1.0);
    moveArm(45, 0);
}

void dropSupplies() {
    move_backwards(10, 2.5);
    moveArm(0, 45);
    Sleep(1.0);
    move_backwards(10, 4);
    Sleep(1.0);
    arm.SetDegree(45);
}

/** findRPSPoints
    Test program to get coordinates of important places on the course.

*/
void findRPSPoints() {
    RPS.InitializeTouchMenu();
    while(true) {
        LCD.WriteLine(RPS.X());
        LCD.WriteLine(RPS.Y());
        LCD.WriteLine(RPS.Heading());
        Sleep(100);
        LCD.Clear();
    }
}
void performance3() {
    RPS.InitializeTouchMenu();
    setServo();
    arm.SetDegree(45);
    //go tu supplies
    move_forward(20, 3);
    turn_right(20, 95);
    moveTo(Location::SUPPLIES_X, Location::SUPPLIES_Y);
    Sleep(1.0);
    //PUT PICKING UP METHODS BELOW

    pickUpSupplies();

    //go to bottom of ramp
    faceDegree(90);
    moveTo(Location::BOTTOM_SIDE_RAMP_X, Location::BOTTOM_SIDE_RAMP_Y);
    faceDegree(0);
    goUpSideRamp();
    turn_left(20, 90);
    faceDegree(270);
    check_y_minus(Location::TOP_MAIN_RAMP_Y);
   //go to drop zone
   turn_right(20, 90);
   faceDegree(180);
   check_x_minus(Location::MID_SWITCH_X-1);
   turn_right(20, 95);
   faceDegree(90);
   move_forward_timed(20, 10, 2);
   //drop package
   dropSupplies();


   Sleep(2.0);
   //go down ramp
   move_backwards(10, 3);
   turn_right(20, 90);
   faceDegree(0);
   check_x_plus(Location::TOP_MAIN_RAMP_X-2);
   turn_right(20, 90);
   faceDegree(270);
   Sleep(1.0);
   move_forward_timed(20, 20, 5);
}

int main(void)
{

    performance3();


    return 0;
}

