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
#define ON_LINE 3
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
    right_motor.SetPercent(percent);
    Sleep(1);
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
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);
    int mp = percent;

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    int start_time = TimeNow();
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts && TimeNow() - start_time < time) {
        mp = TUNING_CONSTANT*(left_encoder.Counts()-right_encoder.Counts())+(percent);
        right_motor.SetPercent(mp);
    }

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
            left_motor.SetPercent(percent+10);
            right_motor.SetPercent(percent-15);
        }
        else if(!frontLeftBump.Value()) {
            right_motor.SetPercent(percent+10);
            left_motor.SetPercent(percent-15);
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
void followLine(float speed, float distance) {
        int leftValue, rightValue, midValue, state;
        float counts = distance * COUNTS_PER_INCH;
        right_encoder.ResetCounts();
        left_encoder.ResetCounts();
        while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts )
        {
            leftValue = left.Value();
            rightValue = right.Value();
            midValue = middle.Value();

            if(midValue >= ON_LINE && rightValue <= ON_LINE && leftValue<= ON_LINE) {
                state = CENTER;
            }
            else if(midValue >= ON_LINE && rightValue>= ON_LINE && leftValue <= ON_LINE) {
                state = LEFT;
            }
            else if(midValue <= ON_LINE && rightValue>= ON_LINE && leftValue <= ON_LINE) {
                state = FAR_LEFT;
            }
            else if(midValue >= ON_LINE && rightValue<= ON_LINE && leftValue >= ON_LINE) {
                state = RIGHT;
            }
            else if(midValue <= ON_LINE && rightValue<= ON_LINE && leftValue >= ON_LINE) {
                state = FAR_RIGHT;
            }
            else {
                state = OFF_LINE;
            }


            switch(state) {
                case CENTER:
                    LCD.WriteLine("CENTER");
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(speed);
                    break;
                case LEFT:
                LCD.WriteLine("LEFT");
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(.75 *speed);
                    break;
                case FAR_LEFT:
                LCD.WriteLine("FAR LEFT");
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(.5*speed);
                    break;
                case RIGHT:
                LCD.WriteLine("RIGHT");
                    left_motor.SetPercent(.75*speed);
                    right_motor.SetPercent(speed);
                    break;
                case FAR_RIGHT:
                LCD.WriteLine("FAR RIGHT");
                    left_motor.SetPercent(.5*speed);
                    right_motor.SetPercent(speed);
                    break;
                case OFF_LINE:
                LCD.WriteLine("OFFLINE");
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(speed);
                    break;


            }
        }
}
void followLineYellow(float speed, float distance) {
        int leftValue, rightValue, midValue, state;
        float counts = distance * COUNTS_PER_INCH;
        right_encoder.ResetCounts();
        left_encoder.ResetCounts();
        while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts )
        {
            leftValue = left.Value();
            rightValue = right.Value();
            midValue = middle.Value();

            if(midValue <= ON_LINE && rightValue >= ON_LINE && leftValue>= ON_LINE) {
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
                    LCD.WriteLine("CENTER");
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(speed);
                    break;
                case LEFT:
                LCD.WriteLine("LEFT");
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(.75 *speed);
                    break;
                case FAR_LEFT:
                LCD.WriteLine("FAR LEFT");
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(.5*speed);
                    break;
                case RIGHT:
                LCD.WriteLine("RIGHT");
                    left_motor.SetPercent(.75*speed);
                    right_motor.SetPercent(speed);
                    break;
                case FAR_RIGHT:
                LCD.WriteLine("FAR RIGHT");
                    left_motor.SetPercent(.5*speed);
                    right_motor.SetPercent(speed);
                    break;
                case OFF_LINE:
                LCD.WriteLine("OFFLINE");
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(speed);
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
    float timeStarted = TimeNow();
    while(deltaTheta > 0.5) {
        LCD.WriteLine(RPS.Heading());
        headingToZero = 0;
        degreeToZero = degree-RPS.Heading();
        if(degreeToZero < 0) {
            degreeToZero+=360;
        }
        deltaTheta = angleBetween(headingToZero, degreeToZero);
        if(degreeToZero > 180) {
                turn_right(15,0.1);
               Sleep(50);

        }
        else {
                turn_left(15,0.1);
               Sleep(50);
            }
        }



 }

void checkPositioning(float x, float y, bool positiveY) {
    //take care of x positioning
    if(x > RPS.X()) {
        move_backwards_timed(10, 2, 1);
        if(positiveY) {
            faceDegree(75);
        }
        else {
            faceDegree(285);
        }
        while((RPS.X() < x - 0.6 || RPS.X() > x + 0.6))
        {
            if(RPS.X() > x)
            {
                move_backwards(20,0.1);
            }
            else if(RPS.X() < x)
            {
                //pulse the motors for a short duration in the correct direction
                move_forward(20,0.1);
            }
            if(RPS.X() < 0) {
                LCD.WriteLine("Override");
                break;
            }
            Sleep(50);
        }
    }
    else {
        move_backwards_timed(10, 2, 1);
        if(positiveY) {
            faceDegree(105);
        }
        else {
            faceDegree(255);
        }
         while(RPS.X() < x - 0.6 || RPS.X() > x + 0.6)
        {
            if(RPS.X() < 0) {
                LCD.WriteLine("Override");
                break;
            }
           if(RPS.X() > x)
            {
                move_forward(20,0.1);
            }
            else if(RPS.X() < x)
            {
                //pulse the motors for a short duration in the correct direction
                move_backwards(20,0.1);
            }
            Sleep(50);


        }

    }
    if(positiveY) {
        faceDegree(90);
        while(RPS.Y() < y - 1 || RPS.Y() > y + 1)
        {
                if(RPS.Y() > y)
                {
                    move_backwards(20,0.1);
                }
                else if(RPS.Y() < y)
                {
                    //pulse the motors for a short duration in the correct direction
                    move_forward(20,0.1);
                }
                Sleep(50);

        }
        faceDegree(90);
    }
    else {
        faceDegree(180);
        while(RPS.Y() < y - 1 || RPS.Y() > y + 1)
        {
                if(RPS.Y() < y)
                {
                    move_backwards(20,0.1);
                }
                else if(RPS.Y() > y)
                {
                    //pulse the motors for a short duration in the correct direction
                    move_forward(20,0.1);
                }
                Sleep(50);

        }
        faceDegree(180);
    }

}

bool offPositioning(float x) {
    return (RPS.X() < x - 0.6 || RPS.X() > x + 0.6);
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
        LCD.WriteLine(frontLeftBump.Value());
        LCD.WriteLine(frontRightBump.Value());

        if(RPS.X() > x_coordinate)
        {
            move_backwards_timed(20,0.1, 1);
        }
        else if(RPS.X() < x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            move_forward_timed(20,0.1,1);
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
    float startingDegree = RPS.Heading();
    while(RPS.X() < x_coordinate - 1 || RPS.X() > x_coordinate + 1)
    {
       if(RPS.X() > x_coordinate)
        {
            move_forward(20,0.1);
        }
        else if(RPS.X() < x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            move_backwards(20,0.1);
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
    float startingDegree = RPS.Heading();
    while(RPS.Y() < y_coordinate - 1 || RPS.Y() > y_coordinate + 1)
    {
        if(RPS.Y() > y_coordinate)
        {
            move_forward(20,0.1);
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            move_backwards(20,0.1);
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
    float startingDegree = RPS.Heading();
    while(RPS.Y() < y_coordinate - 1 || RPS.Y() > y_coordinate + 1)
    {
        if(RPS.Y() > y_coordinate)
        {
            move_backwards(20,0.1);
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            move_forward(20,0.1);
        }

    }
    return condition;
}
float locationDegree(float x, float y, int quadrant) {
    float delY = y - RPS.Y();
    float delX = x - RPS.X();
    if(quadrant == 1) {
        float angle = atan(delY/delX) * 180 / PI;
    }
    else if(quadrant == 2) {
        float angle = atan(delY/delX) * 180/PI;
        angle += 180;
    }
    else if(quadrant == 3) {
        float angle = atan(delY/delX) * 180/PI * -1;
        angle += 180;
    }
    else {
        float angle = atan(delY/delX) * 180/PI * -1;
        angle += 360;
    }
    return angle;
}

void faceLocation(float x, float y, int quadrant) {
    float angle = locationDegree(x, y, quadrant);
    float currentHeading = RPS.Heading();
    float deltaTheta = angleBetween(currentHeading, angle);
    float tempAngle -= currentHeading;
    if(tempAngle < 0) {
        tempAngle += 360;
    }
    if(tempAngle > 180) {
        turn_right(20, deltaTheta);
    }
    else {
        turn_left(20, deltaTheta);
    }
    faceDegree(angle);

}



float distanceTo(float x, float y) {
    return sqrt(pow((x - RPS.X), 2) + pow((y - RPS.Y()), 2));
}
/** moveTo
    Moves the robot to a certain coordinate.
    @param x The x coordinate the robot should go to
    @param y The y coordinate the robot should go to
*/
void moveToForwards(float x, float y) {
    int quad;
    float delY = y - RPS.Y();
    float delX = x - RPS.X();
    if(delX > 0) {
        if(delY >= 0) {
            quad = 1;
        }
        else {
            quad = 4;
        }
    }
    else {
        if(delY >= 0) {
            quad = 2;
        }
        else {
            quad = 3;
        }
    }
    faceLocation(x, y, quad);
    move_forward(20, distanceTo(x, y));
}
void moveToBackwards(float x, float y) {
    int quad;
    float delY = y - RPS.Y();
    float delX = x - RPS.X();
    if(delX > 0) {
        if(delY >= 0) {
            quad = 1;
        }
        else {
            quad = 4;
        }
    }
    else {
        if(delY >= 0) {
            quad = 2;
        }
        else {
            quad = 3;
        }
    }
    faceLocation(x, y, quad);
    float head = RPS.Heading();
    head -= 180;
    if(head < 360) head += 360;
    turn_right(20, 180);
    faceDegree(head);
    move_backwards_timed(20, distanceTo(x, y), 5);
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
    return (cds.Value() < 1.5);
}


/** setServo
    Sets the servo thresholds.
*/
void setServo() {
    arm.SetMin(818);
    arm.SetMax(2355);
}
void moveArm(float currentDegree, float nextDegree) {
    if(currentDegree < nextDegree) {
        while(currentDegree < nextDegree) {
            arm.SetDegree(currentDegree);
            currentDegree++;
            Sleep(20);
        }
    }
    else {
        while(currentDegree > nextDegree) {
            arm.SetDegree(currentDegree);
            currentDegree--;
            Sleep(20);
        }
    }
}

/** pullSwitch
    pulls a switch in front of the robot
*/
void pullSwitch() {
    
    Sleep(500);
    move_backwards(10, 2.7);
    arm.SetDegree(0);
    Sleep(500);
    move_backwards_timed(10, 1, 2);
    Sleep(500);
    arm.SetDegree(90);

}
/** pushSwitch
    pushes a switch in front of the robot
*/
void pushSwitch() {
    move_forward(15, 2);
    Sleep(500);
    move_backwards(10, 4.25);
    arm.SetDegree(0);
    Sleep(1.0);
    move_forward_timed(10,1, 1);
    Sleep(500);
    arm.SetDegree(90);
}
/** goUpSideRamp
    Assuming robot is facing ramp, moves up the side ramp, stopping when robot is completely on top level.
*/
void goUpSideRamp() {
    faceDegree(0);
    Sleep(500);
    move_forward(30, 5);
    followLine(30, 5);
    driveToWall(35);
    Sleep(500);
    move_backwards(30, 0.25);
    Sleep(500);
    turn_left(20, 91);
    Sleep(500);
    followLine(35, 28);
    turn_left(20, 5);
    driveToWall(35);
    Sleep(500);
    move_backwards(35,0.5);
    Sleep(500);
    turn_left(20, 90);
    Sleep(500);
    LCD.WriteLine("FORWARD");

    move_forward(20, 16);
    LCD.WriteLine("STOP");
    right_motor.Stop();
    left_motor.Stop();
    Sleep(500);
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
    while(RPS.Heading() < 0) {
        move_backwards(20, 1);
        Sleep(50);
    }
    moveToBackwards(Location::MID_SWITCH_X, Location::MID_SWITCH_Y);
    flipSwitches(RPS.RedSwitchDirection(), RPS.WhiteSwitchDirection(), RPS.BlueSwitchDirection());
}
/** pushButton
    pushes correct fuel button
*/
void pushButton() {
    int correctButton = getLightColor();
    if(correctButton == 0) {
        LCD.WriteLine("RED");
        move_backwards(10, 8);
        faceDegree(90);
        moveArm(90, 18.7);
        move_forward_timed(20, 3, 6);
        move_forward_timed(10, 100, 7);
        move_backwards_timed(10, 5, 2);
        moveArm(20, 90);
    }
    else {
        LCD.WriteLine("BLUE");
        move_backwards(10, 1);
        arm.SetDegree(95);
        move_forward_timed(20, 100, 7);
        move_backwards_timed(10, 3, 2);
    }
}
void wiggle() {
    turn_right(15, 1);
    turn_left(15, 1);
    turn_right(15, 1);
    turn_left(15, 1);
    turn_right(15, 1);
    turn_left(15, 1);
}
void pickUpSupplies() {

    move_backwards(10, 2);
    LCD.WriteLine("moving backwards");

    moveArm(90, 0);
    LCD.WriteLine("moving arm down");

    move_forward_timed(15, 1, 1);
    LCD.WriteLine("moving forwards");
    wiggle();
    Sleep(1.0);

    move_backwards_timed(10, 0.7, 2);
    LCD.WriteLine("moving backwards");

    moveArm(0, 85);
    LCD.WriteLine("Moving arm up");
}



void dropSupplies() {
    move_backwards(10, 1.5);
    LCD.WriteLine("moving backwards");
    moveArm(45, 20);
    LCD.WriteLine("moving arm down");
    Sleep(3.0);
    LCD.WriteLine("sleep");
    move_backwards(30, 10);
    LCD.WriteLine("moving backwards");
    Sleep(1.0);
    LCD.WriteLine("sleep");
    arm.SetDegree(90);
    LCD.WriteLine("arm up");
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
void startToSupplies() {
    setServo();
    arm.SetDegree(90);
    faceDegree(90);
    moveToForwards(Location::SUPPLIES_X, Location::SUPPLIES_Y+0.6);
    Sleep(1.0);
    //PUT PICKING UP METHODS BELOW

    pickUpSupplies();
}

void suppliesToTop() {
    faceDegree(270);
    move_backwards(20, distanceTo(Location::BOTTOM_SIDE_RAMP_X, Location::BOTTOM_SIDE_RAMP_Y));
    turn_left(30,90);
    faceDegree(0);
    goUpSideRamp();

}

void doButtons() {
    moveToForwards(Location::FUEL_LIGHT_X, Location::FUEL_LIGHT_Y);
    faceDegree(90);
    //push the button
    pushButton();
}

void dropOff() {
    float angle = locationDegree(Location::DROP_OFF_X, Location::DROP_OFF_Y, 3);
    moveToForwards(Location::DROP_OFF_X, Location::DROP_OFF_Y);
    turn_right(20, angleBetween(angle, 90));
    move_forward(20, 2);
    dropSupplies();

}

void finish() {
    moveToForwards(Location::TOP_MAIN_RAMP_X, TOP_MAIN_RAMP_Y);
    faceLocation(RPS.X(), RPS.Y() - 3, 4);
    move_forward(30, 10);
    moveToBackwards(Location::START_X, Location::START_Y);
}

void goGoGo() {
    startToSupplies();
    suppliesToTop();
    doButtons();
    dropOff();
    completeSwitches();
    finish();

}

int main(void)
{   setServo();
    arm.SetDegree(90);
    waitForStart();

    goGoGo();


    return 0;

}