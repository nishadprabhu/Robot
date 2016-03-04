

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
#define ON_LINE 1.9
#define CENTER 0
#define LEFT -1
#define FAR_LEFT -2
#define RIGHT 1
#define FAR_RIGHT 2
#define OFF_LINE 4
//Defining constants to convert counts to inches or degrees
#define COUNTS_PER_INCH 33.74
#define LEFT_COUNTS_PER_DEGREE 1.99
#define RIGHT_COUNTS_PER_DEGREE 1.87
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
        mp = TUNING_CONSTANT*(left_encoder.Counts()-right_encoder.Counts())+(percent+1);
        right_motor.SetPercent(mp);
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}
void move_forward_timed(int percent, int time) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(percent+2);
    left_motor.SetPercent(percent);
    int mp = percent;

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    int start_time = TimeNow();
    while(TimeNow() - start_time < time);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}
void move_backward_timed(int percent, float inches, int time) //using encoders
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
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts && TimeNow() - start_time < time);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}
void accel_forward(int percent, float inches) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    float counts = inches*COUNTS_PER_INCH;
    //Set both motors to desired percent
    int mp = 1;
    right_motor.SetPercent(mp);
    left_motor.SetPercent(mp);


    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts) {
        while(mp<percent) {
            mp++;
            right_motor.SetPercent(mp+1);
            left_motor.SetPercent(mp);
            Sleep(20);

        }
        mp = TUNING_CONSTANT*(left_encoder.Counts()-right_encoder.Counts())+percent;
        right_motor.SetPercent(mp);
//        if((left_encoder.Counts() + right_encoder.Counts())/2. > counts*.75) {
//            right_motor.SetPercent(percent * .5);
//        }
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}
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
void accel_backwards(int percent, float inches) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    float counts = inches*COUNTS_PER_INCH;
    //Set both motors to desired percent
    int mp = -1;
    percent *=-1;
    right_motor.SetPercent(mp);
    left_motor.SetPercent(mp);


    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts) {
        while(mp>percent) {
            mp--;
            right_motor.SetPercent(mp);
            left_motor.SetPercent(mp);
        }
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}
//Drives either forwards or backwards until it hits a wall.
void driveToWall(int percent) {

    right_motor.SetPercent(percent+5);
    left_motor.SetPercent(percent);
    int mp = percent+5;
    while(frontLeftBump.Value() || frontRightBump.Value()) {
        mp = TUNING_CONSTANT*(left_encoder.Counts()-right_encoder.Counts())+(percent+2);
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
void followLine(float speed) {
        int leftValue, rightValue, midValue, state;
        while(cds.Value() )
        {
            leftValue = left.Value();
            rightValue = right.Value();
            midValue = middle.Value();

            if(midValue >= ON_LINE && rightValue <= ON_LINE && leftValue <= ON_LINE) {
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
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(speed);
                    break;
                case LEFT:
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(.75*speed);
                    break;
                case FAR_LEFT:
                    left_motor.SetPercent(speed);
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
                    right_motor.SetPercent(speed);
                    break;
            }
        }

}

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
/*
void faceLocation(float x, float y) {
    float robotX = RPS.X();
    float robotY = RPS.Y();
    double deltaX = x - robotX;
    double deltaY = y - robotY;
    double degree;
    if(deltaX == 0) {
        if(deltaY > 0) {
            degree = 90;
        }
        else if (deltaY < 0) {
            degree = 270;
        }
        else {
            LCD.WriteLine("In Location");
            return;
        }
    }
    else if (deltaX > 0) {
        degree = atan(deltaY/deltaX) * 180/PI;
        if(deltaY < 0) {
            degree = 360 + degree;
        }
    }
    else if (deltaX < 0) {
        degree = atan(deltaY/deltaX) * 180/PI + 180;
    }
    float heading = RPS.Heading();
    float deltaTheta = angleBetween(heading, degree)
    while(abs(deltaTheta) > 0) {
        deltaTheta = angleBetween(RPS.Heading(), degree);
        float headingToZero = 0;
        float degreeToZero = degree-RPS.Heading();
        if(degreeToZero < 0) {
            degreeToZero += 360;
        }
        if(degree < 0) {
            degree = 360 + degree;
        }
        if(degree > 180) {
            turn_right(30,1);
        }
        else {
            turn_left(30,1);
        }
        Sleep(100);
    }
}
void moveTo(float x, float y) {
  faceLocation(x,y);
  float deltaX = x - RPS.X(), deltaY = y - RPS.Y();
  while(abs(deltaX) > 0 && abs(deltaY) > 0){
      accel_forward(30, 1);
      Sleep(0.01);
  }
}
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
void faceDegree(float degree) {
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
            if(deltaTheta > 5) {
                turn_right(20,1);
            }
            else {
               turn_right(10,0.5);
               Sleep(10);
            }

        }
        else {
            if(deltaTheta > 5) {
                turn_left(20,1);
            }
            else {
               turn_right(10,0.5);
               Sleep(10);
            }
        }

    }
}


bool check_x_plus(float x_coordinate) //using RPS while robot is in the +x direction
{
    bool condition = true;
    //check whether the robot is within an acceptable range
    while(RPS.X() < x_coordinate - 1 || RPS.X() > x_coordinate + 1)
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
        else if (RPS.X() < 0) {
            condition = false;
            break;
        }

    }
    return condition;
}
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
        else if(RPS.X() < 0) {
            condition = false;
            break;
        }

    }
    return condition;
}
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
        else if(RPS.Y() < 0) {
            condition = false;
            break;
        }
    }
    return condition;
}

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
        else if(RPS.Y() < 0) {
            condition = false;
            break;
        }
    }
    return condition;
}
void moveTo(float x, float y) {
    float robotX = RPS.X();
    float robotY = RPS.Y();
    double deltaX = x - robotX;
    double deltaY = y - robotY;
    if(deltaX >=0 && deltaY >= 0) {
        faceDegree(0);
        bool check = check_x_plus(x);
        while(!check) {
            turn_left(10, 90);
            move_forward(10, 1);
            turn_right(10, 90);
            check = check_x_plus(x);
        }
        faceDegree(90);
        check = check_y_plus(y);
        while(!check) {
            turn_right(10, 90);
            move_forward(10, 1);
            turn_left(10, 90);
            check = check_y_plus(y);
        }
    }
    else if(deltaX <= 0 && deltaY >= 0 ) {
        faceDegree(180);
        bool check = check_x_minus(x);
        while(!check) {
            turn_right(10, 90);
            move_forward(10, 1);
            turn_left(10, 90);
            check = check_x_plus(x);
        }
        faceDegree(90);
        check = check_y_plus(y);
    }
    else if(deltaX <= 0 && deltaY <= 0 ) {
        faceDegree(270);
        bool check = check_y_minus(y);
        faceDegree(180);
        check = check_x_minus(x);
        while(!check) {
            turn_right(10, 90);
            move_forward(10, 1);
            turn_left(10, 90);
            check = check_x_plus(x);
        }
    }
    else if(deltaX >= 0 && deltaY <= 0 ) {
        faceDegree(0);
        bool check = check_x_plus(x);
        faceDegree(270);
        check = check_y_minus(y);
    }
}

void waitForStart() {
    RPS.InitializeTouchMenu();
    while(!buttons.MiddlePressed()); //Wait for middle button to be pressed
    while(buttons.MiddlePressed()); //Wait for middle button to be unpressed
    LCD.Clear();
    while(cds.Value() > START_LIGHT_ON);
}
//returns 1 if light is BLUE, RED if otherwise
int getLightColor() {
    if(cds.Value() > BLUE_LIGHT_ON) {
        return 1;
    }
    else {
        return 0;
    }
}
bool detectingLight() {
    return (cds.Value() < 2.75);
}

void performance1() {

    Sleep(2.0); //Wait for counts to stabilize
    move_forward(30, 21);
    Sleep(1.0);
    turn_right(20, 45);
    Sleep(1.0);
    driveToWall(20);
    Sleep(1.0);
    move_backwards(20, 1);
    Sleep(1.0);
    turn_left(20, 90);
    Sleep(1.0);
    driveToWall(30);
    Sleep(1.0);
    move_backwards(20,0.5);
    Sleep(1.0);
    turn_left(20, 90);
    Sleep(1.0);
    move_forward(30, 13.5);
    Sleep(1.0);
    turn_right(20,90);
    Sleep(1.0);
    move_forward(30, 11);
    if(detectingLight()) {
        if(getLightColor() == 1) {
            LCD.WriteLine("Blue");
        }
        else if(getLightColor() == 0) {
            LCD.WriteLine("Red");
        }
        else {
            //LCD.WriteLine("Red");
        }
    }

    else {
       // LCD.WriteLine("Red");
    }
}
void setServo() {
    arm.SetMin(818);
    arm.SetMax(2355);
}

void pullSwitch() {
    driveToWall(20);
    Sleep(500);
    move_backwards(10, 2.7);
    arm.SetDegree(25);
    Sleep(2.0);
    move_backward_timed(10, 1, 2);
    Sleep(2.0);
    arm.SetDegree(120);

}

void pushSwitch() {
    driveToWall(20);
    Sleep(500);
    move_backwards(10, 4.25);
    arm.SetDegree(25);
    Sleep(1.0);
    move_forward_timed(10,1);
    Sleep(500);
    arm.SetDegree(120);
}

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

void performance2() {
    setServo();
    arm.SetDegree(120);
    move_forward(20, 4);
    moveTo(Location::BOTTOM_SIDE_RAMP_X, Location::BOTTOM_SIDE_RAMP_Y);
    goUpSideRamp();
    move_forward(20, 24);
    turn_left(20, 90);
    pullSwitch();
    turn_left(20, 90);
    move_forward(20,4);
    turn_right(20, 90);
    faceDegree(270);
    pushSwitch();
    moveTo(Location::FUEL_LIGHT_X, Location::FUEL_LIGHT_Y);
    move_forward_timed(10,2);
}
void flipSwitches() {
    //start facing right switch from upper level
    if(RPS.RedSwitchDirection()==1) {
        pushSwitch();
    }
    else {
        pullSwitch();
    }
    turn_left(20, 90);
    move_forward(20,4);
    turn_right(20, 90);
    faceDegree(270);
    if(RPS.WhiteSwitchDirection()==1) {
        pushSwitch();
    }
    else {
        pullSwitch();
    }
    turn_left(20, 90);
    move_forward(20,4);
    turn_right(20, 90);
    faceDegree(270);
    if(RPS.BlueSwitchDirection()==1) {
        pushSwitch();
        move_forward(10, 1);
    }
    else {
        pullSwitch();
    }
}

void findRPSPoints() {
    while(true) {
        LCD.WriteLine(RPS.X());
        LCD.WriteLine(RPS.Y());
        Sleep(100);
        LCD.Clear();
    }
}


int main(void)
{
    //waitForStart();
    //performance2();
    findRPSPoints();

    return 0;
}
