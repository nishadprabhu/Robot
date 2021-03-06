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
#define LEFT_COUNTS_PER_DEGREE 1.955
#define RIGHT_COUNTS_PER_DEGREE 1.88
//Define thresholds for line following/start light
#define START_LIGHT_ON 1.5
#define BLUE_LIGHT_ON 0.75
//Tuning constant
#define TUNING_CONSTANT 0.08
#define I_TUNING_CONSTANT 0.01
//PI
# define M_PI           3.14159265358979323846
#define SPEED 40
#define MAX_SPEED 45

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
AnalogInputPin cds1(FEHIO::P3_0);
AnalogInputPin cds2(FEHIO::P3_1);


DigitalInputPin frontLeftBump(FEHIO::P2_0);
DigitalInputPin frontRightBump(FEHIO::P2_1);

double accum_error = 0;

float SUPPLIES_X = 29.35;
float SUPPLIES_Y = 12.3;
float DROP_OFF_X = 5.5;
float DROP_OFF_Y = 49.5;
float START_X = 7.6;
float START_Y = 8.9 ;
int lightColor;
void bumpValues() {
    LCD.WriteLine(frontLeftBump.Value());
    LCD.WriteLine(frontRightBump.Value());
}
void setRPSCoords() {
    LCD.WriteLine("SUPPLIES");
    float x, y;
    while(!LCD.Touch(&x, &y)) {
        LCD.WriteLine(RPS.X());
        LCD.WriteLine(RPS.Y());
        LCD.DrawRectangle(289, 219, 30, 20);
        Sleep(50);
        LCD.Clear();
    }
    if(x > 289 && y > 219) {
        return;
    }
    SUPPLIES_X = RPS.X();
    SUPPLIES_Y = RPS.Y();\
    LCD.Clear();
    LCD.WriteLine("DROP OFF");
    Sleep(1.0);
    while(!LCD.Touch(&x, &y)) {
        LCD.WriteLine(RPS.X());
        LCD.WriteLine(RPS.Y());
        LCD.DrawRectangle(289, 219, 30, 20);
        Sleep(50);
        LCD.Clear();
    }
    if(x > 289 && y > 219) {
        SUPPLIES_X = 29.35;
        SUPPLIES_Y = 12.3;
        return;
    }
    DROP_OFF_X = RPS.X();
}



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
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts && (frontLeftBump.Value() && frontRightBump.Value()) ) {
        double current_error = (left_encoder.Counts()-right_encoder.Counts());
        accum_error +=current_error;
        mp = TUNING_CONSTANT*current_error+I_TUNING_CONSTANT*accum_error+(percent);
        right_motor.SetPercent(mp);
        bumpValues();
    }

    right_motor.SetPercent(-20);
    left_motor.SetPercent(-20);
    Sleep(100);
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
void move_forward_timed(int percent, float inches, double time) //using encoders
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
    double start_time = TimeNow();
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts && TimeNow() - start_time < time && (frontLeftBump.Value() && frontRightBump.Value())) {
//        double current_error = (left_encoder.Counts()-right_encoder.Counts());
//        accum_error +=current_error;
//        mp = TUNING_CONSTANT*current_error+I_TUNING_CONSTANT*accum_error+(percent);
//        right_motor.SetPercent(mp);
//        bumpValues();
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}
void pivot_right(int percent, float degrees) {
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    float counts = degrees * LEFT_COUNTS_PER_DEGREE * 1.05;
    left_motor.SetPercent(percent);
    right_motor.SetPercent((-percent) * 0.7);
    while((right_encoder.Counts() + left_encoder.Counts())/2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/** move_backwards
    Moves the robot backwards
    @param percent Motor percent
    @param inches Distance robot needs to travel
*/
void move_backwards(int percent, double inches) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    double counts = inches*COUNTS_PER_INCH;
    //Set both motors to desired percent
    right_motor.SetPercent(-1*percent);
    left_motor.SetPercent(-1*percent);
    int mp = percent;
    //While the average of the left and right encoder are less than counts,
    //keep running motors
     while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts) {
         mp = TUNING_CONSTANT*(left_encoder.Counts()-right_encoder.Counts())+(percent);
         mp *= -1;
         right_motor.SetPercent(mp);
     }

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
void move_backwards_timed(int percent, float inches, float time) //using encoders
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
    float start_time = TimeNow();
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
    int mp = percent;
    double start_time = TimeNow();
    while((frontLeftBump.Value() || frontRightBump.Value()) && TimeNow() - start_time < 3.0) {

        if(!frontRightBump.Value()) {
            left_motor.SetPercent(percent+ 10);
            right_motor.SetPercent(-1 * percent * .1);
        }
        else if(!frontLeftBump.Value()) {
            right_motor.SetPercent(percent+10);
            left_motor.SetPercent(-1 * percent * .1);
        }
        else {
            double current_error = (left_encoder.Counts()-right_encoder.Counts());
            mp = TUNING_CONSTANT*current_error+(percent);
            right_motor.SetPercent(mp);
            if(percent < 0) {
                mp *= -1;
            }
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
        int start_time = TimeNow();
        while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts && TimeNow() - start_time < 3 && (frontLeftBump.Value() || frontRightBump.Value()))
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


            if(!(frontLeftBump.Value() && frontRightBump.Value()))
            {
                if(!frontRightBump.Value()) {
                    left_motor.SetPercent(speed+10);
                    right_motor.SetPercent(-15);
                }
                else if(!frontLeftBump.Value()) {
                    right_motor.SetPercent(speed+10);
                    left_motor.SetPercent(-15);
                }
            }
            else {
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
}
void followLineYellowSquare(float speed, float distance) {
        int leftValue, rightValue, midValue, state;
        float counts = distance * COUNTS_PER_INCH;
        right_encoder.ResetCounts();
        left_encoder.ResetCounts();
        int start_time = TimeNow();
        while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts && (TimeNow() - start_time < 5) && (frontLeftBump.Value() || frontRightBump.Value()))
        {
            leftValue = left.Value();
            rightValue = right.Value();
            midValue = middle.Value();
            bumpValues();

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
            if(!(frontLeftBump.Value() && frontRightBump.Value()))
            {
                if(!frontRightBump.Value()) {
                    LCD.WriteLine("right");
                    left_motor.SetPercent(2 * speed);
                    right_motor.SetPercent(-speed * .01);
                }
                else if(!frontLeftBump.Value()) {
                    LCD.WriteLine("left");

                    right_motor.SetPercent( 2 * speed);
                    left_motor.SetPercent(-speed * .1);
                }
            }
            else {
                switch(state) {
                    case CENTER:
                        //LCD.WriteLine("CENTER");
                        left_motor.SetPercent(speed);
                        right_motor.SetPercent(speed);
                        break;
                    case LEFT:
                   // LCD.WriteLine("LEFT");
                        left_motor.SetPercent(speed);
                        right_motor.SetPercent(.5 *speed);
                        break;
                    case FAR_LEFT:
                   //LCD.WriteLine("FAR LEFT");
                        left_motor.SetPercent(speed);
                        right_motor.SetPercent(.25*speed);
                        break;
                    case RIGHT:
                   // LCD.WriteLine("RIGHT");
                        left_motor.SetPercent(0.5*speed);
                        right_motor.SetPercent(speed);
                        break;
                    case FAR_RIGHT:
                    //LCD.WriteLine("FAR RIGHT");
                        left_motor.SetPercent(.25*speed);
                        right_motor.SetPercent(speed);
                        break;
                    case OFF_LINE:
                    //LCD.WriteLine("OFFLINE");
                        left_motor.SetPercent(speed);
                        right_motor.SetPercent(speed);
                        break;

            }
        }
        }
        right_motor.Stop();
        left_motor.Stop();

}

void followLineYellow(float speed, float distance) {
        int leftValue, rightValue, midValue, state;
        float counts = distance * COUNTS_PER_INCH;
        right_encoder.ResetCounts();
        left_encoder.ResetCounts();
        int start_time = TimeNow();
        while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts && (TimeNow() - start_time < 5) && (frontLeftBump.Value() && frontRightBump.Value()))
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
                    //LCD.WriteLine("CENTER");
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(speed);
                    break;
                case LEFT:
               // LCD.WriteLine("LEFT");
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(.5 *speed);
                    break;
                case FAR_LEFT:
               //LCD.WriteLine("FAR LEFT");
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(.25*speed);
                    break;
                case RIGHT:
               // LCD.WriteLine("RIGHT");
                    left_motor.SetPercent(0.5*speed);
                    right_motor.SetPercent(speed);
                    break;
                case FAR_RIGHT:
                //LCD.WriteLine("FAR RIGHT");
                    left_motor.SetPercent(.25*speed);
                    right_motor.SetPercent(speed);
                    break;
                case OFF_LINE:
                //LCD.WriteLine("OFFLINE");
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(speed);
                    break;


            }
            if(!frontRightBump.Value()) {
                left_motor.SetPercent(speed);
                right_motor.SetPercent(-15);
            }
            else if(!frontLeftBump.Value()) {
                right_motor.SetPercent(speed);
                left_motor.SetPercent(-15);
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

    float headingToZero = 0;
    float degreeToZero = degree - RPS.Heading();
    if(degreeToZero < 0) {
        degreeToZero+=360;
    }
    float deltaTheta = angleBetween(headingToZero, degreeToZero);
    float timeStarted = TimeNow();
    while(deltaTheta > 0.8 && TimeNow() - timeStarted < 5) {
        if(RPS.Heading() >= 0)  {
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



 }
float distanceTo(float x, float y) {
    while(RPS.X() < 0);
    return sqrt((x - RPS.X()) * (x - RPS.X()) + (y - RPS.Y()) * (y - RPS.Y()));
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
            move_backwards(20,0.1);
        }
        else if(RPS.X() < x_coordinate)
        {
            //pulse the motos for a short duration in the correct direction
            move_forward(20,0.1);
        }

        Sleep(50);
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
            move_forward_timed(20,0.1,1);
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
    while(RPS.Y() < y_coordinate - 0.5 || RPS.Y() > y_coordinate + 0.5)
    {
        if(RPS.Y() > y_coordinate)
        {
            move_forward_timed(20,0.1, 1);
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            move_backwards(20,0.1);
        }
        Sleep(50);

    }
    return condition;
}
bool on_line() {
    float leftValue = left.Value();
    float rightValue = right.Value();
   float midValue = middle.Value();
    return (midValue >= ON_LINE && rightValue >= ON_LINE && leftValue>= ON_LINE);
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
    float angle;
    if(quadrant == 1) {
        angle = atan(delY/delX) * 180 / M_PI;
    }
    else if(quadrant == 2) {
        angle = atan(delY/delX) * 180/M_PI;
        angle += 180;
    }
    else if(quadrant == 3) {
        angle = atan(delY/delX) * 180/M_PI;
        angle += 180;
    }
    else {
        angle = atan(delY/delX) * 180/M_PI;
        angle += 360;
    }
    return angle;
}

void faceLocation(float x, float y, int quadrant) {
    float angle = locationDegree(x, y, quadrant);
    float currentHeading = RPS.Heading();
    float deltaTheta = angleBetween(currentHeading, angle);
    float tempAngle = angle - currentHeading;
    if(tempAngle < 0) {
        tempAngle += 360;
    }
    if(tempAngle > 180) {
        turn_right(30, deltaTheta);
    }
    else {
        turn_left(30, deltaTheta);
    }
    LCD.WriteLine("Facing: ");
    LCD.Write(angle);
    faceDegree(angle);

}

void faceLocationBack(float x, float y, int quadrant) {
    float angle = locationDegree(x, y, quadrant);
    angle -= 180;
    if(angle < 0) {
        angle += 360;
    }
    LCD.WriteLine("Current Heading: ");
    float currentHeading = RPS.Heading();
    LCD.WriteLine(currentHeading);
    while(currentHeading < 0) {
        currentHeading = RPS.Heading();
    }

    float deltaTheta = angleBetween(currentHeading, angle);
    LCD.WriteLine("delta theta: ");
    LCD.WriteLine(deltaTheta);
    float tempAngle = angle - currentHeading;
    if(tempAngle < 0) {
        tempAngle += 360;
    }
    if(tempAngle > 180) {
        turn_right(30, deltaTheta);
    }
    else {
        turn_left(30, deltaTheta);
    }
    LCD.WriteLine("Facing: ");
    LCD.Write(angle);

}
void moveToForwards(float x, float y) {
    int quad;
    LCD.WriteLine(RPS.X());
    LCD.WriteLine(RPS.Y());
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
    LCD.WriteLine(distanceTo(x,y));
    move_forward(45, distanceTo(x, y));


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
    faceLocationBack(x, y, quad);
    move_backwards_timed(SPEED, distanceTo(x, y), 5);
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
    faceDegree(0);
    move_forward(SPEED , distanceTo(x, RPS.Y()));
    turn_right(20, 90);
    faceDegree(270);
    move_forward(35, distanceTo(RPS.X(), y));
}
/** waitForStart
    Initializes menu, waits for start light to go on.
*/
void waitForStart() {
    RPS.InitializeTouchMenu();
    LCD.Clear();
    setRPSCoords();
    while(cds2.Value() > 0.8);
}
/** getLightColor
    Returns the color of the fuel light.
    @return 1 if light is blue, 2 if light is red
*/
int getLightColor() {
    LCD.WriteLine(cds1.Value());
    if(cds1.Value() < BLUE_LIGHT_ON) {
        return 0;
    }
    else {
        return 1;
    }
}
/** detectingLight
    Finds out whether the robot is detecting a light or not
    @return true if robot is detecting light, false otherwise
*/
bool detectingLight(int cell) {
    if(cell == 1) {
        if(RPS.CurrentCourse() == 'a' || RPS.CurrentCourse() == 'A') {
            return cds1.Value() < 1.1;
        }
        else {
            return cds1.Value() < 1.1;
        }


    }
    else {
        return cds2.Value() < 0.8;
    }
}


/** setServo
    Sets the servo thresholds.
*/
void setServo() {
    arm.SetMin(884);
    arm.SetMax(2235);
}
void moveArm(float currentDegree, float nextDegree) {
    if(currentDegree < nextDegree) {
        while(currentDegree < nextDegree) {
            arm.SetDegree(currentDegree);
            currentDegree++;
            Sleep(5);
        }
    }
    else {
        while(currentDegree > nextDegree) {
            arm.SetDegree(currentDegree);
            currentDegree--;
            Sleep( 5);
        }
    }
}

/** pullSwitch
    pulls a switch in front of the robot
*/
void pullSwitch(int s) {
    if(s == 2) {
        move_backwards(SPEED, 1.5);
        moveArm(100, 35);

        move_backwards_timed(SPEED, 2, 1);
        //move_forward(20, 1);
         moveArm(35, 100);

    }
    else {
        move_forward_timed(SPEED, 1, 1);
         moveArm(100, 35);

        move_backwards_timed(SPEED, 2.5, 1);
        move_forward(SPEED, 0.5);
         moveArm(35, 100);
    }



}
/** pushSwitch
    pushes a switch in front of the robot
*/
void pushSwitch(int s) {
    if(s == 2) {
        move_backwards(30, 4);
         moveArm(100, 35);

        move_forward_timed(30, 3, .25);
         moveArm(35, 100);
    }
    else {
        move_backwards(30 , 2.5);
        moveArm(100, 35);
        move_forward_timed(30, 3, .25 );
        moveArm(35, 100);
    }
}
/** goUpSideRamp
    Assuming robot is facing ramp, moves up the side ramp, stopping when robot is completely on top level.
*/
void goUpSideRamp() {
    move_forward_timed(50, 5, 100);
    followLine(50, 7);
    driveToWall(30);
    move_backwards(50, 0.25);
    turn_left(30, 90);
    followLine(50, 30);
    driveToWall(30);
    move_backwards(35,1);

    turn_left(30, 90);
    LCD.WriteLine("FORWARD");

    move_forward_timed(SPEED, 15, 100);
    LCD.WriteLine("STOP");
    right_motor.Stop();
    left_motor.Stop();
}
/** flipSwitches
    Flips all 3 switches to their correct orientation
    @param red The direction for the red switch to go
    @param white The direction for the white switch to go
    @param blue The direction for the blue switch to go
*/
void flipSwitches(int red, int white, int blue) {
    //Starting at middle switch
    followLineYellowSquare(SPEED, 3);
    if(white == 1) {
        pushSwitch(2);
    }
    else {
        pullSwitch(2);
    }
    driveToWall(SPEED);
    move_forward_timed(30, 1, 0.5);
    move_backwards(30, 1);
    turn_right(30, 25);
    move_forward(30, 1);
    if(red == 1) {
        pushSwitch(1);
    }
    else {
        pullSwitch(1);
    }
    move_backwards(30, 0.5);
   turn_left(30, 20);
   driveToWall(SPEED);
   move_forward_timed(30, 1, 0.5);
   move_backwards(30, 1);
    turn_left(30, 25);
    //faceDegree(300);
    move_forward(30, 1);
    if(blue == 1) {
        pushSwitch(3);
    }
    else {
        pullSwitch(3);
    }
}
/** completeSwitches
    moves to switches and flips them
*/
void completeSwitches() {

    flipSwitches(RPS.RedSwitchDirection(), RPS.WhiteSwitchDirection(), RPS.BlueSwitchDirection());
}
/** pushButton
    pushes correct fuel button
*/
void pushButton(int correctButton) {

    if(correctButton == 0) {
        LCD.WriteLine("RED");
        move_backwards(SPEED, 4.5);
        moveArm(100, 33);
        move_forward_timed(20, 3, 1.5);
        move_forward_timed(5, 100, 5);
        move_backwards_timed(30,3, 2);
        arm.SetDegree(100);
    }
    else {
        LCD.WriteLine("BLUE");
        arm.SetDegree(120);
        move_forward_timed(30, 100, 6);
        move_backwards_timed(30, 3, 2);
        move_backwards_timed(30, 3, 2);
        arm.SetDegree(100);
    }
}
void wiggle() {
    turn_right(15, 5);
    turn_left(15, 5);
    turn_right(15, 5);
    turn_left(15, 5);
    turn_right(15, 5);
    turn_left(15, 5);
}
void pickUpSupplies() {


    moveArm(100, 15);
    LCD.WriteLine("moving arm down");
    moveArm(15, 100);
    LCD.WriteLine("Moving arm up");
}



void dropSupplies() {
    move_backwards(30, 1);
    LCD.WriteLine("moving backwards");
    moveArm(100, 25);
    LCD.WriteLine("moving arm down");
    move_backwards_timed(30, 5, 3);
    LCD.WriteLine("moving backwards");
    LCD.WriteLine("sleep");
    arm.SetDegree(100);
    LCD.WriteLine("arm up");

    pivot_right( 33, 175);
    followLineYellowSquare(20, 5);

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
    arm.SetDegree(100);
    moveToForwards(SUPPLIES_X, SUPPLIES_Y + 1.8);
    turn_right(30, angleBetween(RPS.Heading(), 270) - 1);
    faceDegree(270);
    check_y_minus(SUPPLIES_Y+1.2);


    pickUpSupplies();
}

void suppliesToTop() {
    while(RPS.X() < 0);
    move_backwards(35, distanceTo(RPS.X(), Location::BOTTOM_SIDE_RAMP_Y + 0.5)) ;
    turn_left(30,90);
    goUpSideRamp();

}
bool inYLightPostition() {
    if(RPS.Y() > 0) {
        return RPS.Y() <= Location::FUEL_LIGHT_Y;
    }
    else return true;
}

void goToLight() {
//    if(RPS.X() > 0) {
//        LCD.WriteLine(RPS.X());
//        LCD.WriteLine(distanceTo(RPS.X(), Location::FUEL_LIGHT_Y));
//        followLineYellow(25, distanceTo(RPS.X(), Location::FUEL_LIGHT_Y) - 0.3);
//    }
    move_forward_timed(30, 5, 5);
    double time = TimeNow();
    while(!detectingLight(1) && TimeNow() - time < 1.5) {
        followLineYellow(25, 0.1);
    }

    right_motor.Stop();
    left_motor.Stop();

    LCD.WriteLine(cds1.Value());
    Sleep(250);

    int correctButton = getLightColor();
    lightColor = correctButton;
    if(correctButton == 0) {
        LCD.WriteLine("RED");
    }
    else {
        LCD.WriteLine("BLUE");

    }
    pushButton(correctButton);
}

void doButtons() {
    while(RPS.X() < 0);
    //check_x_minus(Location::FUEL_LIGHT_X);
    if(RPS.Heading() >= 0) {
        turn_right(30, angleBetween(RPS.Heading(),91));
    }
    else {
        turn_right(30, 85);
    }
    if(RPS.X()>Location::FUEL_LIGHT_X) {
        check_x_minus(Location::FUEL_LIGHT_X);
    }
    faceDegree(91);

    goToLight();




}
void dropOff() {
    turn_right(30, 10);
    move_backwards(30, 2);
    if(RPS.Heading() < 0) {
        move_backwards(30, 1);
    }
    if(RPS.Heading() < 0) {
        move_backwards(30, 1);
    }
    while(RPS.Heading() < 0);
    float angle = locationDegree(4.8, DROP_OFF_Y, 3);
    angle -= 180;
    if(lightColor != 0) {
        moveToBackwards(DROP_OFF_X, DROP_OFF_Y -2);

    }
    else {
        moveToBackwards(DROP_OFF_X, DROP_OFF_Y );

    }
    turn_left(35, angleBetween(angle, 90)+1);
    LCD.WriteLine("FOLLOWING");
    driveToWall(30);
    LCD.WriteLine("DONE");
    move_forward_timed(SPEED, 1, 0.75);

    //followLineYellow(40, 10);
    //drop package
    dropSupplies();

}

void goHome() {
    if(RPS.X() < 0) {
        move_forward(30, 1);
    }
    if(RPS.X() < 0) {
        move_forward(30, 1);
    }

    turn_left(30, angleBetween(RPS.Heading(), 0));
    //faceDegree(0);
    move_forward(SPEED, distanceTo(Location::TOP_MAIN_RAMP_X - 2, RPS.Y()));
    //check_x_plus(Location::TOP_MAIN_RAMP_X-1);
    turn_right(30, 93);
   // faceDegree(270);

    move_forward_timed(35, 16.5, 3);
    Sleep(50);
        turn_left(30, 100);
        move_backwards(50, 17);
        faceLocationBack(0, 0, 3);
        move_backwards_timed(50, 100, 3);

        move_forward(SPEED, 5);
        faceLocationBack(0, 0, 3);
        move_backwards(50, 10);

}

void goGoGo() {
    startToSupplies();
    suppliesToTop();
    doButtons();
    dropOff();
    completeSwitches();
    goHome();
}


void cdsCheck() {
    while(true) {
        LCD.WriteLine(cds1.Value());
        Sleep(50);
        LCD.Clear();
    }
}

int main(void)
{
    setServo();
    arm.SetDegree(100);
    waitForStart();
    START_X = RPS.X();
    START_Y = RPS.Y();
    goGoGo();








    return 0;

}
