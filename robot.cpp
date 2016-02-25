//Including FEH libraries
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>

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
#define COUNTS_PER_DEGREE 1.9444
//Define thresholds for line following/start light
#define START_LIGHT_ON 1.5
#define BLUE_LIGHT_ON 1
//Tuning constant
#define K X
//Declarations for encoders & motors
ButtonBoard buttons(FEHIO::Bank3);
DigitalEncoder right_encoder(FEHIO::P0_1);
DigitalEncoder left_encoder(FEHIO::P0_2);
FEHMotor right_motor(FEHMotor::Motor1,12.0);
FEHMotor left_motor(FEHMotor::Motor0,12.0);

AnalogInputPin right(FEHIO::P1_2);
AnalogInputPin middle(FEHIO::P1_4);
AnalogInputPin left(FEHIO::P1_6);
AnalogInputPin cds(FEHIO::P1_0);

DigitalInputPin frontLeftBump(FEHIO::P2_0);
DigitalInputPin frontRightBump(FEHIO::P2_1);
AnalogInputPin backLeftBump(FEHIO::P0_0);
AnalogInputPin backRightBump(FEHIO::P0_0);
void move_forward(int percent, float inches) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    float counts = inches*COUNTS_PER_INCH;
    //Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts) {
        right_motor.SetPercent(K*(left_encoder.Counts()-right_encoder.Counts())+percent);
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
//Drives either forwards or backwards until it hits a wall.
void driveToWall(int percent) {

    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);
    LCD.WriteLine(frontLeftBump.Value());
    LCD.WriteLine(frontRightBump.Value());
    while(frontLeftBump.Value() && frontRightBump.Value());
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
    float counts = degrees * COUNTS_PER_DEGREE;
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-1 * percent);

    while((right_encoder.Counts() + left_encoder.Counts())/2 < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}
void turn_right(int percent, float degrees) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    float counts = degrees * COUNTS_PER_DEGREE;
    right_motor.SetPercent(-1 * percent);
    left_motor.SetPercent(percent);

    while((right_encoder.Counts() + left_encoder.Counts())/2 < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}
void turn(int percent, int counts) {
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    right_motor.SetPercent(-1 * percent);
    left_motor.SetPercent(percent);

    while((right_encoder.Counts() + left_encoder.Counts())/2 < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void waitForStart() {
    while(!buttons.MiddlePressed()); //Wait for middle button to be pressed
    while(buttons.MiddlePressed()); //Wait for middle button to be unpressed
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
    driveToWall(30);
    Sleep(1.0);
    move_backwards(20, 1);
    Sleep(1.0);
    turn_left(20, 90);
    Sleep(1.0);
    driveToWall(30);
    Sleep(1.0);
    move_backwards(30,1);
    turn_left(20, 90);
    Sleep(1.0);
    move_forward(30, 18);
    Sleep(1.0);
    turn_right(20,90);
    Sleep(1.0);
    move_forward(30, 15);
    if(getLightColor == 1) {
        LCD.SetBackgroundColor(LCD.Blue);
    }
    else if(getLightColor == 0) {
        LCD.SetBackgroundColor(LCD.Red);
    }
    else {
        LCD.WriteLine("No light detected");
    }
}

int main(void)
{
    waitForStart();
    performance1();



    return 0;
}
