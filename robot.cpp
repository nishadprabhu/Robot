#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#define ON_LINE 1.9
#define CENTER 0
#define LEFT -1
#define FAR_LEFT -2
#define RIGHT 1
#define FAR_RIGHT 2
#define COUNTS_PER_INCH 40.5
#define COUNTS_PER_DEGREE 2.389
//Declarations for encoders & motors
ButtonBoard buttons(FEHIO::Bank3);
DigitalEncoder right_encoder(FEHIO::P0_0);
DigitalEncoder left_encoder(FEHIO::P0_1);
FEHMotor right_motor(FEHMotor::Motor1,12.0);
FEHMotor left_motor(FEHMotor::Motor0,12.0);

AnalogInputPin right(FEHIO::P1_2);
AnalogInputPin middle(FEHIO::P1_4);
AnalogInputPin left(FEHIO::P1_6);
AnalogInputPin cds(FEHIO::P1_0);
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
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}
void followLine(float speed) {
        int leftValue, rightValue, midValue, state;
        while( true )
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

            switch(state) {
                case CENTER:
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(speed);
                    LCD.Write(leftValue);
                    LCD.Write(rightValue);
                    LCD.Write(midValue);
                    break;
                case LEFT:
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(.75*speed);
                    LCD.Write(leftValue);
                    LCD.Write(rightValue);
                    LCD.Write(midValue);
                    break;
                case FAR_LEFT:
                    left_motor.SetPercent(speed);
                    right_motor.SetPercent(.5*speed);
                    LCD.Write(leftValue);
                    LCD.Write(rightValue);
                    LCD.Write(midValue);
                    break;
                case RIGHT:
                    left_motor.SetPercent(.75*speed);
                    right_motor.SetPercent(speed);
                    LCD.Write(leftValue);
                    LCD.Write(rightValue);
                    LCD.Write(midValue);
                    break;
                case FAR_RIGHT:
                    left_motor.SetPercent(.5*speed);
                    right_motor.SetPercent(speed);
                    LCD.Write(leftValue);
                    LCD.Write(rightValue);
                    LCD.Write(midValue);
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
//Runs through performance test 1
void performance1() {

    Sleep(2.0); //Wait for counts to stabilize
    move_forward(30, 21);
    Sleep(1.0);
    turn_right(20, 45);
    Sleep(1.0);
    move_forward(30, 23);
    Sleep(1.0);
    turn_left(20, 90);
    Sleep(1.0);
    move_forward(30, 27 );
    Sleep(1.0);
    turn_left(20, 90);
    left_motor.SetPercent(30);
    right_motor.SetPercent(30);
    while(middle.Value() > 1.2);
    left_motor.Stop();
    right_motor.Stop();
    turn_right(20,135);
    followLine(10);
}

int main(void)
{

    while(!buttons.MiddlePressed()); //Wait for middle button to be pressed
    while(buttons.MiddlePressed()); //Wait for middle button to be unpressed
    performance1();


    return 0;
}
