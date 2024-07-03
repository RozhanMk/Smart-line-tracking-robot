#include <Arduino.h>
#include <PID_v1.h>
#include <TimerOne.h>
#define LEFTENC 2 //interrupt
#define RIGHTENC 3 //interrupt
#define IN1 9
#define IN2 8
#define IN3 8
#define IN4 8
#define ENB1 10
#define ENB2 11
#define LEFTSENSOR A0
#define CenterSENSOR A1
#define RIGHTSENSOR A2

void timerISR();
void leftEncoderISR();
void rightEncoderISR();
void goForward();
void turnRight();
void turnLeft();
void stopMotors();

double interval = 100000; //100ms

volatile long leftCount = 0;    //left encoder
volatile long rightCount = 0;   //right encoder

double setPoint = 100;
double leftInput, leftOutput;   //left motor

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID leftPID(&leftInput, &leftOutput, &setPoint, Kp, Ki, Kd, DIRECT);

double rightInput, rightOutput;   //right motor
PID rightPID(&rightInput, &rightOutput, &setPoint, Kp, Ki, Kd, DIRECT);


void setup() {
    /****Motor drivers setup***/
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB1, OUTPUT);
    pinMode(ENB2, OUTPUT);
    analogWrite(ENB1, 255); // turn on
    analogWrite(ENB2, 255); // turn on

    /****Encoders setup****/
    pinMode(LEFTENC, INPUT);
    pinMode(RIGHTENC, INPUT);
    attachInterrupt(digitalPinToInterrupt(LEFTENC), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHTENC), rightEncoderISR, RISING);

    /****PID setup****/
    leftPID.SetMode(AUTOMATIC);
    leftPID.SetOutputLimits(0, 255);
    rightPID.SetMode(AUTOMATIC);
    rightPID.SetOutputLimits(0, 255);

    /****IR Sensors setup****/
    pinMode(LEFTSENSOR, INPUT);
    pinMode(CenterSENSOR, INPUT);
    pinMode(RIGHTSENSOR, INPUT);

    Timer1.initialize(interval);
    Timer1.attachInterrupt(timerISR);

    Serial.begin(9600);
}

void loop() {
    int leftValue = analogRead(LEFTSENSOR);
    int centerValue = analogRead(CenterSENSOR);
    int rightValue = analogRead(RIGHTSENSOR);

    //line tracker
    if (centerValue > 500) { //center sensor On line
        goForward();
    }
    else if (leftValue > 500) { //left sensor on line
        turnLeft();
    }
    else if (rightValue > 500) { //right sensor on line
        turnRight();
    }
    else { //stop
        stopMotors();
    }

    delay(10);
}

void timerISR(){
    //calculate speed
    leftInput = (leftCount * (1000.0 / (interval / 1000.0)));
    rightInput = (rightCount * (1000.0 / (interval / 1000.0)));

    //reset counts
    leftCount = 0;
    rightCount = 0;

    //compute pid
    leftPID.Compute();
    rightPID.Compute();

    //change motors speed
    analogWrite(ENB1, leftOutput);
    analogWrite(ENB2, rightOutput);
}
void leftEncoderISR() {
  leftCount++;
}

void rightEncoderISR() {
  rightCount++;
}

void goForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void turnLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
