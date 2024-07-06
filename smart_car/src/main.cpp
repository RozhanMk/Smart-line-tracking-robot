#include "functions.h"
#include <Arduino.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>

double tilt = 0;

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

int machineEnergy = 100;  //initial energy level
LiquidCrystal_I2C lcd(0x27,20,4);
long prevEnergyUpdateTime = 0;
long prevSpeedUpdateTime = 0;

void setup() {
    /****Motor drivers setup***/
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB1, OUTPUT);
    pinMode(ENB2, OUTPUT);
    pinMode(LIGHT_SENSOR, INPUT);
    pinMode(CAR_LIGHT, OUTPUT);
    pinMode(TILT_SENSOR, INPUT);
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

    pinMode(USONIC, OUTPUT);

    pinMode(ALERT_LED, OUTPUT);

    lcd.init();                     
    lcd.backlight();

    Serial.begin(9600);
    
}

void loop() {
    int leftValue = analogRead(LEFTSENSOR);
    int centerValue = analogRead(CenterSENSOR);
    int rightValue = analogRead(RIGHTSENSOR);

    long distance = readDistance();

    if (distance < 30) {    //30cm
        stopMotors();
    }
    else{
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
    }

    long currentTime = millis();
    if(currentTime - prevSpeedUpdateTime >= 100) {
        updateSpeed();
        prevSpeedUpdateTime = currentTime;
    }

    currentTime = millis();
    if(currentTime - prevEnergyUpdateTime >= 1000) {
        decreaseEnergy();
        prevEnergyUpdateTime = currentTime;
    }
    if(machineEnergy <= 15){    // low charge
        digitalWrite(ALERT_LED, HIGH);
    }

    //display energy on lcd
    lcd.setCursor(0, 0);
    lcd.print("Energy: ");
    lcd.print(machineEnergy);
    lcd.print("%");
  
    checkEnvironment();

    delay(10);
}

void updateSpeed(){
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

long readDistance() {
    pinMode(USONIC, OUTPUT);
    digitalWrite(USONIC, LOW);
    delayMicroseconds(2);
    digitalWrite(USONIC, HIGH);
    delayMicroseconds(10);
    digitalWrite(USONIC, LOW);

    pinMode(USONIC, INPUT);
    long duration = pulseIn(USONIC, HIGH);
    long distance = duration * 0.034 / 2;

    return distance;
}

void decreaseEnergy() {
    int decreaseRate = 1; 
    machineEnergy -= decreaseRate;

    if(machineEnergy < 0) {
        machineEnergy = 0;
    }
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

float fMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readOutsideLight() {
    float outsideLight = analogRead(LIGHT_SENSOR);
    Serial.print("Outside Light: ");
    Serial.println(outsideLight);

    return outsideLight;
}

void updateCarLight() {
    float outsideLight = readOutsideLight();
    float carLightVoltage = 255 - fMap(outsideLight, 6, 679, 0, 255);

    Serial.print("Car Light Voltage: ");
    Serial.println(carLightVoltage);

    analogWrite(CAR_LIGHT, carLightVoltage);
}

void readTilt() {
    tilt = digitalRead(TILT_SENSOR);
    Serial.print("Tilt: ");
    Serial.println(tilt);
}

void checkEnvironment() {
    updateCarLight();
    readTilt();
}