#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#define LEFTENC 2 //interrupt
#define RIGHTENC 3 //interrupt
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6
#define ENB1 10
#define ENB2 11
#define USONIC 4
#define ALERT_LED 12
#define LEFTSENSOR A0
#define CenterSENSOR A1
#define RIGHTSENSOR A2
#define LIGHT_SENSOR A3
#define CAR_LIGHT 13

extern double machineEnergy;

void updateSpeed();
void leftEncoderISR();
void rightEncoderISR();
long readDistance();
void decreaseEnergy();
void goForward();
void turnRight();
void turnLeft();
void stopMotors();
void updateCarLight();

#endif