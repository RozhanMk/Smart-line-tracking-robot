#include <Arduino.h>
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

void test_lineTracker(){
    int leftValue = analogRead(LEFTSENSOR);
    int centerValue = analogRead(CenterSENSOR);
    int rightValue = analogRead(RIGHTSENSOR);
    Serial.println("left:");
    Serial.println(leftValue);

    Serial.println("center:");
    Serial.println(centerValue);

    Serial.println("right:");
    Serial.println(rightValue);
  	delay(200);
}

void setup() {
    Serial.begin(9600);
  
  	pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB1, OUTPUT);
    pinMode(ENB2, OUTPUT);
    analogWrite(ENB1, 255); // turn on
    analogWrite(ENB2, 255); // turn on
  
}

void loop() {
  test_lineTracker();
}