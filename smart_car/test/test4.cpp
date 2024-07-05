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

#define assert(condition) customAssert(condition, __func__)

void customAssert(bool condition, const char* function_name) {
    if (!condition) {
        Serial.print("Assertion failed in ");
        Serial.println(function_name);
        while (true) {
        }
    }
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

void loop(){
    long distance = readDistance();
    Serial.println(distance);
    delay(200);
}