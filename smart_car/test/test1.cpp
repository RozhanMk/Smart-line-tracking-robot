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

void test_motorMovement() {
    goForward();
    assert(digitalRead(IN1) == HIGH);
    assert(digitalRead(IN2) == LOW);
    assert(digitalRead(IN3) == HIGH);
    assert(digitalRead(IN4) == LOW);
  	delay(2000);

    turnLeft();
    assert(digitalRead(IN1) == LOW);
    assert(digitalRead(IN2) == HIGH);
    assert(digitalRead(IN3) == HIGH);
    assert(digitalRead(IN4) == LOW);
    delay(2000);

    turnRight();
    assert(digitalRead(IN1) == HIGH);
    assert(digitalRead(IN2) == LOW);
    assert(digitalRead(IN3) == LOW);
    assert(digitalRead(IN4) == HIGH);
    delay(2000);

    stopMotors();
    assert(digitalRead(IN1) == LOW);
    assert(digitalRead(IN2) == LOW);
    assert(digitalRead(IN3) == LOW);
    assert(digitalRead(IN4) == LOW);
    delay(2000);
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
  
  	test_motorMovement();
  	Serial.println("test passed!");
}

void loop() {
    
}