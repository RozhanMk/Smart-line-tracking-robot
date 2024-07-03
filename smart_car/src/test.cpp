#include <Arduino.h>
#include <functions.h>
#define assert(condition) customAssert(condition, __func__)

void customAssert(bool condition, const char* function_name) {
    if (!condition) {
        Serial.print("Assertion failed in ");
        Serial.println(function_name);
        while (true) {
        }
    }
}
void test_motorMovement() {

    goForward();
    assert(digitalRead(IN1) == HIGH);
    assert(digitalRead(IN2) == LOW);
    assert(digitalRead(IN3) == HIGH);
    assert(digitalRead(IN4) == LOW);

    turnLeft();
    assert(digitalRead(IN1) == LOW);
    assert(digitalRead(IN2) == HIGH);
    assert(digitalRead(IN3) == HIGH);
    assert(digitalRead(IN4) == LOW);

    turnRight();
    assert(digitalRead(IN1) == HIGH);
    assert(digitalRead(IN2) == LOW);
    assert(digitalRead(IN3) == LOW);
    assert(digitalRead(IN4) == HIGH);

    stopMotors();
    assert(digitalRead(IN1) == LOW);
    assert(digitalRead(IN2) == LOW);
    assert(digitalRead(IN3) == LOW);
    assert(digitalRead(IN4) == LOW);
}