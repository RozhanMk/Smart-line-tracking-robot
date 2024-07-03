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
void test_energyManagement() {
    machineEnergy = 100.0;
    
    decreaseEnergy();
    assert(machineEnergy == 99.9);  //check if energy decreased correctly

    machineEnergy = 0.0;
    decreaseEnergy();
    assert(machineEnergy == 0.0);  
}

void test_distanceSensor() {
    
    long distance = readDistance();
    assert(distance >= 0 && distance <= 400); //example range for HC-SR04
}

void setup() {
    Serial.begin(9600);

    test_motorMovement();
    test_energyManagement();
    test_distanceSensor();

    Serial.println("all tests passed.");
}

void loop() {

}
