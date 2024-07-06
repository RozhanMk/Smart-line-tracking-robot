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
#define LIGHT_SENSOR A3
#define CAR_LIGHT 13

float fMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void updateCarLight() {
    float outsideLight = analogRead(LIGHT_SENSOR);
    float carLightVoltage = 255 - fMap(outsideLight, 6, 679, 0, 255);
    Serial.print("Outside Light: ");
    Serial.println(outsideLight);
    Serial.print("Car Light Voltage: ");
    Serial.println(carLightVoltage);
    analogWrite(CAR_LIGHT, carLightVoltage);
}

void setup() {
    Serial.begin(9600);

    /****Motor drivers setup***/
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB1, OUTPUT);
    pinMode(ENB2, OUTPUT);
    pinMode(LIGHT_SENSOR, INPUT);
    pinMode(CAR_LIGHT, OUTPUT);
    analogWrite(ENB1, 255); // turn on
    analogWrite(ENB2, 255); // turn on
  
}

void loop() {
  updateCarLight();
  delay(200);
}