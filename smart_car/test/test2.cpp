#include <LiquidCrystal_I2C.h>
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
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
int machineEnergy = 100.0;
long prevEnergyUpdateTime = 0;

void customAssert(bool condition, const char* function_name) {
    if (!condition) {
        Serial.print("Assertion failed in ");
        Serial.println(function_name);
        while (true) {
        }
    }
}

void decreaseEnergy() {
    int decreaseRate = 1; 
    machineEnergy -= decreaseRate;

    if(machineEnergy < 0) {
        machineEnergy = 0;
    }
}

void test_energyManagement() {
    
    decreaseEnergy();
    assert(machineEnergy == 99);  //check if energy decreased correctly

    machineEnergy = 0;
    decreaseEnergy();
    assert(machineEnergy == 0);  
}

void setup()
{
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.setCursor(3,0);
  
}

void loop()
{
    int currentTime = millis();
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

    delay(10);
}