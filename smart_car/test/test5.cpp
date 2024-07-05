#define LEFTENC 2 
#define RIGHTENC 3
volatile long leftCount = 0;    
volatile long rightCount = 0;  
void setup() {
    Serial.begin(9600);
    pinMode(LEFTENC, INPUT);
    pinMode(RIGHTENC, INPUT); 
    
    attachInterrupt(digitalPinToInterrupt(LEFTENC), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHTENC), rightEncoderISR, RISING);
}

void loop() {
    
    digitalWrite(LEFTENC, LOW);
    delay(100);
    digitalWrite(LEFTENC, HIGH);
    delay(100);

    digitalWrite(RIGHTENC, LOW);
    delay(100);
    digitalWrite(RIGHTENC, HIGH);
    delay(100);
}

void leftEncoderISR() {
    Serial.println("Left Encoder ISR");
    leftCount++;
}

void rightEncoderISR() {
    Serial.println("Right Encoder ISR");
    rightCount++;
}
