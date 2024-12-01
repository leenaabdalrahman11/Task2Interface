#include <LiquidCrystal.h>

// Define LCD Pins
LiquidCrystal lcd(4, 5, 6, 7, 8, A1); // RS, E, D4, D5, D6, D7

// Define Motor Pins
#define IN1 9
#define IN2 10
#define ENABLE 11

// Define Encoder Pins
#define ENCODER_A 2
#define ENCODER_B 3

// Define Joystick Pin
#define JOYSTICK_PIN A0

// Encoder resolution (pulses per revolution)
#define PPR 360 // Adjust this value based on your encoder's specification

// Variables for Encoder
volatile long encoderCount = 0;
float motorTurns = 0.0;
int motorSpeed = 0;

// Interrupt Service Routine for Encoder
void encoderISR() {
  int stateA = digitalRead(ENCODER_A);
  int stateB = digitalRead(ENCODER_B);
  if (stateA == stateB) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(9600);

  // Initialize motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  // Initialize encoder pins
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  // Initialize joystick pin
  pinMode(JOYSTICK_PIN, INPUT);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();
}

void loop() {
  // Read joystick and map to motor speed
  int joystickValue = analogRead(JOYSTICK_PIN);
  motorSpeed = map(joystickValue, 0, 1023, -255, 255);

  // Set motor direction and speed
  if (motorSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (motorSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENABLE, abs(motorSpeed));

  // Calculate the number of motor turns
  motorTurns = float(encoderCount) / PPR; // Divide encoder counts by PPR to get turns

  // Update LCD
  lcd.setCursor(0, 0);
  lcd.print("Count: ");
  lcd.print(encoderCount);
  lcd.setCursor(0, 1);
  lcd.print("Turns: ");
  lcd.print(motorTurns, 3); // Display turns with 3 decimal places

  // Print encoder data to Serial Monitor
  Serial.print("Encoder Count: ");
  Serial.print(encoderCount);
  Serial.print(" Turns: ");
  Serial.println(motorTurns);

  delay(100);
}
