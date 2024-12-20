#include <CircularBuffer.hpp>

// Encoder pins
const int encoderPinA = 2; // Connect A to pin 2
const int encoderPinB = 3; // Connect B to pin 3
const int encoder5v = 4;// should be 4
const int motor5v = 5;

volatile int encoderPosition = 0; // Variable to store encoder position
volatile int lastEncoded = 0;     // To keep track of the last encoded value

CircularBuffer<float, 12> speedBuffer; // Circular buffer for 4 speed samples
float filteredSpeed = 0;             // Smoothed speed value

unsigned long lastTime = 0;          // Last time the speed was calculated
int lastPosition = 0;                // Encoder position during the last check

// Motor pins
#define DR_MOTOR_ENA 10
#define DR_MOTOR_IN1 8
#define DR_MOTOR_IN2 9

// Constants
const int posPerRev = 2381;//600;          // Encoder positions per revolution
const float BELT_RAD_IN = 1.0445; // Belt radius in inches
const int controlInterval = 150;   // Control interval in milliseconds
unsigned long lastControlTime = 0; // Last time the control was applied

// Speed control
float goalInchesPerSec = 2.5; // Desired speed in inches per second
int currentPWM = 85;         // Starting PWM value
const int pwmStep = 2;        // Step size for increasing/decreasing PWM
const int maxPWM = 255;       // Maximum PWM value
const int minPWM = 0;         // Minimum PWM value

unsigned long last_print = 0;

void setup() {
  // Motor setup
  pinMode(DR_MOTOR_ENA, OUTPUT);
  pinMode(DR_MOTOR_IN1, OUTPUT);
  pinMode(DR_MOTOR_IN2, OUTPUT);

  // Encoder setup
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoder5v, OUTPUT);
  pinMode(motor5v, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
  digitalWrite(encoder5v, LOW);
  digitalWrite(motor5v, LOW);
  
  Serial.begin(9600);

  // Start the motor
  analogWrite(DR_MOTOR_ENA, currentPWM);
  digitalWrite(DR_MOTOR_IN1, LOW);
  digitalWrite(DR_MOTOR_IN2, HIGH);
}

void loop() {
  unsigned long currentTime = millis();

  // Update encoder properties (filtered speed)
  if (currentTime - lastTime >= 4) {
    updateEncoderProperties();
  }

  // Perform iterative speed control at regular intervals
  if (currentTime - lastControlTime >= controlInterval) {
    lastControlTime = currentTime;
    iterativeControl();
  }
}

// Update encoder speed and calculate filtered speed
void updateEncoderProperties() {
  unsigned long currentTime = millis();
  int positionChange = encoderPosition - lastPosition;
  float positionsPerSecond = positionChange / ((currentTime - lastTime) / 1000.0);

  // Add to the buffer for smoothing
  speedBuffer.push(positionsPerSecond);
  filteredSpeed = 0;
  for (int i = 0; i < speedBuffer.size(); i++) {
    filteredSpeed += speedBuffer[i];
  }
  filteredSpeed /= speedBuffer.size();

  // Convert to inches per second
  filteredSpeed = posPerSec2InPerSec(filteredSpeed);

  // Update last values
  lastTime = currentTime;
  lastPosition = encoderPosition;

  // Debugging output
  
//  Serial.print("Filtered Speed (in/sec): ");
//  Serial.println(filteredSpeed);
}

// Iterative speed control logic
void iterativeControl() {
  int currentPWMStep = pwmStep;
  if (abs(filteredSpeed - goalInchesPerSec) <.9){
      currentPWMStep /= 2;
    }
  
  if (abs(filteredSpeed - goalInchesPerSec) <.1){
    currentPWM = currentPWM;
  }
  else if (filteredSpeed < goalInchesPerSec) {
    // If speed is too low, increase PWM
    currentPWM += currentPWMStep;
    if (currentPWM > maxPWM) {
      currentPWM = maxPWM;
    }
  } else if (filteredSpeed > goalInchesPerSec) {
    // If speed is too high, decrease PWM
    currentPWM -= currentPWMStep;
    if (currentPWM < minPWM) {
      currentPWM = minPWM;
    }
  }

  // Apply the new PWM value
  analogWrite(DR_MOTOR_ENA, currentPWM);

  // Debugging output

  float interval = 300; //ms
  if (millis() - last_print > interval){
  Serial.print("Goal Speed: ");
  Serial.print(goalInchesPerSec);
  Serial.print(" | Filtered Speed: ");
  Serial.print(filteredSpeed);
  Serial.print(" | PWM Value: ");
  Serial.println(currentPWM);
  last_print = millis();
  }
}

// Convert encoder positions per second to inches per second
float posPerSec2InPerSec(float posPerSec) {
  float revPerSec = posPerSec / posPerRev;
  float inchPerRev = 2 * PI * BELT_RAD_IN;
  return inchPerRev * revPerSec;
}

// ISR to update encoder position
void updateEncoder() {
  int MSB = digitalRead(encoderPinA); // Most Significant Bit
  int LSB = digitalRead(encoderPinB); // Least Significant Bit

  int encoded = (MSB << 1) | LSB; // Combine A and B signals
  int sum = (lastEncoded << 2) | encoded; // Track transitions

  // Determine rotation direction and increment/decrement position
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPosition++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPosition--;
  }

  lastEncoded = encoded; // Update last encoded value
}
