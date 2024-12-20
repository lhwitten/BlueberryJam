const int SOL1_PIN = 7;
const int SOL2_PIN = 6;

#define DR_MOTOR_ENA 10
#define DR_MOTOR_IN1 8
#define DR_MOTOR_IN2 9

// const int LIGHTS_PIN = 0;


void setup() {
  // setup code, to run once:
  pinMode(SOL1_PIN, OUTPUT);
  pinMode(SOL2_PIN, OUTPUT);
  // pinMode(LIGHTS_PIN, OUTPUT);
  
  Serial.begin(9600); // Initialize serial communication for debugging

  pinMode(DR_MOTOR_ENA, OUTPUT);
  pinMode(DR_MOTOR_IN1, OUTPUT);
  pinMode(DR_MOTOR_IN2, OUTPUT);
  analogWrite(DR_MOTOR_ENA, 190);
  digitalWrite(DR_MOTOR_IN1, LOW);
  digitalWrite(DR_MOTOR_IN2, HIGH);
  delay(1000);
}

void loop() {
  // main code here, to run repeatedly:
//
//  // digitalWrite(LIGHTS_PIN, LOW);
//
//  digitalWrite(SOL1_PIN, HIGH);
//  Serial.println("solenoid 1 on");
//  delay(1000);
//  digitalWrite(SOL1_PIN, LOW);
//  Serial.println("solenoid 1 off");
//  
//  //delay(1000);  // delay for 1 sec
//
//  digitalWrite(SOL2_PIN, HIGH);
//  Serial.println("solenoid 2 on");
//  delay(1000);
//  digitalWrite(SOL2_PIN, LOW);
//  Serial.println("solenoid 2 off");
//
//  delay(1000);  // delay for 1 sec

}
