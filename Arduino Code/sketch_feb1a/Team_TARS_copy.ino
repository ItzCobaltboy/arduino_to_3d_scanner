#include "Wire.h"
#include "VL53L0X.h"
#include "AccelStepper.h"
#include "Servo.h"

// Define ToF XSHUT pins
#define SIDE_XSHUT_PIN 8
#define TOP_XSHUT_PIN 9

// Motor pins
// Turntable stepper (Motor 1)
#define TURN_IN1 2
#define TURN_IN2 3
#define TURN_IN3 4
#define TURN_IN4 5

// Vertical servo
const int SERVO_PIN = 6;

// Define ToF sensor addresses
#define SIDE_TOF_ADDRESS 0x30


// Scanner parameters
const int STEPS_PER_REV = 2048;    // For 28BYJ-48 stepper
const float MAX_HEIGHT = 80.0;     // Maximum height in mm (12cm)
const float HEIGHT_STEP = 8.0;      // Height increment per step in mm
const float ANGLE_STEP = 20.0;      // Angle increment in degrees
const int SETTLING_TIME = 50;       // Time to wait after movement in ms
const int SAMPLES_PER_READING = 5;  // Number of readings to average
const int ZERO_ERR = 50;

// Servo parameters
const int SERVO_MIN_ANGLE = 0;      // Minimum servo angle (bottom position)
const int SERVO_MAX_ANGLE = 180;    // Maximum servo angle (top position)

// Scaling factors for data transmission
const float SCALE_FACTOR = 1;  // Magic number for scaling
const char START_MARKER = '<';
const char END_MARKER = '>';

// Initialize motors
AccelStepper turntableStepper(AccelStepper::FULL4WIRE, TURN_IN1, TURN_IN2, TURN_IN3, TURN_IN4);
Servo verticalServo;

// Initialize ToF sensors
VL53L0X sideSensor;
VL53L0X topSensor;

// Current position tracking
float currentHeight = 0.0;
float currentAngle = 0.0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  setupMotors();
  initializePins();
  setupTofSensors();
  
  // Start scanning automatically
  delay(1000);  // Give everything time to initialize
  performScan();
}

void setupMotors() {
  // Setup turntable stepper
  turntableStepper.setMaxSpeed(500);
  turntableStepper.setAcceleration(200);
  turntableStepper.setSpeed(200);
  
  // Setup vertical servo - start at top position (180 degrees = height 0)
  verticalServo.attach(SERVO_PIN);
  verticalServo.write(SERVO_MAX_ANGLE);
  delay(1000); // Give servo time to reach starting position
}

void initializePins() {
  pinMode(SIDE_XSHUT_PIN, OUTPUT);
  pinMode(TOP_XSHUT_PIN, OUTPUT);
}

void setupTofSensors() {
  digitalWrite(SIDE_XSHUT_PIN, LOW);
  digitalWrite(TOP_XSHUT_PIN, LOW);
  delay(10);
  
  digitalWrite(SIDE_XSHUT_PIN, HIGH);
  delay(10);
  if (!sideSensor.init()) {
    Serial.println("ERR:SIDE_SENSOR");
    return;
  }
  sideSensor.setAddress(SIDE_TOF_ADDRESS);
  


  
  sideSensor.startContinuous();

}

void publishData(int sensorId, float height, uint16_t distance, float angle) {
  Serial.flush();
  delay(20);

  // if (distance >= 170)
  //  {distance = -1;}

  String dataString = String(START_MARKER) + 
                     String(sensorId) + "," +
                     String(height) + "," +
                     String(distance) + "," +
                     String(angle) + 
                     String(END_MARKER);

  Serial.print(dataString);
  delay(50);
}

uint16_t getAveragedReading(VL53L0X &sensor) {
  uint32_t sum = 0;
  for (int i = 0; i < SAMPLES_PER_READING; i++) {
    sum += sensor.readRangeContinuousMillimeters();
    delay(1);
  }
  return (sum / SAMPLES_PER_READING) * 0.9;
  //return sensor.readRangeContinuousMillimeters();
}

void moveToPosition(float targetHeight, float targetAngle) {
  // Convert height to servo angle (180° = 0mm, 0° = 120mm)
  int servoAngle = SERVO_MAX_ANGLE - (int)((targetHeight / MAX_HEIGHT) * SERVO_MAX_ANGLE);
  servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  verticalServo.write(servoAngle);
  
  // Move turntable
  long angleSteps = (targetAngle / 360.0) * STEPS_PER_REV;
  turntableStepper.moveTo(angleSteps);
  
  while (turntableStepper.distanceToGo() != 0) {
    turntableStepper.run();
  }
  
  currentHeight = targetHeight;
  currentAngle = targetAngle;
  delay(SETTLING_TIME); // Allow servo to reach position
}

void performScan() {
  float height = 0;
  while (height <= MAX_HEIGHT) {
    moveToPosition(height, 0);
    delay(100);  // Additional settling time
    
    for (float angle = 0; angle < 360; angle += ANGLE_STEP) {
      moveToPosition(height, angle);
      delay(50);  // Settling time after rotation
      
      uint16_t sideDist = getAveragedReading(sideSensor);
      publishData(1, height, sideDist, angle);
      delay(20);  // Extra delay between data points
    }
    height += HEIGHT_STEP;
  }
  
  // Return to home position (top)
  moveToPosition(0, 0);
}

void loop() {
  // Empty loop - everything is done in setup
}