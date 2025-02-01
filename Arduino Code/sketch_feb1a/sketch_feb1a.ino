#include "Wire.h"
#include "VL53L0X.h"
#include "AccelStepper.h"

// Define ToF XSHUT pins
#define SIDE_XSHUT_PIN 8
#define TOP_XSHUT_PIN 9

// Motor pins for L298N
// Turntable stepper (Motor 1)
#define TURN_IN1 2
#define TURN_IN2 3
#define TURN_IN3 4
#define TURN_IN4 5

// Vertical stepper (Motor 2)
#define VERT_IN1 6
#define VERT_IN2 7
#define VERT_IN3 10
#define VERT_IN4 11

// Define ToF sensor addresses
#define SIDE_TOF_ADDRESS 0x30
#define TOP_TOF_ADDRESS 0x31

// Scanner parameters
const int STEPS_PER_REV = 2048;    // For 28BYJ-48 stepper
const float MAX_HEIGHT = 100.0;     // Maximum height in mm
const float HEIGHT_STEP = 4.0;      // Height increment per step in mm
const float ANGLE_STEP = 2.0;       // Angle increment in degrees
const int SETTLING_TIME = 50;       // Time to wait after movement in ms
const int SAMPLES_PER_READING = 3;  // Number of readings to average

// Initialize steppers for 28BYJ-48
AccelStepper turntableStepper(AccelStepper::FULL4WIRE, TURN_IN1, TURN_IN2, TURN_IN3, TURN_IN4);
AccelStepper verticalStepper(AccelStepper::FULL4WIRE, VERT_IN1, VERT_IN2, VERT_IN3, VERT_IN4);

// Initialize ToF sensors
VL53L0X sideSensor;
VL53L0X topSensor;

// Current position tracking
float currentHeight = 0.0;
float currentAngle = 0.0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  setupSteppers();
  initializePins();
  setupTofSensors();
  
  // Start scanning automatically
  delay(1000);  // Give everything time to initialize
  performScan();
}

void setupSteppers() {
  turntableStepper.setMaxSpeed(500);
  turntableStepper.setAcceleration(200);
  turntableStepper.setSpeed(200);
  
  verticalStepper.setMaxSpeed(500);
  verticalStepper.setAcceleration(200);
  verticalStepper.setSpeed(200);
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
  
  digitalWrite(TOP_XSHUT_PIN, HIGH);
  delay(10);
  if (!topSensor.init()) {
    Serial.println("ERR:TOP_SENSOR");
    return;
  }
  topSensor.setAddress(TOP_TOF_ADDRESS);
  
  sideSensor.startContinuous();
  topSensor.startContinuous();
}

// Scaling factors for data transmission
const long SCALE_FACTOR = 1;  // Magic number for scaling
const char START_MARKER = '<';
const char END_MARKER = '>';

void publishData(int sensorId, float height, uint16_t distance, float angle) {
  // Clear output buffer
  Serial.flush();
  
  delay(250);

  // Scale floating point values to integers
  long scaledHeight = (long)(height * SCALE_FACTOR);
  long scaledAngle = (long)(angle * SCALE_FACTOR);
  
  // Create complete string first
  String dataString = String(START_MARKER) + 
                     String(sensorId) + "," +
                     String(scaledHeight) + "," +
                     String(distance) + "," +
                     String(scaledAngle) + 
                     String(END_MARKER);

  // Send the complete string at once
  Serial.print(dataString);
  
  // Small delay to ensure complete transmission
  delay(250);
}

// [Rest of the scanner code remains the same]
uint16_t getAveragedReading(VL53L0X &sensor) {
  uint32_t sum = 0;
  for (int i = 0; i < SAMPLES_PER_READING; i++) {
    sum += sensor.readRangeContinuousMillimeters();
    delay(2);
  }
  return sum / SAMPLES_PER_READING;
}

void moveSteppers(float targetHeight, float targetAngle) {
  long heightSteps = (targetHeight / 8.0) * STEPS_PER_REV;
  verticalStepper.moveTo(heightSteps);
  
  long angleSteps = (targetAngle / 360.0) * STEPS_PER_REV;
  turntableStepper.moveTo(angleSteps);
  
  while (verticalStepper.distanceToGo() != 0 || turntableStepper.distanceToGo() != 0) {
    verticalStepper.run();
    turntableStepper.run();
  }
  
  currentHeight = targetHeight;
  currentAngle = targetAngle;
}

void performScan() {
  float height = 0;
  while (height <= MAX_HEIGHT) {
    moveSteppers(height, 0);
    delay(100);  // Increased settling time
    
    for (float angle = 0; angle < 360; angle += ANGLE_STEP) {
      moveSteppers(height, angle);
      delay(50);  // Settling time after rotation
      
      uint16_t sideDist = getAveragedReading(sideSensor);
      publishData(1, height, sideDist, angle);
      delay(20);  // Extra delay between data points
    }
    height += HEIGHT_STEP;
  }
  
  // Move back to start
  moveSteppers(0, 0);
  delay(1000);
  
  // Top Sensor Scan
  height = 0;
  while (height <= MAX_HEIGHT) {
    moveSteppers(height, 0);
    delay(SETTLING_TIME);
    
    for (float angle = 0; angle < 360; angle += ANGLE_STEP) {
      moveSteppers(height, angle);
      delay(SETTLING_TIME);
      
      uint16_t topDist = getAveragedReading(topSensor);
      publishData(2, height, topDist, angle);
    }
    height += HEIGHT_STEP;
  }
  
  // Return to home position
  moveSteppers(0, 0);
}

void loop() {
  // Empty loop - everything is done in setup
}