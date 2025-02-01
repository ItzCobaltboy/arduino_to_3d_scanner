#include "Wire.h"
#include "VL53L0X.h"

// Define ToF XSHUT pins
#define SIDE_XSHUT_PIN 8
#define TOP_XSHUT_PIN 9

// L298N pins for turntable stepper
#define TURN_IN1 2
#define TURN_IN2 3
#define TURN_IN3 4
#define TURN_IN4 5

// L298N pins for vertical stepper
#define VERT_IN1 6
#define VERT_IN2 7
#define VERT_IN3 10
#define VERT_IN4 11

// Define ToF sensor addresses
#define SIDE_TOF_ADDRESS 0x30
#define TOP_TOF_ADDRESS 0x31

// Scanner parameters
const int STEPS_PER_REV = 2048;        // Steps per revolution for 28BYJ-48
const int STEP_SEQUENCE_COUNT = 8;      // 8-step sequence
const float MAX_HEIGHT = 100.0;         // Maximum height in mm
const float HEIGHT_STEP = 2.0;          // Height increment per step in mm
const float ANGLE_STEP = 2.0;           // Angle increment in degrees
const int SETTLING_TIME = 100;          // Time to wait after movement in ms
const int SAMPLES_PER_READING = 3;      // Number of readings to average

// Stepper motor sequence (8-step sequence)
const byte stepSequence[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

// Initialize ToF sensors
VL53L0X sideSensor;
VL53L0X topSensor;

// Current position tracking
float currentHeight = 0.0;
float currentAngle = 0.0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Initialize all pins
  initializePins();
  
  // Configure ToF sensors
  setupTofSensors();
  
  Serial.println("3D Scanner Ready");
  printMenu();
}

void initializePins() {
  // ToF XSHUT pins
  pinMode(SIDE_XSHUT_PIN, OUTPUT);
  pinMode(TOP_XSHUT_PIN, OUTPUT);
  
  // Turntable stepper pins
  pinMode(TURN_IN1, OUTPUT);
  pinMode(TURN_IN2, OUTPUT);
  pinMode(TURN_IN3, OUTPUT);
  pinMode(TURN_IN4, OUTPUT);
  
  // Vertical stepper pins
  pinMode(VERT_IN1, OUTPUT);
  pinMode(VERT_IN2, OUTPUT);
  pinMode(VERT_IN3, OUTPUT);
  pinMode(VERT_IN4, OUTPUT);
}

void setupTofSensors() {
  // Reset both sensors
  digitalWrite(SIDE_XSHUT_PIN, LOW);
  digitalWrite(TOP_XSHUT_PIN, LOW);
  delay(10);
  
  // Initialize Side Sensor
  digitalWrite(SIDE_XSHUT_PIN, HIGH);
  delay(10);
  if (!sideSensor.init()) {
    Serial.println("Failed to initialize side sensor!");
  }
  sideSensor.setAddress(SIDE_TOF_ADDRESS);
  
  // Initialize Top Sensor
  digitalWrite(TOP_XSHUT_PIN, HIGH);
  delay(10);
  if (!topSensor.init()) {
    Serial.println("Failed to initialize top sensor!");
  }
  topSensor.setAddress(TOP_TOF_ADDRESS);
  
  // Start continuous mode
  sideSensor.startContinuous();
  topSensor.startContinuous();
}

void printMenu() {
  Serial.println("Commands:");
  Serial.println("1 - Start Full Scan");
  Serial.println("2 - Start Side Scan Only");
  Serial.println("3 - Start Top Scan Only");
  Serial.println("h - Return to Home Position");
  Serial.println("s - Stop Current Operation");
  Serial.println("? - Show this menu");
}

void moveStepperMotor(int in1, int in2, int in3, int in4, int steps, int direction) {
  static int currentStep = 0;
  
  for (int i = 0; i < abs(steps); i++) {
    if (direction > 0) {
      currentStep = (currentStep + 1) % STEP_SEQUENCE_COUNT;
    } else {
      currentStep = (currentStep - 1 + STEP_SEQUENCE_COUNT) % STEP_SEQUENCE_COUNT;
    }
    
    digitalWrite(in1, stepSequence[currentStep][0]);
    digitalWrite(in2, stepSequence[currentStep][1]);
    digitalWrite(in3, stepSequence[currentStep][2]);
    digitalWrite(in4, stepSequence[currentStep][3]);
    
    delay(2); // Adjust for speed control
  }
  
  // Turn off coils
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void rotateTurntable(float targetAngle) {
  float angleDiff = targetAngle - currentAngle;
  int steps = (angleDiff / 360.0) * STEPS_PER_REV;
  moveStepperMotor(TURN_IN1, TURN_IN2, TURN_IN3, TURN_IN4, steps, steps > 0 ? 1 : -1);
  currentAngle = targetAngle;
}

void moveVertical(float targetHeight) {
  float heightDiff = targetHeight - currentHeight;
  // Convert height difference to steps (adjust based on your mechanical setup)
  int steps = (heightDiff / 8.0) * STEPS_PER_REV; // Assuming 8mm per full revolution
  moveStepperMotor(VERT_IN1, VERT_IN2, VERT_IN3, VERT_IN4, steps, steps > 0 ? 1 : -1);
  currentHeight = targetHeight;
}

uint16_t getAveragedReading(VL53L0X &sensor) {
  uint32_t sum = 0;
  
  for (int i = 0; i < SAMPLES_PER_READING; i++) {
    sum += sensor.readRangeContinuousMillimeters();
    delay(10);
  }
  
  return sum / SAMPLES_PER_READING;
}

void performFullScan() {
  Serial.println("Starting full scan...");
  Serial.println("Format: SensorID,Height(mm),Distance(mm),Angle(deg)");
  
  float height = 0;
  while (height <= MAX_HEIGHT) {
    moveVertical(height);
    delay(SETTLING_TIME);
    
    for (float angle = 0; angle < 360; angle += ANGLE_STEP) {
      rotateTurntable(angle);
      delay(SETTLING_TIME);
      
      // Get readings from both sensors
      uint16_t sideDist = getAveragedReading(sideSensor);
      uint16_t topDist = getAveragedReading(topSensor);
      
      // Output readings
      // Side sensor (ID=1)
      Serial.print("1,");
      Serial.print(height, 2);
      Serial.print(",");
      Serial.print(sideDist);
      Serial.print(",");
      Serial.println(angle, 2);
      
      // Top sensor (ID=2)
      Serial.print("2,");
      Serial.print(height, 2);
      Serial.print(",");
      Serial.print(topDist);
      Serial.print(",");
      Serial.println(angle, 2);
      
      // Check for stop command
      if (Serial.available() && Serial.read() == 's') {
        Serial.println("Scan stopped by user");
        return;
      }
    }
    
    height += HEIGHT_STEP;
  }
  
  Serial.println("Scan complete");
}

void performSideScan() {
  Serial.println("Starting side scan...");
  Serial.println("Format: SensorID,Height(mm),Distance(mm),Angle(deg)");
  
  float height = 0;
  while (height <= MAX_HEIGHT) {
    moveVertical(height);
    delay(SETTLING_TIME);
    
    for (float angle = 0; angle < 360; angle += ANGLE_STEP) {
      rotateTurntable(angle);
      delay(SETTLING_TIME);
      
      uint16_t sideDist = getAveragedReading(sideSensor);
      
      Serial.print("1,");
      Serial.print(height, 2);
      Serial.print(",");
      Serial.print(sideDist);
      Serial.print(",");
      Serial.println(angle, 2);
      
      if (Serial.available() && Serial.read() == 's') {
        Serial.println("Scan stopped by user");
        return;
      }
    }
    
    height += HEIGHT_STEP;
  }
  
  Serial.println("Side scan complete");
}

void performTopScan() {
  Serial.println("Starting top scan...");
  Serial.println("Format: SensorID,Height(mm),Distance(mm),Angle(deg)");
  
  for (float angle = 0; angle < 360; angle += ANGLE_STEP) {
    rotateTurntable(angle);
    delay(SETTLING_TIME);
    
    uint16_t topDist = getAveragedReading(topSensor);
    
    Serial.print("2,");
    Serial.print("0,"); // Height is fixed for top sensor
    Serial.print(topDist);
    Serial.print(",");
    Serial.println(angle, 2);
    
    if (Serial.available() && Serial.read() == 's') {
      Serial.println("Scan stopped by user");
      return;
    }
  }
  
  Serial.println("Top scan complete");
}

void homePosition() {
  Serial.println("Moving to home position...");
  moveVertical(0);
  delay(SETTLING_TIME);
  rotateTurntable(0);
  currentHeight = 0;
  currentAngle = 0;
  Serial.println("Home position reached");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case '1':
        performFullScan();
        break;
      case '2':
        performSideScan();
        break;
      case '3':
        performTopScan();
        break;
      case 'h':
        homePosition();
        break;
      case 's':
        Serial.println("Stop command received");
        break;
      case '?':
        printMenu();
        break;
      default:
        Serial.println("Unknown command");
        printMenu();
        break;
    }
  }
}
