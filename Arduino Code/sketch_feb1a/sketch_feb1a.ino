#include <Wire.h>
#include <Stepper.h>
#include <Adafruit_VL53L0X.h>

///////////////////////////// CONFIG //////////////////////////////////////
const int verticalSensorAddress = 0x29; // I2C address for vertical VL53L0X sensor (default address)
const int horizontalSensorAddress = 0x30; // I2C address for horizontal VL53L0X sensor (change address if needed)
const int verticalMotorPin1 = 3; // Stepper motor pin 1 (Vertical) connected to ULN2003 IN1
const int verticalMotorPin2 = 4; // Stepper motor pin 2 (Vertical) connected to ULN2003 IN2
const int verticalMotorPin3 = 5; // Stepper motor pin 3 (Vertical) connected to ULN2003 IN3
const int verticalMotorPin4 = 6; // Stepper motor pin 4 (Vertical) connected to ULN2003 IN4
const int horizontalMotorPin1 = 7; // Stepper motor pin 1 (Horizontal) connected to ULN2003 IN1
const int horizontalMotorPin2 = 8; // Stepper motor pin 2 (Horizontal) connected to ULN2003 IN2
const int horizontalMotorPin3 = 9; // Stepper motor pin 3 (Horizontal) connected to ULN2003 IN3
const int horizontalMotorPin4 = 10; // Stepper motor pin 4 (Horizontal) connected to ULN2003 IN4
const int maxVerticalSteps = 1000; // Maximum steps for vertical motor
const int maxHorizontalSteps = 1000; // Maximum steps for horizontal motor

///////////////////////////////////////////////////////////////////////////

// Initialize stepper motors
Stepper verticalMotor(2048, verticalMotorPin1, verticalMotorPin2, verticalMotorPin3, verticalMotorPin4); // 2048 steps per revolution
Stepper horizontalMotor(2048, horizontalMotorPin1, horizontalMotorPin2, horizontalMotorPin3, horizontalMotorPin4); // 2048 steps per revolution

// Initialize VL53L0X LIDAR sensors
Adafruit_VL53L0X verticalSensor = Adafruit_VL53L0X();
Adafruit_VL53L0X horizontalSensor = Adafruit_VL53L0X();

void setup() {
  // Start Serial for debugging
  Serial.begin(115200);
  
  // Initialize I2C communication
  Wire.begin();
  
  // Set the speed for the motors (adjust as needed)
  verticalMotor.setSpeed(15);  // Speed in RPM (adjust as needed)
  horizontalMotor.setSpeed(15); // Speed in RPM (adjust as needed)

  // Initialize VL53L0X sensors
  if (!verticalSensor.begin()) {
    Serial.println("Failed to initialize vertical VL53L0X sensor!");
    while (1);
  }
  
  if (!horizontalSensor.begin()) {
    Serial.println("Failed to initialize horizontal VL53L0X sensor!");
    while (1);
  }
  
  // Calibration or additional setup for VL53L0X (if necessary)
}

void loop() {
  // Read distance from the vertical sensor
  uint16_t verticalDistance = readDistance(verticalSensor);
  
  // Read distance from the horizontal sensor
  uint16_t horizontalDistance = readDistance(horizontalSensor);

  // For demonstration, we move the motors based on sensor readings
  // Adjust movement logic based on actual sensor data and desired behavior
  
  // Move vertical motor based on vertical sensor value
  int verticalSteps = map(verticalDistance, 0, 2000, 0, maxVerticalSteps); // Adjust range as needed
  verticalMotor.step(verticalSteps); 

  // Move horizontal motor based on horizontal sensor value
  int horizontalSteps = map(horizontalDistance, 0, 2000, 0, maxHorizontalSteps); // Adjust range as needed
  horizontalMotor.step(horizontalSteps);

  // Add delay to avoid excessive updates
  delay(100);
}

// Function to read distance from the VL53L0X sensor
uint16_t readDistance(Adafruit_VL53L0X &sensor) {
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);  // Pass in 'true' for debug data
  
  if (measure.RangeStatus != 4) {  // Check if there is a valid measurement
    return measure.RangeMilliMeter; // Return the measured distance in millimeters
  } else {
    return 0;  // Return 0 if the measurement failed
  }
}
