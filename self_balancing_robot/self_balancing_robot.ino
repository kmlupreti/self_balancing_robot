#include "MPU6050.h"
#include "PID_v1.h"
#include "LMotorController.h"

//####################################################
// PIN DEFINITIONS
//####################################################
#define IN1 11
#define IN2 10
#define ENA 12
#define IN3 9 
#define IN4 8
#define ENB 7

//####################################################
// PARAMETERS FOR  FINE TUNING
//####################################################

// PID variables
#define Kp 10
#define Kd 0.5
#define Ki 0.1

// left and right motor settings
#define MOTOR_SPEED_LIMIT 200 // set between 0 to 255
#define LEFT_MOTOR_FACTOR 1  // factor by which left motor speed is multiplied
#define RIGHT_MOTOR_FACTOR 1 // factor by which right motor speed is multiplied

// Constants for complementary filter
#define ALPHA 0.96
#define SAMPLE_TIME 10  // Sample time in milliseconds


//####################################################
// OTHER VARIABLES AND DEFINITIONS
//####################################################

// PID variables
double setpoint = 0, input, output;

// define PID control
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//define mpu sensor
MPU6050 mpu;

//setup motor controller
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, LEFT_MOTOR_FACTOR,RIGHT_MOTOR_FACTOR);

// Variables for angle calculation
float pitch = 0;
float gyroRate = 0;
unsigned long lastTime = 0;

void setup() {
  // uncomment line below to enable serial monitor
  // Serial.begin(9600);

  // Begin I2C communication
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();

  // Set manual data offsets obtained after callibration
  // mpu.setXAccelOffset(-4577);
  // mpu.setYAccelOffset(-753);
  // mpu.setZAccelOffset(16382);
  // mpu.setXGyroOffset(67);
  // mpu.setYGyroOffset(-31);
  // mpu.setZGyroOffset(91);

  // Setup PID control
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-MOTOR_SPEED_LIMIT, MOTOR_SPEED_LIMIT);
  pid.SetSampleTime(SAMPLE_TIME);
}

void loop() {
  unsigned long currentTime = millis();

  // Ensure consistent sample time
  if (currentTime - lastTime >= SAMPLE_TIME) {
    // Read sensor data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Calculate pitch angle using complementary filter
    float accPitch = atan2(ay, az) * 180 / PI;

    // uncomment lines below to show pitch angle in serial monitor
    // Serial.print("pitch: \t");
    // Serial.println(accPitch);

    gyroRate = (float)gx / 131.0;  // Convert to degrees per second
    pitch = ALPHA * (pitch + gyroRate * (currentTime - lastTime) / 1000.0) + (1 - ALPHA) * accPitch;

    // Update PID input
    input = pitch;
    pid.Compute();

    //don't correct when when pitch angle is very small
    if (abs(pitch) < 3)
      output = 0;

    // don't move if pitch angle is very high as its not possible to recover
    if (abs(pitch) > 45)
      output = 0;

    // move motors with calculated output
    motorController.move(output);

    lastTime = currentTime;
  }
}
