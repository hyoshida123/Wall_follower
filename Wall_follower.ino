
#include <PID_v1.h>

#define PIN_DISTANCE_LEFT A0
#define PIN_DISTANCE_CENTER A6
#define PIN_DISTANCE_RIGHT A7

#define PIN_LED1 A3
#define PIN_LED2 A2
#define PIN_BUTTON A1

#define PIN_ENCODER_LEFT_A 11
#define PIN_ENCODER_LEFT_B 2
#define PIN_ENCODER_RIGHT_A 12
#define PIN_ENCODER_RIGHT_B 3

#define PIN_MOTOR_LEFT_1 9
#define PIN_MOTOR_LEFT_2 10
#define PIN_MOTOR_RIGHT_1 5
#define PIN_MOTOR_RIGHT_2 6

// Invert encoder directions if needed
const boolean INVERT_ENCODER_LEFT = true;
const boolean INVERT_ENCODER_RIGHT = true;

// Invert motor directions if needed
const boolean INVERT_MOTOR_LEFT = true;
const boolean INVERT_MOTOR_RIGHT = false;

// Define Variables we'll be connecting to
double velocity_linear_setpoint = 200;
double velocity_linear = 0;
double velocity_linear_power = 0;

double velocity_angular_setpoint = 0;
double velocity_angular = 0;
double velocity_angular_power = 0;

double distance_left_setpoint = 18;
double distance_left = 0;

double left = 0;
double right = 0;

// Specify the links and initial tuning parameters
// Here, Kp = 0.006, Ki = 0.001, and Kd = 0
PID velocity_linear_pid(&velocity_linear, &velocity_linear_power, &velocity_linear_setpoint, 0.007, 0.0005, 0.0, DIRECT);
PID velocity_angular_pid(&velocity_angular, &velocity_angular_power, &velocity_angular_setpoint, 0.4, 0.05, 0.0, DIRECT);
PID distance_left_pid(&distance_left, &velocity_angular_setpoint, &distance_left_setpoint, 0.07, 0.03, 0.0, REVERSE);


void setup()
{
  Serial.begin(9600);
  velocity_linear_pid.SetOutputLimits(-1.0, 1.0);
  velocity_linear_pid.SetSampleTime(10);
  velocity_angular_pid.SetOutputLimits(-1.0, 1.0);
  velocity_angular_pid.SetSampleTime(10);
  distance_left_pid.SetOutputLimits(-10.0, 10.0);
  distance_left_pid.SetSampleTime(10);

  // Turn the PID loop on
  velocity_linear_pid.SetMode(AUTOMATIC);
  velocity_angular_pid.SetMode(AUTOMATIC);
  distance_left_pid.SetMode(AUTOMATIC);
  
  pinSetup();
}

void loop()
{
  velocity_linear = getLinearVelocity();
  velocity_angular = getAngularVelocity();
  distance_left = readDistanceLeft();
  
  velocity_linear_pid.Compute();
  velocity_angular_pid.Compute();
  distance_left_pid.Compute();
    
  left = velocity_linear_power - velocity_angular_power;
  right = velocity_linear_power + velocity_angular_power; 
  
  applyPowerLeft(left);
  applyPowerRight(right);

  checkEncodersZeroVelocity();
  updateDistanceSensors();
}

