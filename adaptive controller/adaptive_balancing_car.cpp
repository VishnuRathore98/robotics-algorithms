/* 
* Arduino-ready adaptive balance controller for a two-wheeled balancing robot (Segway-style). 
* It’s written in C++ for the Arduino ecosystem, uses an MPU6050 IMU for tilt and angular rate, 
* and drives two DC motors via PWM + direction pins.
*/

/*
  Adaptive Balancing Robot (Arduino)

  - IMU: MPU6050 (Jeff Rowberg's library assumed)
  - Motor driver: PWM + 2 direction pins per motor (adjust pins to your wiring)
  - Control: PD + adaptive gravity compensation
  - Discrete adaptation updates for m_hat and J_hat

  Notes:
  - Tune Kp, Kd, gamma_m, gamma_J, and safety limits carefully!
  - Use a motor driver capable of required current.
  - For production, implement current sensing, state observer, and safety kill switch.

  Required library:
   - "MPU6050" from Jeff Rowberg (I2Cdevlib). Or replace IMU read code with your IMU's lib.
*/

#include <Wire.h>
#include "MPU6050.h"    // Jeff Rowberg's library (MPU6050.h + I2Cdev.h). Install before use.

// --- Pins (example, change to your wiring) ---
const int MOTOR_L_PWM = 5;    // PWM pin left motor
const int MOTOR_L_DIR = 4;    // Direction pin left motor
const int MOTOR_R_PWM = 6;    // PWM pin right motor
const int MOTOR_R_DIR = 7;    // Direction pin right motor
const int ENABLE_PIN = 8;     // Optional enable / kill pin

// --- Physical constants (reasonable starting guesses) ---
const float g = 9.81f;     // gravity (m/s^2)
const float l = 0.12f;     // distance from wheel axle to COM (m) - tune for your robot

// --- Controller gains (start conservative, then tune) ---
float Kp = 60.0f;    // proportional (angle)
float Kd = 1.8f;     // derivative (angular rate)

// --- Adaptation gains (small, stable values) ---
float gamma_m = 0.2f;   // mass adaptation rate
float gamma_J = 0.05f;  // inertia adaptation rate

// --- Initial parameter estimates (controller's knowledge) ---
float m_hat = 0.8f;   // initial estimated mass (kg)
float J_hat = 0.8f * l * l;  // initial estimated inertia (kg m^2)

// --- Safety / limits ---
const float MAX_TORQUE = 2.5f;   // maximum torque (N*m) to send to motors (map to PWM)
const float MAX_PWM = 255.0f;
const float MIN_PWM = -255.0f;
const float ANGLE_SAFETY_LIMIT = 0.8f; // radians (~45 degrees) - if exceeded, shut down motors

// --- IMU object ---
MPU6050 imu;
float ax, ay, az;
float gx, gy, gz;

// --- Complementary filter / state variables ---
float theta = 0.0f;     // tilt angle (rad), upright = 0
float omega = 0.0f;     // angular rate (rad/s)
float dt = 0.01f;       // sampling time (s) - will be measured
unsigned long lastTime = 0;

// Complementary filter parameter
const float alpha = 0.98f;

// --- Helper functions for motors ---
void driveMotors(float torqueCommand) {
  // torqueCommand in [-MAX_TORQUE, MAX_TORQUE]
  // Map torque to PWM. This mapping depends on your motor, gearbox, battery, wheel radius.
  // For a simple approximation, assume linear: pwm = torque / MAX_TORQUE * 255
  float pwmFloat = (torqueCommand / MAX_TORQUE) * MAX_PWM;
  if (pwmFloat > MAX_PWM) pwmFloat = MAX_PWM;
  if (pwmFloat < -MAX_PWM) pwmFloat = -MAX_PWM;

  int pwmVal = (int)abs(pwmFloat);
  pwmVal = constrain(pwmVal, 0, (int)MAX_PWM);

  if (pwmFloat >= 0) {
    // forward
    digitalWrite(MOTOR_L_DIR, HIGH);
    digitalWrite(MOTOR_R_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_L_DIR, LOW);
    digitalWrite(MOTOR_R_DIR, LOW);
  }

  analogWrite(MOTOR_L_PWM, pwmVal);
  analogWrite(MOTOR_R_PWM, pwmVal);
}

void stopMotors() {
  analogWrite(MOTOR_L_PWM, 0);
  analogWrite(MOTOR_R_PWM, 0);
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1) { delay(1000); }
  } else {
    Serial.println("MPU6050 connected");
  }

  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_R_DIR, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  lastTime = micros();
}

// Simple function to get angle (theta) using accel + gyro complementary filter
void readIMU(float &outTheta, float &outOmega, float dtLocal) {
  // Read raw accel/gyro
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Convert accelerometer to angle around X axis (assuming pitch axis)
  // ax, ay, az in raw units. Convert to g: accel / 16384 (for default +/-2g) if using MPU6050 default scale
  float ax_g = ax / 16384.0f;
  float az_g = az / 16384.0f;

  // angle from accel
  float angleAcc = atan2(ax_g, az_g); // radians - approximate small-angle pitch

  // Gyro rate in deg/s: raw / 131.0 for default FS=250deg/s
  float gyroRateDeg = gy / 131.0f;
  float gyroRateRad = gyroRateDeg * (PI / 180.0f);

  // Complementary filter
  outTheta = alpha * (outTheta + gyroRateRad * dtLocal) + (1 - alpha) * angleAcc;
  outOmega = (1 - alpha) * outOmega + alpha * gyroRateRad; // smoothed omega
}

void loop() {
  // compute dt
  unsigned long now = micros();
  dt = (now - lastTime) / 1e6;
  if (dt <= 0 || dt > 0.05) dt = 0.01; // clamp dt if reading glitch
  lastTime = now;

  // Read IMU and update theta, omega
  float theta_prev = theta;
  float omega_prev = omega;
  // Use previous values to feed into read function (complementary filter)
  readIMU(theta, omega, dt);

  // Safety: if robot tilted too far, kill motors and wait for manual reset
  if (fabs(theta) > ANGLE_SAFETY_LIMIT) {
    stopMotors();
    Serial.println("Angle exceeded safety limit. Motors stopped.");
    while (1) { delay(100); } // block - require reset
  }

  // Error (we want theta -> 0)
  float e = theta;             // tilt error
  float edot = omega;          // angular velocity

  // Control torque (N*m) - PD + adaptive gravity compensation
  float u = -Kp * e - Kd * edot + m_hat * g * l * sin(theta);

  // Saturate torque
  if (u > MAX_TORQUE) u = MAX_TORQUE;
  if (u < -MAX_TORQUE) u = -MAX_TORQUE;

  // Apply torque to motors (map to PWM)
  driveMotors(u);

  // Estimate angular acceleration (alpha) by finite difference
  float alpha_est = (omega - omega_prev) / dt;

  // Discrete adaptation updates (Euler)
  // m_hat_dot = -gamma_m * e * sin(theta)
  // J_hat_dot = -gamma_J * e * alpha
  float m_hat_dot = -gamma_m * e * sin(theta);
  float J_hat_dot = -gamma_J * e * alpha_est;

  m_hat += m_hat_dot * dt;
  J_hat += J_hat_dot * dt;

  // Enforce sensible bounds to keep estimates stable
  if (m_hat < 0.1f) m_hat = 0.1f;
  if (m_hat > 10.0f) m_hat = 10.0f;
  if (J_hat < 0.001f) J_hat = 0.001f;
  if (J_hat > 10.0f) J_hat = 10.0f;

  // Debug print at lower rate
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("t(s): "); Serial.print(millis() / 1000.0);
    Serial.print(" theta: "); Serial.print(theta, 3);
    Serial.print(" omega: "); Serial.print(omega, 3);
    Serial.print(" u(Nm): "); Serial.print(u, 3);
    Serial.print(" m_hat: "); Serial.print(m_hat, 3);
    Serial.print(" J_hat: "); Serial.println(J_hat, 4);
    lastPrint = millis();
  }
}
