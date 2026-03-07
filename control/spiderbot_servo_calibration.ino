#include <ESP32Servo.h>

// ===== Same pin mapping as your spider bot =====
const int servo_pin[4][3] = {
  {18, 14, 13},   // Leg 0
  {26, 25, 27},   // Leg 1
  {32, 33, 23},   // Leg 2
  {21, 22, 19}    // Leg 3
};

Servo servo[4][3];

void setup() {
  Serial.begin(115200);
  Serial.println("=== Servo 90° Calibration ===");

  // Allocate PWM timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach all servos and immediately set to 90°
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      servo[leg][joint].setPeriodHertz(50);
      servo[leg][joint].attach(servo_pin[leg][joint], 500, 2400);
      delay(100);
      servo[leg][joint].write(90);
      Serial.printf("Leg %d  Joint %d  → 90°  (GPIO %d)\n",
                    leg, joint, servo_pin[leg][joint]);
    }
    delay(200);
  }

  Serial.println("\nAll 12 servos set to 90°.");
  Serial.println("Physically attach your servo horns NOW.");
  Serial.println("Do NOT power off until horns are secured.");
}

void loop() {
  // Keep writing 90° every 500ms so servos hold position
  // even if anything tries to move them.
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++)
      servo[leg][joint].write(90);

  delay(500);
}
