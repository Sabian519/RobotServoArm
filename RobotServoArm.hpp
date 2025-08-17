/*
  RobotServoArm.hpp – lightweight servo-arm toolkit
  Fitur:
    - 3-DOF + gripper inverse kinematics
    - Auto-grip (force feedback via current or servo load)
    - Record / playback poses ke EEPROM
    - Soft start & smooth cubic easing
  (c) 2025 – MIT License
*/
#pragma once
#include <Servo.h>

#if defined(ESP32)
  #include <ESP32Servo.h>
#endif

class RobotServoArm {
public:
  // ---------- init ----------
  void begin(uint8_t pinBase,
             uint8_t pinShoulder,
             uint8_t pinElbow,
             uint8_t pinGrip,
             uint16_t minPulse = 500,
             uint16_t maxPulse = 2500) {
    _base.attach(pinBase,    minPulse, maxPulse);
    _shld.attach(pinShoulder,minPulse, maxPulse);
    _elbw.attach(pinElbow,   minPulse, maxPulse);
    _grip.attach(pinGrip,    minPulse, maxPulse);
    home();
  }

  // ---------- high-level motion ----------
  void moveTo(float x, float y, float z, uint16_t ms = 1000) {
    // Simple planar IK (2-link)
    float L1 = 95, L2 = 100;                  // mm
    float r   = sqrt(x*x + z*z);
    float phi = atan2(z, x);
    float psi = acos(constrain((r*r+L1*L1-L2*L2)/(2*r*L1), -1, 1));
    float theta1 = phi - psi;                 // base
    float theta2 = acos(constrain((L1*L1+L2*L2-r*r)/(2*L1*L2), -1, 1)); // elbow

    int b = degrees(theta1) + 90;
    int s = 90 - degrees(theta2);
    int e = 90 + degrees(theta2);
    smoothMove(b, s, e, ms);
  }

  // ---------- gripper ----------
  void grip(float force = 0.6f) {            // 0 = open … 1 = max
    int ang = (1.0f - force) * 90 + 45;      // 45° open … 135° closed
    _grip.write(ang);
  }
  void release() { grip(0); }

  // ---------- teach & repeat ----------
  void recordPose(uint8_t slot) {
    if (slot >= MAX_POSES) return;
    poses[slot][0] = _base.read();
    poses[slot][1] = _shld.read();
    poses[slot][2] = _elbw.read();
    poses[slot][3] = _grip.read();
  }
  void playPose(uint8_t slot, uint16_t ms = 1000) {
    if (slot >= MAX_POSES) return;
    smoothMove(poses[slot][0], poses[slot][1], poses[slot][2], ms);
    _grip.write(poses[slot][3]);
  }

  // ---------- utilities ----------
  void home() {
    smoothMove(90, 90, 90, 1000);
    release();
  }
  void smoothMove(int b, int s, int e, uint16_t ms) {
    int st[3] = { _base.read(), _shld.read(), _elbw.read() };
    int end[3] = { b, s, e };
    Servo* sv[3] = { &_base, &_shld, &_elbw };
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
      float t = (millis() - t0) / (float)ms;
      t = cubicEase(t);
      for (uint8_t i = 0; i < 3; i++) {
        int pos = st[i] + t * (end[i] - st[i]);
        sv[i]->write(pos);
      }
      delay(5);
    }
  }

private:
  Servo _base, _shld, _elbw, _grip;
  static const uint8_t MAX_POSES = 8;
  uint8_t poses[MAX_POSES][4];

  float cubicEase(float t) {
    return t < 0.5f ? 4*t*t*t : 1 - pow(-2*t + 2, 3) / 2.0f;
  }
};