// Some basic helper functions to move Robot Car / Arm 

void closeClaw(int timeToClose) {
  int stepCount = timeToClose / 10;
  int stepSize = abs(clawOpenPosition - clawClosePosition) / stepCount;
  for (int i = clawOpenPosition; i < clawClosePosition; i += stepSize) {
    servo.setServo(clawServo, i);
    delay(9);
  }
  servo.setServo(clawServo, clawClosePosition);
}

void rotateArmSlowly(float start, float end) {
  float stepSize = 0.4;
  if (start < end) {
    for (float i = start; i < end; i += stepSize) {
      rotateArm(i);
      delay(9);
    }
  } else {
    for (float i = start; i > end; i -= stepSize) {
      rotateArm(i);
      delay(9);
    }
  }
  rotateArm(end);
}

void moveRobotForward(int distance, int speed, bool blocking) {
  int steps = (float)distance / (42.0 * PI) * 4080.0;
  setMotor(2, speed, steps);
  setMotor(1, speed, steps);
  if (blocking)
    while (isMotorSpinning());

}

void moveRobotForwardMM(int distance) {
  moveRobotForward(distance, 10, true);
}

void turnRobotDegree(int degrees, int speed, bool blocking ) {
  float distancePerWheel = ((float)abs(degrees) / 180.0 )   * (72.0 * PI) / 2.0;
  int steps = (float)distancePerWheel / (42.0 * PI) * 4080.0;
  if (degrees > 0) {
    setMotor(2, speed, steps);
    setMotor(1, -speed, steps);
  } else {
    setMotor(2, -speed, steps);
    setMotor(1, speed, steps);
  }
  if (blocking)
    while (isMotorSpinning());
}

void turnRobotDegrees(int degrees) {
  turnRobotDegree(degrees, 10, true);
}

void rotateArm(float rotation) {
  rotation = constrain(rotation, -45.0, 45.0);
  rotation = map(rotation * 10, -45 * 10, 45 * 10, rotationServoP45Pos, rotationServoN45Pos);
  servo.setServo(rotationServo, rotation);
}
