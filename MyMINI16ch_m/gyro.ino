#include <MyMINI_Pro.h>
#include <MyMINI_ProInternal.h>
#include <MotorDriver.h>
#include <MotionHeading.h>

#include <math.h>


constexpr float GYRO_RATE_MIN_DT_SECONDS = 0.004f;//--------------------------------
constexpr float GYRO_RATE_MAX_DT_SECONDS = 0.060f;
constexpr float GYRO_TURN_TOLERANCE_DEG = 2.0f;
constexpr float GYRO_TURN_APPROACH_DEG = 12.0f;//----------------------------------
constexpr uint8_t GYRO_TURN_MIN_SPEED = 10u;
constexpr uint8_t GYRO_TURN_APPROACH_MIN_SPEED = 6u;
constexpr float GYRO_TURN_APPROACH_MAX_SPEED = 18.0f;
constexpr uint32_t GYRO_TURN_TIMEOUT_MS = 8000UL;//---------------------------

// ---------------------------------------------------------------------------

// Exact tuning used by the recovered fw_gyror/bw_gyror code.
constexpr uint32_t GYROR_TIMEOUT_MS = 15000UL;
// Keep the established forward launch.  Reverse needs a softer launch to
// prevent a left/right sway before the gyro correction settles.
constexpr uint32_t GYROR_FORWARD_ACCEL_TIME_MS = 300UL;
constexpr uint32_t GYROR_BACKWARD_ACCEL_TIME_MS = 400UL;
constexpr float GYROR_MM_PER_SECOND_AT_100 = 900.0f;
constexpr float GYROR_DECEL_DISTANCE_MM = 120.0f;
constexpr float GYROR_MIN_DECEL_FACTOR = 0.25f;
constexpr float GYROR_CORRECTION_SIGN = -1.0f;
constexpr float GYROR_FORWARD_KD = 0.00850f * 2.5f;
// Extra reverse damping: slows the yaw before it swings through target.
constexpr float GYROR_BACKWARD_KD = 0.0125f * 2.7f;
// Reverse at full strength when far from the heading, then soften only near
// the target to prevent the robot from swinging through it.
constexpr float GYROR_BACKWARD_SOFTEN_RANGE_DEG = 7.0f;
constexpr float GYROR_BACKWARD_MIN_KP_FACTOR = 0.45f;
constexpr uint32_t GYROR_FORWARD_HEADING_STARTUP_MS = 120UL;
// Reverse at speed 100 drifts before a long heading delay can react.
constexpr uint32_t GYROR_BACKWARD_HEADING_STARTUP_MS = 55UL;
// Keep this equal to set_line_distance_scale() in set_robo.ino.
float gyrorDistanceScale = 2.0f;

float wrapAngle180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float smoothstep(float value) {
  value = constrain(value, 0.0f, 1.0f);
  return value * value * (3.0f - 2.0f * value);
}

void stopWithBrake(bool forward, uint8_t speed, uint8_t brakeMs) {
  // brakeMs == 0 is the continuous-motion mode: do not issue any motor
  // command, so the following command can take over without a slow-down.
  if (brakeMs == 0u) return;

  // Apply an equal, short reverse pulse.  This stops straight without the
  // bounce caused by braking only one side, then release both motors.
  const int brakeSpeed = max(20, static_cast<int>(speed * 0.45f));
  const int brakeCommand = forward ? -brakeSpeed : brakeSpeed;
  motor(brakeCommand, brakeCommand);
  delay(brakeMs);
  motor(0, 0);
}

void stopTurnWithBrake(int leftCommand, int rightCommand, uint8_t brakeMs) {
  // A zero brake time keeps the last motor command active so a following
  // fgyro/bgyro command can continue without an unwanted stop.
  if (brakeMs == 0u) return;

  // Do not reverse the motors to stop a spin.  Reverse braking rotates the
  // robot back past the target, then the PID reverses it again, causing the
  // visible shake at the end of gyro_turn().  (1, 1) is the driver's equal
  // electrical-brake command for both wheels.
  (void)leftCommand;
  (void)rightCommand;
  motor(1, 1);
  delay(brakeMs);
  motor(0, 0);
}

void stopDriveWithBrake(bool forward, uint8_t speed, uint8_t brakeMs) {
  // brake = 0: do not slow down or stop; the next motion command takes over.
  if (brakeMs == 0u) return;

  const int brakeSpeed = max(20, static_cast<int>(speed * 0.45f));
  const int brakeCommand = forward ? -brakeSpeed : brakeSpeed;
  motor(brakeCommand, brakeCommand);
  delay(brakeMs);
  motor(0, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
constexpr float TURN_HEADING_DEADBAND_DEG = 1.5f;

// Absolute gyro turn.  Motor values may be signed, so (50, -1) is a pivot
// around the right wheel and (40, -40) is a two-wheel spin.
void gyro_turn(int speedLeft, int speedRight, float targetAngle, float kp,
               float kd, uint8_t brake) {
  if (!imuReady || !headingReferenceReady || !isfinite(targetAngle) ||
      !isfinite(kp) || !isfinite(kd) || kp < 0.0f || kd < 0.0f ||
      abs(speedLeft) > 100 || abs(speedRight) > 100 ||
      (speedLeft == 0 && speedRight == 0)) {
    motor(1, 1);
    return;
  }

  if (!imu.update() || !isfinite(imu.yawRaw())) {
    motor(1, 1);
    return;
  }

  targetAngle = wrapAngle180(targetAngle);
  const int leftLimit = abs(speedLeft);
  const int rightLimit = abs(speedRight);
  const float maxSpeed = static_cast<float>(max(leftLimit, rightLimit));
  const uint32_t startedAtUs = micros();
  float lastYaw = imu.yawRaw();
  uint32_t lastYawAtUs = startedAtUs;
  float filteredYawRate = 0.0f;
  const float initialError = wrapAngle180(targetAngle - wrapAngle180(lastYaw));

  for (;;) {
    const uint32_t nowUs = micros();
    if (static_cast<uint32_t>(nowUs - startedAtUs) >=
            GYRO_TURN_TIMEOUT_MS * 1000UL || !imu.update()) {
      motor(1, 1);
      return;
    }

    const float rawYaw = imu.yawRaw();
    if (!isfinite(rawYaw)) {
      motor(1, 1);
      return;
    }

    // yaw rate: คำนวณเฉพาะตอนที่ sensor ให้มุมใหม่จริง
    if (rawYaw != lastYaw) {
      const float yawDt = static_cast<float>(nowUs - lastYawAtUs) * 0.000001f;
      if (yawDt >= GYRO_RATE_MIN_DT_SECONDS &&
          yawDt <= GYRO_RATE_MAX_DT_SECONDS) {
        const float rate = wrapAngle180(rawYaw - lastYaw) / yawDt;
        filteredYawRate += 0.5f * (rate - filteredYawRate);
      }
      lastYaw = rawYaw;
      lastYawAtUs = nowUs;
    } else if (static_cast<uint32_t>(nowUs - lastYawAtUs) >
               static_cast<uint32_t>(GYRO_RATE_MAX_DT_SECONDS * 1000000.0f)) {
      filteredYawRate = 0.0f;  // yaw นิ่งนาน = ไม่ได้หมุนจริง
    }

    const float error = wrapAngle180(targetAngle - wrapAngle180(rawYaw));

    // A fast pivot can cross the target between two loop samples.  Stop on
    // that crossing instead of chasing back and potentially rotating away.
    if (fabs(initialError) > TURN_HEADING_DEADBAND_DEG &&
        ((initialError > 0.0f && error <= 0.0f) ||
         (initialError < 0.0f && error >= 0.0f))) {
      break;
    }
    if (fabs(error) < TURN_HEADING_DEADBAND_DEG) break;

    float turnPower = kp * error - kd * filteredYawRate;
    const bool approaching = fabs(error) < GYRO_TURN_APPROACH_DEG;
    const float turnLimit = approaching
                              ? min(maxSpeed, GYRO_TURN_APPROACH_MAX_SPEED)
                              : maxSpeed;
    const float minimumPower = min(maxSpeed,
        static_cast<float>(approaching ? GYRO_TURN_APPROACH_MIN_SPEED
                                       : GYRO_TURN_MIN_SPEED));
    turnPower = constrain(turnPower, -turnLimit, turnLimit);
    if (fabs(turnPower) < minimumPower) {
      turnPower = turnPower < 0.0f ? -minimumPower : minimumPower;
    }

    // error บวก = ล้อซ้ายไปหน้า ล้อขวาถอย (ทิศเดียวกับ gyro_turn)
    const int direction = turnPower < 0.0f ? -1 : 1;
    const float magnitude = fabs(turnPower);
    const int leftPower = leftLimit == 0 ? 0
        : max(1, static_cast<int>(magnitude * leftLimit / maxSpeed));
    const int rightPower = rightLimit == 0 ? 0
        : max(1, static_cast<int>(magnitude * rightLimit / maxSpeed));
    motor(direction * leftPower, -direction * rightPower);
    delay(4);
  }

  stopTurnWithBrake(0, 0, brake);
}

void gyrorMotorMotion(int left, int right) {
  if (left == 1) left = 2;
  else if (left == -1) left = -2;
  if (right == 1) right = 2;
  else if (right == -1) right = -2;
  motor(left, right);
}

void runGyror(bool forward, float targetAngle, uint8_t speed, float kp,
              float distanceCm, uint8_t stopPull) {
  if (!motion_heading_ready() || speed == 0u || speed > 100u ||
      !(kp >= 0.0f) || !(distanceCm > 0.0f) || stopPull > 100u ||
      !isfinite(targetAngle)) {
    motor(1, 1);
    return;
  }

  targetAngle = wrapAngle180(targetAngle);
  if (!motion_heading_update() || !isfinite(motion_heading_yaw())) {
    motor(1, 1);
    return;
  }

  const float targetDistanceMm = distanceCm * 10.0f;
  const bool useRamp = stopPull > 0u;
  const float kd = forward ? GYROR_FORWARD_KD : GYROR_BACKWARD_KD;
  const uint32_t startedAtUs = micros();
  uint32_t previousAtUs = startedAtUs;
  float estimatedDistanceMm = 0.0f;
  float lastYaw = motion_heading_yaw();
  uint32_t lastYawAtUs = startedAtUs;
  float filteredYawRate = 0.0f;

  for (;;) {
    const uint32_t nowUs = micros();
    if (static_cast<uint32_t>(nowUs - startedAtUs) >=
            GYROR_TIMEOUT_MS * 1000UL || !motion_heading_update()) {
      motor(1, 1);
      return;
    }

    const float rawYaw = motion_heading_yaw();
    if (!isfinite(rawYaw)) {
      motor(1, 1);
      return;
    }

    const float currentYaw = wrapAngle180(rawYaw);
    const float error = wrapAngle180(targetAngle - currentYaw);
    const float dt = static_cast<float>(nowUs - previousAtUs) * 0.000001f;
    previousAtUs = nowUs;

    // Same rate filter as the recovered code: update only on a fresh yaw.
    if (rawYaw != lastYaw) {
      const float yawDt = static_cast<float>(nowUs - lastYawAtUs) * 0.000001f;
      if (yawDt >= GYRO_RATE_MIN_DT_SECONDS &&
          yawDt <= GYRO_RATE_MAX_DT_SECONDS) {
        filteredYawRate += 0.5f *
            (wrapAngle180(rawYaw - lastYaw) / yawDt - filteredYawRate);
      }
      lastYaw = rawYaw;
      lastYawAtUs = nowUs;
    } else if (static_cast<uint32_t>(nowUs - lastYawAtUs) >= 60000UL) {
      filteredYawRate = 0.0f;
      lastYawAtUs = nowUs;
    }

    if (estimatedDistanceMm >= targetDistanceMm) {
      if (stopPull == 0u) return;
      const int brakeSpeed = max(12, static_cast<int>(speed * 0.30f));
      const int brakeCommand = forward ? -brakeSpeed : brakeSpeed;
      gyrorMotorMotion(brakeCommand, brakeCommand);
      delay(stopPull);
      motor(1, 1);
      return;
    }

    float rampFactor = 1.0f;
    if (useRamp) {
      const float elapsedMs = static_cast<float>(nowUs - startedAtUs) * 0.001f;
      const uint32_t accelTimeMs = forward ? GYROR_FORWARD_ACCEL_TIME_MS
                                           : GYROR_BACKWARD_ACCEL_TIME_MS;
      const float accelFactor = smoothstep(elapsedMs /
          static_cast<float>(accelTimeMs));
      const float remainingMm = max(0.0f, targetDistanceMm - estimatedDistanceMm);
      const float decelRangeMm = min(GYROR_DECEL_DISTANCE_MM,
                                     targetDistanceMm * 0.40f);
      float decelFactor = 1.0f;
      if (remainingMm < decelRangeMm) {
        const float x = remainingMm / decelRangeMm;
        decelFactor = GYROR_MIN_DECEL_FACTOR +
            (1.0f - GYROR_MIN_DECEL_FACTOR) * smoothstep(x);
      }
      rampFactor = min(accelFactor, decelFactor);
    }

    const float baseSpeed = (forward ? 1.0f : -1.0f) * speed * rampFactor;
    float effectiveKp = kp;
    if (!forward && useRamp) {
      const float nearTarget = fabs(error) / GYROR_BACKWARD_SOFTEN_RANGE_DEG;
      const float kpFactor = GYROR_BACKWARD_MIN_KP_FACTOR +
          (1.0f - GYROR_BACKWARD_MIN_KP_FACTOR) * smoothstep(nearTarget);
      effectiveKp *= kpFactor;
    }
    // Do not let a small, initial gyro offset turn one wheel before both
    // motors have gripped the floor.  Full heading control follows quickly.
    const uint32_t headingStartupMs = forward
        ? GYROR_FORWARD_HEADING_STARTUP_MS
        : GYROR_BACKWARD_HEADING_STARTUP_MS;
    // brake=0 is a continuous handoff: retain the original immediate gyro
    // control.  The launch softening is only for moves that stop afterward.
    const float startupHeadingFactor = useRamp ? smoothstep(
        static_cast<float>(nowUs - startedAtUs) * 0.001f /
        static_cast<float>(headingStartupMs)) : 1.0f;
    float correction = startupHeadingFactor * GYROR_CORRECTION_SIGN *
        (effectiveKp * error - kd * filteredYawRate);
    const float minimum = forward ? 0.0f : -100.0f;
    const float maximum = forward ? 100.0f : 0.0f;
    const int leftOutput = static_cast<int>(constrain(baseSpeed - correction,
                                                       minimum, maximum));
    const int rightOutput = static_cast<int>(constrain(baseSpeed + correction,
                                                        minimum, maximum));
    gyrorMotorMotion(leftOutput, rightOutput);

    const float travelCommand = forward
        ? (static_cast<float>(leftOutput) + rightOutput) * 0.5f
        : -(static_cast<float>(leftOutput) + rightOutput) * 0.5f;
    estimatedDistanceMm += GYROR_MM_PER_SECOND_AT_100 * gyrorDistanceScale *
        (travelCommand / 100.0f) * dt;
    // Match the recovered implementation: finish in this same control frame.
    if (estimatedDistanceMm >= targetDistanceMm) {
      if (stopPull == 0u) return;
      const int brakeSpeed = max(12, static_cast<int>(speed * 0.30f));
      const int brakeCommand = forward ? -brakeSpeed : brakeSpeed;
      gyrorMotorMotion(brakeCommand, brakeCommand);
      delay(stopPull);
      motor(1, 1);
      return;
    }
  }
}

// Recovered stable controller, kept here so a library update cannot remove it.
void fw_gyror(float targetAngle, uint8_t speed, float kp, float distanceCm,
              uint8_t brake) {
  runGyror(true, targetAngle, speed, kp, distanceCm, brake);
}

void bw_gyror(float targetAngle, uint8_t speed, float kp, float distanceCm,
              uint8_t brake) {
  runGyror(false, targetAngle, speed, kp, distanceCm, brake);
}

// Route-command names used throughout the user's sketch.
void fgyro(float targetAngle, uint8_t speed, float kp, float distanceCm,
           uint8_t brake) {
  fw_gyror(targetAngle, speed, kp, distanceCm, brake);
}

void bgyro(float targetAngle, uint8_t speed, float kp, float distanceCm,
           uint8_t brake) {
  bw_gyror(targetAngle, speed, kp, distanceCm, brake);
}

bool set_fgyro_distance_scale(float scale) {
  if (!isfinite(scale) || scale < 0.1f || scale > 5.0f) return false;
  gyrorDistanceScale = scale;
  return true;
}

