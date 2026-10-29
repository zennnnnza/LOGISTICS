// ---------- helper สำหรับ F_FLINE (ย้ายแนวคิดกลุ่มเส้นจาก runLine) ----------
// แบ่งเซนเซอร์ที่ดำติดกันเป็นกลุ่ม แล้วหาตำแหน่งกึ่งกลางถ่วงน้ำหนักของแต่ละกลุ่ม
static uint8_t FFL_findGroups(const uint16_t strength[16],
                              const int16_t position[16],
                              uint8_t groupStart[8],
                              uint8_t groupEnd[8],
                              float groupCenter[8]) {
  uint8_t count = 0;
  uint8_t i = 0;
  while (i < 16 && count < 8) {
    if (strength[i] == 0) { ++i; continue; }
    const uint8_t startIndex = i;
    uint32_t sum = 0;
    int32_t weighted = 0;
    while (i < 16 && strength[i] > 0) {
      sum += strength[i];
      weighted += static_cast<int32_t>(strength[i]) * position[i];
      ++i;
    }
    groupStart[count] = startIndex;
    groupEnd[count] = i - 1;
    groupCenter[count] = static_cast<float>(weighted) / sum;
    ++count;
  }
  return count;
}

// เลือกกลุ่มที่อยู่ใกล้ตำแหน่งอ้างอิงที่สุด (count ต้อง > 0)
static uint8_t FFL_nearestGroup(const float groupCenter[8], uint8_t count,
                                float reference, float& bestDistance) {
  uint8_t best = 0;
  bestDistance = fabsf(groupCenter[0] - reference);
  for (uint8_t i = 1; i < count; ++i) {
    const float d = fabsf(groupCenter[i] - reference);
    if (d < bestDistance) { bestDistance = d; best = i; }
  }
  return best;
}

// [ใหม่] กึ่งกลางกลุ่มแบบต่อเนื่อง: น้ำหนัก = darkness - floorDarkness และนับช่องข้างเคียงอีกข้างละ 1 ช่อง
// เซนเซอร์ที่ข้ามเส้น LINE_BLACK_MIN จึงค่อย ๆ เพิ่มน้ำหนักเข้ามา ไม่กระโดดเข้ากลุ่มทีเดียว
static float FFL_smoothCenter(const uint16_t darkness[16],
                              const int16_t position[16],
                              uint8_t startIndex, uint8_t endIndex,
                              uint16_t floorDarkness) {
  const uint8_t first = startIndex > 0 ? startIndex - 1 : 0;
  const uint8_t last = endIndex < 15 ? endIndex + 1 : 15;
  float sum = 0.0f;
  float weighted = 0.0f;
  for (uint8_t i = first; i <= last; ++i) {
    if (darkness[i] <= floorDarkness) continue;
    const float w = static_cast<float>(darkness[i] - floorDarkness);
    sum += w;
    weighted += w * static_cast<float>(position[i]);
  }
  return sum > 0.0f ? weighted / sum : 0.0f;
}

// แกนกลางของ F_FLINE และ F_FLINE_RL. avoidSide: 'N', 'R' หรือ 'L'
static void F_FLINE_Core(int sl, int sr, float kp, float distanceCm,
                         uint8_t stopPull, char avoidSide,
                         bool stopAtSensor = false,
                         LineExitSensor exitSensor = f0) {
  // ===== ค่าจูน F_FLINE: ปรับที่นี่ได้โดยไม่ต้องแก้ libraries =====
  // normalized = 0 คือดำ, 1000 คือขาว
  // --- การตามเส้น (แนวเดียวกับ runLine) ---
  const uint16_t LINE_BLACK_MIN = 500;          // darkness > ค่านี้ถึงนับว่าเป็นเส้นดำจริง
  const float TRACK_WINDOW = 30.0f;             // กลุ่มเส้นต้องอยู่ห่างจาก error เดิมไม่เกินนี้
  const float OUTSIDE_ACCEPT_JUMP = 12.0f;      // กลุ่มนอกหน้าต่างต้องนิ่งภายในนี้ต่อเนื่อง
  const uint8_t OUTSIDE_FRAMES_WIDE = 2;        // กลุ่มกว้าง >= 2 ช่อง ใช้กี่เฟรมถึงยอมรับ
  const uint8_t OUTSIDE_FRAMES_NARROW = 3;      // กลุ่มแคบ 1 ช่อง
  const float MAX_ERROR_RATE_PER_SECOND = 5000.0f;
  const float MIN_ERROR_STEP = 2.0f;
  const float MAX_ERROR_STEP = 15.0f;
  const float DT_MIN = 0.0005f;
  const float DT_MAX = 0.020f;
  const float EDGE_SIDE_MIN = 8.0f;             // |ตำแหน่งเส้น| เกินนี้ถึงจำว่าเส้นอยู่ซ้าย/ขวา
  // --- PID ---
  const float P_SCALE = 0.48f;
  const float KD = 0.0010f;
  const float MAX_CORRECTION = 80.0f;
  const float MAX_EFFECTIVE_KP = 1.0f;
  const float MAX_DISTANCE_KP = 2.00f;
  const float LINE_CORRECTION_STEP_PER_SCAN = 3.5f;
  const float FAST_RECENTER_ERROR = 10.0f;
  const float FAST_RECENTER_P_MULTIPLIER = 1.45f;
  const float FAST_RECENTER_STEP_PER_SCAN = 9.0f;
  const float DISTANCE_P_MULTIPLIER = 1.55f;
  const float DISTANCE_FAST_ERROR = 3.0f;
  const float DISTANCE_NORMAL_STEP_PER_SCAN = 8.0f;
  const float DISTANCE_FAST_STEP_PER_SCAN = 16.0f;
  const float DISTANCE_RECENTER_START_ERROR = 22.0f;
  const float DISTANCE_RECENTER_RELEASE_ERROR = 14.0f;
  const float DISTANCE_RECENTER_CORRECTION = 38.0f;
  const uint8_t DISTANCE_RECENTER_CENTER_SCANS = 2;
  // --- [ใหม่] ลดอาการสั่น ---
  const uint16_t CENTROID_FLOOR = 200;          // darkness ต่ำกว่านี้ไม่นับน้ำหนัก (ต้อง < LINE_BLACK_MIN)
  const float D_FILTER_TAU = 0.040f;            // วินาที: ยิ่งมากยิ่งนิ่ง แต่ D จะหน่วงขึ้น
  const float FAST_RECENTER_BLEND = 6.0f;       // mm: ช่วงไล่ gain "fast" รอบ fastError (เดิมสลับทันที)
  const float SPEED_SCALE_TAU = 0.030f;         // วินาที: กรอง speedScale ไม่ให้กระตุกตาม noise
  // --- เส้นหาย ---
  const uint8_t SOFT_LOST_SCANS = 2;            // โหมด L/R/เซนเซอร์: ถือคำสั่งเดิมกี่ scan
  const uint16_t SEARCH_STRAIGHT_MS = 35;
  const uint16_t MAX_LOST_MS = 2500;
  const int RECOVERY_OUTER_SPEED = 88;          // โหมด L/R/เซนเซอร์
  const int RECOVERY_INNER_REVERSE = 30;
  const int DIST_RECOVERY_OUTER_SPEED = 100;    // โหมดระยะทางปกติ (หมุนทันทีเฟรมแรกที่ขาว)
  const int DIST_RECOVERY_INNER_REVERSE = 30;
  // --- ระบบดันหลบเส้นด้านข้าง L/R (คงเดิม) ---
  const uint16_t AVOID_DARKNESS_MIN = 360;
  const uint16_t AVOID_CLEAR_DARKNESS = 260;
  const uint8_t AVOID_CLEAR_SCANS = 2;
  const float AVOID_SOFT_PUSH_MIN = 9.0f;
  const float AVOID_SOFT_PUSH_MAX = 20.0f;
  const float AVOID_HARD_PUSH = 30.0f;
  const float AVOID_SOFT_MIN_SPEED_SCALE = 0.92f;
  const float AVOID_HARD_MIN_SPEED_SCALE = 0.80f;
  const float AVOID_BIAS_STEP_PER_SCAN = 2.5f;
  const float CENTER_LOCK_AVOID_MAX = 12.0f;
  // --- จบด้วยเซนเซอร์ / ระยะทาง ---
  const uint8_t EXIT_CONFIRM_SCANS = 2;
  const uint16_t EXIT_BLACK_DARKNESS_MIN = 500;
  const float DISTANCE_SCALE = 1.95f;  // ใช้ค่าเดียวกับ set_line_distance_scale(1.95f)
  const float MM_PER_SEC_AT_100 = 900.0f;
  // =====================================================================

  const uint8_t MUX_SELECT[] = {17, 16, 15, 14};
  const uint8_t MUX_SIGNAL = 27;
  const uint16_t MUX_SETTLE_US = 40;
  const uint8_t MUX_DISCARD_READS = 1;
  const uint8_t MUX_AVERAGE_READS = 2;
  const int16_t SENSOR_POSITION[] = {
      -75, -65, -55, -45, -35, -25, -15, -5,
        5,  15,  25,  35,  45,  55,  65, 75};

  // ดึง min/max ที่คาลิเบรตไว้ครั้งเดียว แล้วอ่าน MUX เองแบบรวดเร็ว
  int32_t minRaw[16];
  int32_t maxRaw[16];
  for (uint8_t channel = 0; channel < 16; ++channel) {
    LineSensorReading reading;
    if (!read_line_sensor(static_cast<LineExitSensor>(f0 + channel), reading) ||
        !reading.calibrationValid || reading.maxRaw <= reading.minRaw) {
      Serial.print(F("f_line_color_pass stopped: line sensor F"));
      Serial.print(channel);
      Serial.println(F(" is not calibrated"));
      motor(-1, -1);
      return;
    }
    minRaw[channel] = reading.minRaw;
    maxRaw[channel] = reading.maxRaw;
  }

  sl = constrain(sl, 2, 100);
  sr = constrain(sr, 2, 100);
  kp = constrain(kp, 0.0f,
                 stopAtSensor ? MAX_EFFECTIVE_KP : MAX_DISTANCE_KP);
  if (avoidSide == 'r') avoidSide = 'R';
  if (avoidSide == 'l') avoidSide = 'L';
  const bool avoidRight = avoidSide == 'R';
  const bool avoidLeft = avoidSide == 'L';
  const bool distanceOnlyMode = !stopAtSensor && !avoidRight && !avoidLeft;
  if (!stopAtSensor && distanceCm <= 0.0f) {
    motor(-1, -1);
    return;
  }
  if (stopAtSensor && !((exitSensor >= f0 && exitSensor <= f15) ||
                        exitSensor == cl || exitSensor == cr)) {
    motor(-1, -1);
    return;
  }

  // โหมดระยะทางปกติหมุนกลับทันทีที่แถวขาว (เหมือน runLine); kp ต่ำมากไม่เปิดการหมุน
  const bool recoveryEnabled = !distanceOnlyMode || kp > 0.5f;
  const uint8_t softLostScans = distanceOnlyMode ? 0 : SOFT_LOST_SCANS;

  const float targetMm = distanceCm * 10.0f;
  uint32_t previousUs = micros();
  float travelledMm = 0.0f;
  float previousError = 0.0f;
  float filteredDerivative = 0.0f;
  float lastTrackedError = 0.0f;    // ตำแหน่งเส้นที่ผ่าน rate limit แล้ว (ใช้เป็น error)
  float lastBlackPos = 0.0f;        // ตำแหน่งเส้นล่าสุดก่อนขาวทั้งแถว
  bool trackingInitialized = false;
  bool edgeRecoveryActive = false;
  float previousRecoveryCandidate = 0.0f;
  uint8_t recoveryFrameCount = 0;
  int lastLeftCommand = sl;
  int lastRightCommand = sr;
  uint8_t lostScans = 0;
  uint32_t lineLostSinceMs = 0;
  bool hardAvoidActive = false;
  uint8_t avoidClearScans = 0;
  float filteredAvoidCorrection = 0.0f;
  float filteredLineCorrection = 0.0f;
  uint8_t exitBlackScans = 0;
  int8_t lastEdgeSide = 0;  // -1 = เส้นหายทางซ้าย, +1 = เส้นหายทางขวา
  bool distanceRecenterActive = false;
  int8_t distanceRecenterDirection = 0;
  uint8_t distanceRecenterCenterScans = 0;
  float filteredSpeedScale = 1.0f;  // [ใหม่] speedScale ที่ผ่านการกรองแล้ว

  while (stopAtSensor || travelledMm < targetMm) {
    const uint32_t nowUs = micros();
    float realDt = (nowUs - previousUs) * 0.000001f;
    if (realDt <= 0.0f) realDt = 0.001f;
    previousUs = nowUs;
    const float dt = constrain(realDt, DT_MIN, DT_MAX);

    // ---------- อ่านเซนเซอร์ ----------
    uint16_t sensorDarkness[16] = {};
    uint16_t blackStrength[16] = {};
    bool centerLocked = false;
    for (uint8_t channel = 0; channel < 16; ++channel) {
      for (uint8_t bit = 0; bit < 4; ++bit) {
        digitalWrite(MUX_SELECT[bit], (channel >> bit) & 0x01U);
      }
      delayMicroseconds(MUX_SETTLE_US);
      for (uint8_t discard = 0; discard < MUX_DISCARD_READS; ++discard) {
        (void)analogRead(MUX_SIGNAL);
      }
      uint32_t sum = 0;
      for (uint8_t sample = 0; sample < MUX_AVERAGE_READS; ++sample) {
        sum += static_cast<uint16_t>(analogRead(MUX_SIGNAL));
      }
      const int32_t raw = sum / MUX_AVERAGE_READS;
      const int32_t normalized = constrain(
          (raw - minRaw[channel]) * 1000L / (maxRaw[channel] - minRaw[channel]),
          0L, 1000L);
      const uint16_t darkness = 1000U - normalized;
      sensorDarkness[channel] = darkness;
      if (darkness > LINE_BLACK_MIN) {
        blackStrength[channel] = darkness;
        if (channel >= 6 && channel <= 10) centerLocked = true;
      }
    }

    // ---------- จบด้วยเซนเซอร์ (คงเดิม) ----------
    if (stopAtSensor) {
      bool exitIsBlack = false;
      if (exitSensor >= f0 && exitSensor <= f15) {
        const uint8_t exitChannel = static_cast<uint8_t>(exitSensor - f0);
        exitIsBlack = sensorDarkness[exitChannel] > EXIT_BLACK_DARKNESS_MIN;
        if (exitChannel == 0) {
          exitIsBlack = exitIsBlack &&
                        sensorDarkness[1] > EXIT_BLACK_DARKNESS_MIN;
        } else if (exitChannel == 15) {
          exitIsBlack = exitIsBlack &&
                        sensorDarkness[14] > EXIT_BLACK_DARKNESS_MIN;
        }
      } else {
        LineSensorReading centerReading;
        exitIsBlack = read_line_sensor(exitSensor, centerReading) &&
                      centerReading.normalizedValid &&
                      centerReading.normalized < 500u;
      }
      if (exitIsBlack) {
        if (exitBlackScans < 255) ++exitBlackScans;
        if (exitBlackScans >= EXIT_CONFIRM_SCANS) break;
      } else {
        exitBlackScans = 0;
      }
    }

    // ---------- เลือกกลุ่มเส้นที่ต้องตาม (แบบ runLine) ----------
    uint8_t groupStart[8];
    uint8_t groupEnd[8];
    float groupCenter[8];
    const uint8_t groupCount = FFL_findGroups(blackStrength, SENSOR_POSITION,
                                              groupStart, groupEnd, groupCenter);
    // [แก้] ใช้กึ่งกลางแบบต่อเนื่อง ไม่กระโดดตอนเซนเซอร์ข้ามเส้น LINE_BLACK_MIN
    for (uint8_t g = 0; g < groupCount; ++g) {
      groupCenter[g] = FFL_smoothCenter(sensorDarkness, SENSOR_POSITION,
                                        groupStart[g], groupEnd[g],
                                        CENTROID_FLOOR);
    }
    const bool anyBlack = groupCount > 0;

    if (anyBlack) {
      lostScans = 0;
      lineLostSinceMs = 0;
    } else {
      if (lostScans < 255) ++lostScans;
      if (lineLostSinceMs == 0) lineLostSinceMs = millis();
    }

    bool lineFound = false;
    bool resumeNoSlew = false;   // เฟรมที่ต้องกระโดดไปตำแหน่งใหม่ทันที ไม่ rate limit ไม่มี D kick
    float measuredError = lastTrackedError;
    if (anyBlack) {
      float dist = 0.0f;
      if (edgeRecoveryActive) {
        // เจอดำครั้งแรกหลังหมุนหา: เลือกกลุ่มที่ใกล้ฝั่งที่จำไว้ แล้ว rejoin ทันที
        const uint8_t s = FFL_nearestGroup(groupCenter, groupCount, lastBlackPos, dist);
        measuredError = groupCenter[s];
        lineFound = true;
        resumeNoSlew = true;
      } else if (!trackingInitialized) {
        const uint8_t s = FFL_nearestGroup(groupCenter, groupCount, 0.0f, dist);
        measuredError = groupCenter[s];
        lineFound = true;
        resumeNoSlew = true;
      } else {
        const uint8_t s = FFL_nearestGroup(groupCenter, groupCount, lastTrackedError, dist);
        if (dist <= TRACK_WINDOW) {
          measuredError = groupCenter[s];
          lineFound = true;
          recoveryFrameCount = 0;
        } else {
          // กลุ่มนอกหน้าต่าง (กิ่ง/โค้งหักเร็ว): ต้องนิ่งหลายเฟรมก่อนยอมรับ
          const float candidate = groupCenter[s];
          const uint8_t groupSize = groupEnd[s] - groupStart[s] + 1;
          recoveryFrameCount =
              (recoveryFrameCount > 0 &&
               fabsf(candidate - previousRecoveryCandidate) <= OUTSIDE_ACCEPT_JUMP)
                  ? recoveryFrameCount + 1 : 1;
          previousRecoveryCandidate = candidate;
          if (recoveryFrameCount >= (groupSize >= 2 ? OUTSIDE_FRAMES_WIDE
                                                     : OUTSIDE_FRAMES_NARROW)) {
            measuredError = candidate;
            lineFound = true;
            resumeNoSlew = true;
          }
        }
      }
    } else {
      recoveryFrameCount = 0;
    }

    if (lineFound) {
      if (resumeNoSlew) {
        lastTrackedError = constrain(measuredError, -75.0f, 75.0f);
        trackingInitialized = true;
        edgeRecoveryActive = false;
        recoveryFrameCount = 0;
      } else {
        // จำกัดความเร็วที่ error เปลี่ยนต่อ scan (กันกระโดดตามเส้นกิ่ง/noise)
        float maxErrorChange = MAX_ERROR_RATE_PER_SECOND * dt;
        maxErrorChange = constrain(maxErrorChange, MIN_ERROR_STEP, MAX_ERROR_STEP);
        const float errorChange = constrain(measuredError - lastTrackedError,
                                            -maxErrorChange, maxErrorChange);
        lastTrackedError = constrain(lastTrackedError + errorChange,
                                     -75.0f, 75.0f);
      }
      // จำฝั่งที่เส้นอยู่ล่าสุด ใช้เลือกทิศหมุนเมื่อทั้งแถวขาว
      lastBlackPos = measuredError;
      lastEdgeSide = measuredError < -EDGE_SIDE_MIN ? -1
                   : (measuredError > EDGE_SIDE_MIN ? 1 : 0);
    }
    const float error = lastTrackedError;

    // ---------- ระบบดันหลบเส้นด้านข้าง L/R (คงเดิมทั้งหมด) ----------
    float avoidCorrection = 0.0f;
    bool moderateAvoid = false;
    if (avoidRight || avoidLeft) {
      const uint8_t moderateNear = avoidRight ? 10 : 5;
      const uint8_t moderateFar = avoidRight ? 11 : 4;
      const uint8_t hardNear = avoidRight ? 12 : 3;
      const uint8_t hardFar = avoidRight ? 13 : 2;
      const uint16_t moderateDarkness = max(sensorDarkness[moderateNear],
                                             sensorDarkness[moderateFar]);
      const bool hardNow = min(sensorDarkness[hardNear],
                               sensorDarkness[hardFar]) > AVOID_DARKNESS_MIN;

      if (hardNow) {
        hardAvoidActive = true;
        avoidClearScans = 0;
      } else if (hardAvoidActive) {
        const uint16_t guardDarkness = max(sensorDarkness[hardNear],
                                            sensorDarkness[hardFar]);
        if (guardDarkness < AVOID_CLEAR_DARKNESS) {
          if (avoidClearScans < 255) ++avoidClearScans;
          if (avoidClearScans >= AVOID_CLEAR_SCANS) {
            hardAvoidActive = false;
          }
        } else {
          avoidClearScans = 0;
        }
      }

      if (hardAvoidActive) {
        avoidCorrection = avoidRight ? AVOID_HARD_PUSH : -AVOID_HARD_PUSH;
      } else if (moderateDarkness > AVOID_DARKNESS_MIN) {
        moderateAvoid = true;
        const float depth = constrain(
            static_cast<float>(moderateDarkness - AVOID_DARKNESS_MIN) /
                (1000.0f - AVOID_DARKNESS_MIN),
            0.0f, 1.0f);
        float pushOut = AVOID_SOFT_PUSH_MIN +
                              (AVOID_SOFT_PUSH_MAX - AVOID_SOFT_PUSH_MIN) * depth;
        if (centerLocked) pushOut = min(pushOut, CENTER_LOCK_AVOID_MAX);
        avoidCorrection = avoidRight ? pushOut : -pushOut;
      }
    }
    filteredAvoidCorrection += constrain(avoidCorrection - filteredAvoidCorrection,
                                         -AVOID_BIAS_STEP_PER_SCAN,
                                         AVOID_BIAS_STEP_PER_SCAN);

    // ---------- D term (รีเซตตอนกระโดด/ถือ error ไม่ให้ kick) ----------
    if (resumeNoSlew || !lineFound) {
      previousError = error;
      filteredDerivative = 0.0f;
    } else {
      const float derivative = constrain((error - previousError) / dt, -900.0f, 900.0f);
      // [แก้] low-pass แบบอิงเวลา: scan เร็วหรือช้า ระดับการกรองก็เท่าเดิม (เดิม alpha 0.35 ต่อ scan)
      filteredDerivative += (derivative - filteredDerivative) *
                            (dt / (D_FILTER_TAU + dt));
      previousError = error;
    }

    // ---------- recenter โหมดระยะทางปกติ (คงเดิม) ----------
    const uint16_t tightCenter = max(sensorDarkness[7],
                                     max(sensorDarkness[8], sensorDarkness[9]));
    const bool tightCenterLocked = tightCenter > LINE_BLACK_MIN;
    if (distanceOnlyMode && anyBlack) {
      if (!distanceRecenterActive &&
          fabsf(error) >= DISTANCE_RECENTER_START_ERROR) {
        distanceRecenterActive = true;
        distanceRecenterDirection = error < 0.0f ? -1 : 1;
        distanceRecenterCenterScans = 0;
      }
      if (distanceRecenterActive) {
        if (tightCenterLocked &&
            fabsf(error) <= DISTANCE_RECENTER_RELEASE_ERROR) {
          if (distanceRecenterCenterScans < 255) {
            ++distanceRecenterCenterScans;
          }
          if (distanceRecenterCenterScans >=
              DISTANCE_RECENTER_CENTER_SCANS) {
            distanceRecenterActive = false;
            filteredLineCorrection = 0.0f;
            filteredDerivative = 0.0f;
          }
        } else {
          distanceRecenterCenterScans = 0;
        }
      }
    } else if (!anyBlack) {
      distanceRecenterActive = false;
      distanceRecenterCenterScans = 0;
    }

    // ---------- สั่งมอเตอร์ ----------
    int leftCommand = 0;
    int rightCommand = 0;
    const bool allWhiteLost = !anyBlack && lostScans > softLostScans;
    if (allWhiteLost) {
      const uint32_t lostMs = millis() - lineLostSinceMs;
      if (lostMs > MAX_LOST_MS) {
        motor(-1, -1);
        return;
      }
    }

    if (allWhiteLost && recoveryEnabled) {
      edgeRecoveryActive = true;   // เฟรมแรกที่เจอดำหลังจากนี้ = rejoin
      const uint32_t lostMs = millis() - lineLostSinceMs;
      const uint16_t searchStraightMs = distanceOnlyMode ? 0 : SEARCH_STRAIGHT_MS;
      // ฝั่งที่เส้นหาย: ใช้ฝั่งที่จำไว้ ถ้าไม่มีใช้เครื่องหมายของ error ล่าสุด
      const int8_t recoverySide = lastEdgeSide != 0
          ? lastEdgeSide : (lastTrackedError < 0.0f ? -1 : 1);
      const int outer = distanceOnlyMode ? DIST_RECOVERY_OUTER_SPEED
                                         : RECOVERY_OUTER_SPEED;
      const int inner = distanceOnlyMode ? DIST_RECOVERY_INNER_REVERSE
                                         : RECOVERY_INNER_REVERSE;
      if (lostMs < searchStraightMs) {
        leftCommand = lastLeftCommand;
        rightCommand = lastRightCommand;
      } else if (recoverySide < 0) {
        // เส้นหายทางซ้าย: ล้อซ้ายถอย/ล้อขวาเดินหน้า
        leftCommand = -inner;
        rightCommand = outer;
      } else {
        // เส้นหายทางขวา: ล้อซ้ายเดินหน้า/ล้อขวาถอย
        leftCommand = outer;
        rightCommand = -inner;
      }
    } else if (!anyBlack && lostScans <= softLostScans) {
      // หลุดเพียงไม่กี่ scan: ใช้คำสั่งเดิม ข้ามรอยขาด/เงา
      leftCommand = lastLeftCommand;
      rightCommand = lastRightCommand;
    } else {
      // ตามเส้นด้วย PID (รวมกรณีเห็นดำนอกหน้าต่างที่ยังไม่ยอมรับ = ถือ error เดิม)
      const float errorRatio = constrain(fabsf(error) / 75.0f, 0.0f, 1.0f);
      const float rateRatio = constrain(fabsf(filteredDerivative) / 900.0f,
                                        0.0f, 1.0f);
      // [แก้] กรอง speedScale: เดิมกระโดดตาม noise ของ derivative ทำให้ความเร็วรวมสั่น
      const float targetSpeedScale = constrain(1.0f - 0.20f * errorRatio -
                                               0.08f * rateRatio, 0.75f, 1.0f);
      filteredSpeedScale += (targetSpeedScale - filteredSpeedScale) *
                            (dt / (SPEED_SCALE_TAU + dt));
      float protectedSpeedScale = filteredSpeedScale;
      if (hardAvoidActive) {
        protectedSpeedScale = min(protectedSpeedScale,
                                  AVOID_HARD_MIN_SPEED_SCALE);
      } else if (moderateAvoid) {
        protectedSpeedScale = min(protectedSpeedScale,
                                  AVOID_SOFT_MIN_SPEED_SCALE);
      }
      const float fastError = stopAtSensor ? FAST_RECENTER_ERROR
                                           : DISTANCE_FAST_ERROR;
      // [แก้] เดิมสลับ gain/step ทันทีที่ fastError พอ error แกว่งรอบค่านี้ gain จะสลับไปมา
      // ตอนนี้ไล่ต่อเนื่องในช่วง fastError ± FAST_RECENTER_BLEND/2
      const float fastBlend = constrain(
          (fabsf(error) - (fastError - 0.5f * FAST_RECENTER_BLEND)) /
              FAST_RECENTER_BLEND,
          0.0f, 1.0f);
      const float modePScale = stopAtSensor ? 1.0f : DISTANCE_P_MULTIPLIER;
      const float activePScale = P_SCALE * modePScale *
          (1.0f + (FAST_RECENTER_P_MULTIPLIER - 1.0f) * fastBlend);
      const float requestedLineCorrection = constrain(
          kp * error * activePScale + KD * filteredDerivative,
          -MAX_CORRECTION, MAX_CORRECTION);
      const float normalStep = stopAtSensor ? LINE_CORRECTION_STEP_PER_SCAN
                                            : DISTANCE_NORMAL_STEP_PER_SCAN;
      const float fastStep = stopAtSensor ? FAST_RECENTER_STEP_PER_SCAN
                                          : DISTANCE_FAST_STEP_PER_SCAN;
      const float correctionStep = normalStep +
                                   (fastStep - normalStep) * fastBlend;
      if (resumeNoSlew) {
        // เจอเส้นกลับมา: ใช้ PID เต็มทันที ไม่ต้องไต่ขึ้นทีละ step
        filteredLineCorrection = requestedLineCorrection;
      } else {
        filteredLineCorrection += constrain(
            requestedLineCorrection - filteredLineCorrection,
            -correctionStep, correctionStep);
      }
      const float correction = hardAvoidActive ? filteredAvoidCorrection
          : distanceRecenterActive
              ? static_cast<float>(distanceRecenterDirection) *
                    DISTANCE_RECENTER_CORRECTION
              : constrain(filteredLineCorrection + filteredAvoidCorrection,
                          -MAX_CORRECTION, MAX_CORRECTION);
      leftCommand = constrain(lroundf(sl * protectedSpeedScale + correction),
                              -100, 100);
      rightCommand = constrain(lroundf(sr * protectedSpeedScale - correction),
                               -100, 100);
    }
    motor(leftCommand, rightCommand);
    lastLeftCommand = leftCommand;
    lastRightCommand = rightCommand;

    const float forwardCommand = max(0.0f,
        (static_cast<float>(leftCommand) + rightCommand) * 0.5f);
    travelledMm += MM_PER_SEC_AT_100 * DISTANCE_SCALE *
                    (forwardCommand / 100.0f) * realDt;

  }

  if (stopPull > 0) {
    motor(-sl, -sr);
    delay(stopPull);
    motor(-1, -1);
  } else if (stopAtSensor) {
    motor(-1, -1);
  }
  // stopPull = 0 หมายถึงต่อคำสั่ง F_FLINE_RL/F_FLINE ถัดไปแบบไม่เบรก
}
// // เวอร์ชันเดิม: ตามเส้นปกติ ใช้ได้เหมือนเดิมทุกบรรทัด
// void F_FLINE(int sl, int sr, float kp, float distanceCm, uint8_t stopPull) {
//   F_FLINE_Core(sl, sr, kp, distanceCm, stopPull, 'N');
// }

// // จบเมื่อ f0..f15 หรือ cl/cr แตะดำ; f0/f15 ต้องติดช่องข้างในด้วย
// void F_FLINE(int sl, int sr, float kp, LineExitSensor exitSensor,
//              uint8_t stopPull) {
//   F_FLINE_Core(sl, sr, kp, 0.0f, stopPull, 'N', true, exitSensor);
// }

// รูปแบบย่อของ F_FLINE_RL: เลือกแรงดันด้าน 'L'/'R' ได้โดยใช้ชื่อ F_FLINE
void f_line(int sl, int sr, float kp, float distanceCm,
             uint8_t stopPull, char side) {
  F_FLINE_Core(sl, sr, kp, distanceCm, stopPull, side);
}

void f_line(int sl, int sr, float kp, LineExitSensor exitSensor,
             uint8_t stopPull, char side) {
  F_FLINE_Core(sl, sr, kp, 0.0f, stopPull, side, true, exitSensor);
}




















// #include <MyMINI_Pro.h>
// #include <MyMINI_ProInternal.h>
// #include <MotorDriver.h>
// #include <MotionHeading.h>

// #include <math.h>


// constexpr float GYRO_RATE_MIN_DT_SECONDS = 0.004f;//--------------------------------
// constexpr float GYRO_RATE_MAX_DT_SECONDS = 0.060f;
// constexpr float GYRO_TURN_TOLERANCE_DEG = 2.0f;
// constexpr float GYRO_TURN_APPROACH_DEG = 12.0f;//----------------------------------
// constexpr uint8_t GYRO_TURN_MIN_SPEED = 12u;
// constexpr uint8_t GYRO_TURN_APPROACH_MIN_SPEED = 6u;
// constexpr float GYRO_TURN_APPROACH_MAX_SPEED = 18.0f;
// constexpr uint32_t GYRO_TURN_TIMEOUT_MS = 8000UL;//---------------------------

// // ---------------------------------------------------------------------------

// // Exact tuning used by the recovered fw_gyror/bw_gyror code.
// constexpr uint32_t GYROR_TIMEOUT_MS = 15000UL;
// constexpr uint32_t GYROR_ACCEL_TIME_MS = 400UL;
// constexpr float GYROR_MM_PER_SECOND_AT_100 = 900.0f;
// constexpr float GYROR_DECEL_DISTANCE_MM = 120.0f;
// constexpr float GYROR_MIN_DECEL_FACTOR = 0.25f;
// constexpr float GYROR_CORRECTION_SIGN = -1.0f;
// constexpr float GYROR_FORWARD_KD = 0.00850f * 1.5f;
// // Extra reverse damping: slows the yaw before it swings through target.
// constexpr float GYROR_BACKWARD_KD = 0.0125f * 2.6f;
// // Reverse at full strength when far from the heading, then soften only near
// // the target to prevent the robot from swinging through it.
// constexpr float GYROR_BACKWARD_SOFTEN_RANGE_DEG = 7.0f;
// constexpr float GYROR_BACKWARD_MIN_KP_FACTOR = 0.45f;
// // Keep this equal to set_line_distance_scale() in set_robo.ino.
// float gyrorDistanceScale = 2.0f;

// float wrapAngle180(float angle) {
//   while (angle > 180.0f) angle -= 360.0f;
//   while (angle < -180.0f) angle += 360.0f;
//   return angle;
// }

// float smoothstep(float value) {
//   value = constrain(value, 0.0f, 1.0f);
//   return value * value * (3.0f - 2.0f * value);
// }

// void stopWithBrake(bool forward, uint8_t speed, uint8_t brakeMs) {
//   // brakeMs == 0 is the continuous-motion mode: do not issue any motor
//   // command, so the following command can take over without a slow-down.
//   if (brakeMs == 0u) return;

//   // Apply an equal, short reverse pulse.  This stops straight without the
//   // bounce caused by braking only one side, then release both motors.
//   const int brakeSpeed = max(20, static_cast<int>(speed * 0.45f));
//   const int brakeCommand = forward ? -brakeSpeed : brakeSpeed;
//   motor(brakeCommand, brakeCommand);
//   delay(brakeMs);
//   motor(0, 0);
// }

// void stopTurnWithBrake(int leftCommand, int rightCommand, uint8_t brakeMs) {
//   // A zero brake time keeps the last motor command active so a following
//   // fgyro/bgyro command can continue without an unwanted stop.
//   if (brakeMs == 0u) return;

//   // Do not reverse the motors to stop a spin.  Reverse braking rotates the
//   // robot back past the target, then the PID reverses it again, causing the
//   // visible shake at the end of gyro_turn().  (1, 1) is the driver's equal
//   // electrical-brake command for both wheels.
//   (void)leftCommand;
//   (void)rightCommand;
//   motor(1, 1);
//   delay(brakeMs);
//   motor(0, 0);
// }

// void stopDriveWithBrake(bool forward, uint8_t speed, uint8_t brakeMs) {
//   // brake = 0: do not slow down or stop; the next motion command takes over.
//   if (brakeMs == 0u) return;

//   const int brakeSpeed = max(20, static_cast<int>(speed * 0.45f));
//   const int brakeCommand = forward ? -brakeSpeed : brakeSpeed;
//   motor(brakeCommand, brakeCommand);
//   delay(brakeMs);
//   motor(0, 0);
// }

// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// constexpr float TURN_HEADING_DEADBAND_DEG = 1.5f;

// // Absolute gyro turn.  Motor values may be signed, so (50, -1) is a pivot
// // around the right wheel and (40, -40) is a two-wheel spin.
// void gyro_turn(int speedLeft, int speedRight, float targetAngle, float kp,
//                float kd, uint8_t brake) {
//   if (!imuReady || !headingReferenceReady || !isfinite(targetAngle) ||
//       !isfinite(kp) || !isfinite(kd) || kp < 0.0f || kd < 0.0f ||
//       abs(speedLeft) > 100 || abs(speedRight) > 100 ||
//       (speedLeft == 0 && speedRight == 0)) {
//     motor(1, 1);
//     return;
//   }

//   if (!imu.update() || !isfinite(imu.yawRaw())) {
//     motor(1, 1);
//     return;
//   }

//   targetAngle = wrapAngle180(targetAngle);
//   const int leftLimit = abs(speedLeft);
//   const int rightLimit = abs(speedRight);
//   const float maxSpeed = static_cast<float>(max(leftLimit, rightLimit));
//   const uint32_t startedAtUs = micros();
//   float lastYaw = imu.yawRaw();
//   uint32_t lastYawAtUs = startedAtUs;
//   float filteredYawRate = 0.0f;

//   for (;;) {
//     const uint32_t nowUs = micros();
//     if (static_cast<uint32_t>(nowUs - startedAtUs) >=
//             GYRO_TURN_TIMEOUT_MS * 1000UL || !imu.update()) {
//       motor(1, 1);
//       return;
//     }

//     const float rawYaw = imu.yawRaw();
//     if (!isfinite(rawYaw)) {
//       motor(1, 1);
//       return;
//     }

//     // yaw rate: คำนวณเฉพาะตอนที่ sensor ให้มุมใหม่จริง
//     if (rawYaw != lastYaw) {
//       const float yawDt = static_cast<float>(nowUs - lastYawAtUs) * 0.000001f;
//       if (yawDt >= GYRO_RATE_MIN_DT_SECONDS &&
//           yawDt <= GYRO_RATE_MAX_DT_SECONDS) {
//         const float rate = wrapAngle180(rawYaw - lastYaw) / yawDt;
//         filteredYawRate += 0.5f * (rate - filteredYawRate);
//       }
//       lastYaw = rawYaw;
//       lastYawAtUs = nowUs;
//     } else if (static_cast<uint32_t>(nowUs - lastYawAtUs) >
//                static_cast<uint32_t>(GYRO_RATE_MAX_DT_SECONDS * 1000000.0f)) {
//       filteredYawRate = 0.0f;  // yaw นิ่งนาน = ไม่ได้หมุนจริง
//     }

//     const float error = wrapAngle180(targetAngle - wrapAngle180(rawYaw));

//     // ถึงมุมแล้ว จบทันที ไม่รอนิ่ง
//     if (fabs(error) < TURN_HEADING_DEADBAND_DEG) break;

//     float turnPower = kp * error - kd * filteredYawRate;
//     const bool approaching = fabs(error) < GYRO_TURN_APPROACH_DEG;
//     const float turnLimit = approaching
//                               ? min(maxSpeed, GYRO_TURN_APPROACH_MAX_SPEED)
//                               : maxSpeed;
//     const float minimumPower = min(maxSpeed,
//         static_cast<float>(approaching ? GYRO_TURN_APPROACH_MIN_SPEED
//                                        : GYRO_TURN_MIN_SPEED));
//     turnPower = constrain(turnPower, -turnLimit, turnLimit);
//     if (fabs(turnPower) < minimumPower) {
//       turnPower = turnPower < 0.0f ? -minimumPower : minimumPower;
//     }

//     // error บวก = ล้อซ้ายไปหน้า ล้อขวาถอย (ทิศเดียวกับ gyro_turn)
//     const int direction = turnPower < 0.0f ? -1 : 1;
//     const float magnitude = fabs(turnPower);
//     const int leftPower = leftLimit == 0 ? 0
//         : max(1, static_cast<int>(magnitude * leftLimit / maxSpeed));
//     const int rightPower = rightLimit == 0 ? 0
//         : max(1, static_cast<int>(magnitude * rightLimit / maxSpeed));
//     motor(direction * leftPower, -direction * rightPower);
//     delay(4);
//   }

//   stopTurnWithBrake(0, 0, brake);
// }

// void gyrorMotorMotion(int left, int right) {
//   if (left == 1) left = 2;
//   else if (left == -1) left = -2;
//   if (right == 1) right = 2;
//   else if (right == -1) right = -2;
//   motor(left, right);
// }

// void runGyror(bool forward, float targetAngle, uint8_t speed, float kp,
//               float distanceCm, uint8_t stopPull) {
//   if (!motion_heading_ready() || speed == 0u || speed > 100u ||
//       !(kp >= 0.0f) || !(distanceCm > 0.0f) || stopPull > 100u ||
//       !isfinite(targetAngle)) {
//     motor(1, 1);
//     return;
//   }

//   targetAngle = wrapAngle180(targetAngle);
//   if (!motion_heading_update() || !isfinite(motion_heading_yaw())) {
//     motor(1, 1);
//     return;
//   }

//   const float targetDistanceMm = distanceCm * 10.0f;
//   const bool useRamp = stopPull > 0u;
//   const float kd = forward ? GYROR_FORWARD_KD : GYROR_BACKWARD_KD;
//   const uint32_t startedAtUs = micros();
//   uint32_t previousAtUs = startedAtUs;
//   float estimatedDistanceMm = 0.0f;
//   float lastYaw = motion_heading_yaw();
//   uint32_t lastYawAtUs = startedAtUs;
//   float filteredYawRate = 0.0f;

//   for (;;) {
//     const uint32_t nowUs = micros();
//     if (static_cast<uint32_t>(nowUs - startedAtUs) >=
//             GYROR_TIMEOUT_MS * 1000UL || !motion_heading_update()) {
//       motor(1, 1);
//       return;
//     }

//     const float rawYaw = motion_heading_yaw();
//     if (!isfinite(rawYaw)) {
//       motor(1, 1);
//       return;
//     }

//     const float currentYaw = wrapAngle180(rawYaw);
//     const float error = wrapAngle180(targetAngle - currentYaw);
//     const float dt = static_cast<float>(nowUs - previousAtUs) * 0.000001f;
//     previousAtUs = nowUs;

//     // Same rate filter as the recovered code: update only on a fresh yaw.
//     if (rawYaw != lastYaw) {
//       const float yawDt = static_cast<float>(nowUs - lastYawAtUs) * 0.000001f;
//       if (yawDt >= GYRO_RATE_MIN_DT_SECONDS &&
//           yawDt <= GYRO_RATE_MAX_DT_SECONDS) {
//         filteredYawRate += 0.5f *
//             (wrapAngle180(rawYaw - lastYaw) / yawDt - filteredYawRate);
//       }
//       lastYaw = rawYaw;
//       lastYawAtUs = nowUs;
//     } else if (static_cast<uint32_t>(nowUs - lastYawAtUs) >= 60000UL) {
//       filteredYawRate = 0.0f;
//       lastYawAtUs = nowUs;
//     }

//     if (estimatedDistanceMm >= targetDistanceMm) {
//       if (stopPull == 0u) return;
//       const int brakeCommand = forward ? -static_cast<int>(speed)
//                                        : static_cast<int>(speed);
//       gyrorMotorMotion(brakeCommand, brakeCommand);
//       delay(stopPull);
//       motor(1, 1);
//       return;
//     }

//     float rampFactor = 1.0f;
//     if (useRamp) {
//       const float elapsedMs = static_cast<float>(nowUs - startedAtUs) * 0.001f;
//       const float accelFactor = smoothstep(elapsedMs /
//           static_cast<float>(GYROR_ACCEL_TIME_MS));
//       const float remainingMm = max(0.0f, targetDistanceMm - estimatedDistanceMm);
//       const float decelRangeMm = min(GYROR_DECEL_DISTANCE_MM,
//                                      targetDistanceMm * 0.40f);
//       float decelFactor = 1.0f;
//       if (remainingMm < decelRangeMm) {
//         const float x = remainingMm / decelRangeMm;
//         decelFactor = GYROR_MIN_DECEL_FACTOR +
//             (1.0f - GYROR_MIN_DECEL_FACTOR) * smoothstep(x);
//       }
//       rampFactor = min(accelFactor, decelFactor);
//     }

//     const float baseSpeed = (forward ? 1.0f : -1.0f) * speed * rampFactor;
//     float effectiveKp = kp;
//     if (!forward) {
//       const float nearTarget = fabs(error) / GYROR_BACKWARD_SOFTEN_RANGE_DEG;
//       const float kpFactor = GYROR_BACKWARD_MIN_KP_FACTOR +
//           (1.0f - GYROR_BACKWARD_MIN_KP_FACTOR) * smoothstep(nearTarget);
//       effectiveKp *= kpFactor;
//     }
//     const float correction = GYROR_CORRECTION_SIGN *
//         (effectiveKp * error - kd * filteredYawRate);
//     const float minimum = forward ? 0.0f : -100.0f;
//     const float maximum = forward ? 100.0f : 0.0f;
//     const int leftOutput = static_cast<int>(constrain(baseSpeed - correction,
//                                                        minimum, maximum));
//     const int rightOutput = static_cast<int>(constrain(baseSpeed + correction,
//                                                         minimum, maximum));
//     gyrorMotorMotion(leftOutput, rightOutput);

//     const float travelCommand = forward
//         ? (static_cast<float>(leftOutput) + rightOutput) * 0.5f
//         : -(static_cast<float>(leftOutput) + rightOutput) * 0.5f;
//     estimatedDistanceMm += GYROR_MM_PER_SECOND_AT_100 * gyrorDistanceScale *
//         (travelCommand / 100.0f) * dt;
//     // Match the recovered implementation: finish in this same control frame.
//     if (estimatedDistanceMm >= targetDistanceMm) {
//       if (stopPull == 0u) return;
//       const int brakeCommand = forward ? -static_cast<int>(speed)
//                                        : static_cast<int>(speed);
//       gyrorMotorMotion(brakeCommand, brakeCommand);
//       delay(stopPull);
//       motor(1, 1);
//       return;
//     }
//   }
// }

// // Recovered stable controller, kept here so a library update cannot remove it.
// void fw_gyror(float targetAngle, uint8_t speed, float kp, float distanceCm,
//               uint8_t brake) {
//   runGyror(true, targetAngle, speed, kp, distanceCm, brake);
// }

// void bw_gyror(float targetAngle, uint8_t speed, float kp, float distanceCm,
//               uint8_t brake) {
//   runGyror(false, targetAngle, speed, kp, distanceCm, brake);
// }

// // Route-command names used throughout the user's sketch.
// void fgyro(float targetAngle, uint8_t speed, float kp, float distanceCm,
//            uint8_t brake) {
//   fw_gyror(targetAngle, speed, kp, distanceCm, brake);
// }

// void bgyro(float targetAngle, uint8_t speed, float kp, float distanceCm,
//            uint8_t brake) {
//   bw_gyror(targetAngle, speed, kp, distanceCm, brake);
// }

// bool set_fgyro_distance_scale(float scale) {
//   if (!isfinite(scale) || scale < 0.1f || scale > 5.0f) return false;
//   gyrorDistanceScale = scale;
//   return true;
// }

