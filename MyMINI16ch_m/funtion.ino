// ---------- อ่าน/นับเส้นสีขณะ f_line วิ่งด้วยระยะ ----------
// นับสีต่อเนื่องเป็นเส้นเดียว และต้องออกจากสีนั้นก่อนจึงนับเส้นถัดไปได้
const uint8_t COLOR_PASS_MAX_LINES = 6;
// แถบสีจริงห่างกัน 5 ซม.; ใช้ 4 ซม. เพื่อเผื่อคลาดเคลื่อนของระยะวิ่ง.
const float COLOR_PASS_MIN_GAP_CM = 4.0f;
// จุดเริ่มวางหุ่นคงที่ก่อนชุดสี 5 ซม. ค่าระยะที่ f_line ประเมินจากการทดสอบจริง
// ของหุ่นนี้มีจุดกึ่งกลางแถบที่ 9,16,23,30,37,44 ซม. (ห่าง 7 ซม.).
const float COLOR_SLOT_FIRST_CENTER_CM = 9.0f;
const float COLOR_SLOT_PITCH_CM = 7.0f;
// ตรวจสีในช่วงกว้าง 5 ซม. ของแต่ละช่อง: หากไม่เจอสีที่ยืนยันได้ตลอดช่วงนี้
// ให้ถือว่าเป็น WHITE (พื้นสนาม). ช่องห่าง 7 ซม. จึงยังไม่ทับกัน.
const float COLOR_SLOT_HALF_WINDOW_CM = 2.5f;
const uint8_t COLOR_SLOT_MIN_COLOR_VOTES = 3;
const float COLOR_PASS_WHITE_REFERENCE_END_CM = 5.5f;
String colorPassLines[COLOR_PASS_MAX_LINES];
uint8_t colorPassCount = 0;

uint8_t color_pass_count() {
  return colorPassCount;
}

static bool colorPassOnLine = false;
static String colorPassCandidate;
static String colorPassLastSaved;
static uint8_t colorPassCandidateSamples = 0;
static uint8_t colorPassClearSamples = 0;
static uint32_t colorPassNextReadMs = 0;
static uint32_t colorPassLastDebugMs = 0;
static bool colorPassScanEnabled = false;
static bool colorPassStarted = false;
static float colorPassLastSavedCm = -1000.0f;
static bool colorPassWhiteIsFirst = true;
static bool colorPassSlotMode = false;
static uint8_t colorPassSlotVotes[COLOR_PASS_MAX_LINES][COLOR_PASS_MAX_LINES] = {};
static uint16_t colorPassSlotMinClear[COLOR_PASS_MAX_LINES] = {};
static uint32_t colorPassWhiteClearSum = 0;
static uint8_t colorPassWhiteClearSamples = 0;

static uint16_t color_pass_black_clear_threshold() {
  // ระหว่างเริ่มต้นยังไม่มีพื้นขาวอ้างอิง ให้ใช้ค่าปลอดภัยเดิม.
  if (colorPassWhiteClearSamples == 0) return 90;
  const uint16_t whiteClear = colorPassWhiteClearSum / colorPassWhiteClearSamples;
  // BLACK ต้องมืดกว่าพื้นขาวอย่างชัดเจน; ปรับตามแสงของรอบนั้นเอง.
  const uint16_t threshold = static_cast<uint16_t>(whiteClear * 0.55f);
  return threshold < 70 ? 70 : threshold;
}

static bool isTrackColor(const String& color) {
  return color == "RED" || color == "YELLOW" ||
         color == "BLUE" || color == "GREEN" ||
         color == "WHITE" || color == "BLACK";
}

static int8_t color_pass_index(const String& color) {
  if (color == "WHITE") return 0;
  if (color == "RED") return 1;
  if (color == "GREEN") return 2;
  if (color == "BLUE") return 3;
  if (color == "BLACK") return 4;
  if (color == "YELLOW") return 5;
  return -1;
}

static const char* color_pass_name(uint8_t index) {
  static const char* const names[COLOR_PASS_MAX_LINES] = {
      "WHITE", "RED", "GREEN", "BLUE", "BLACK", "YELLOW"};
  return index < COLOR_PASS_MAX_LINES ? names[index] : "UNKNOWN";
}

static void color_pass_vote_slot(const String& color, float travelledCm) {
  const int8_t colorIndex = color_pass_index(color);
  if (colorIndex < 0) return;
  for (uint8_t slot = 0; slot < COLOR_PASS_MAX_LINES; ++slot) {
    const float center = COLOR_SLOT_FIRST_CENTER_CM +
                         COLOR_SLOT_PITCH_CM * static_cast<float>(slot);
    if (fabsf(travelledCm - center) <= COLOR_SLOT_HALF_WINDOW_CM) {
      if (tcs0.c < colorPassSlotMinClear[slot]) {
        colorPassSlotMinClear[slot] = tcs0.c;
      }
      if (colorPassSlotVotes[slot][colorIndex] < 255) {
        ++colorPassSlotVotes[slot][colorIndex];
      }
    }
  }
}

static void color_pass_finish_slots() {
  colorPassCount = COLOR_PASS_MAX_LINES;
  for (uint8_t slot = 0; slot < COLOR_PASS_MAX_LINES; ++slot) {
    uint8_t bestColor = 1;  // เริ่มที่ RED; WHITE เป็นพื้นสนาม ไม่ใช่คู่แข่ง
    for (uint8_t color = 2; color < COLOR_PASS_MAX_LINES; ++color) {
      // WHITE คือพื้นสนาม จึงไม่ใช้เป็นคู่แข่งของสีแถบ.
      // ถ้าพบสีอื่นแม้เพียงครั้งเดียว ให้เลือกสีอื่นที่คะแนนมากสุด.
      if (colorPassSlotVotes[slot][color] > colorPassSlotVotes[slot][bestColor]) {
        bestColor = color;
      }
    }
    // สีหลุดเพียง 1-2 sample มักเกิดจากขอบแถบ/แสงสะท้อนของพื้นขาว.
    // ต้องยืนยันอย่างน้อย 3 ครั้ง; ถ้าไม่ถึง ให้ช่องนั้นเป็น WHITE.
    colorPassLines[slot] = colorPassSlotVotes[slot][bestColor] >= COLOR_SLOT_MIN_COLOR_VOTES
        ? color_pass_name(bestColor) : "WHITE";

    // BLACK อาจอ่านได้ไม่ครบ 3 ครั้งขณะวิ่ง แต่ clear ต่ำกว่าพื้นขาวชัดเจน.
    // ตัวอย่างสนามจริง: WHITE minClear ~294, BLACK minClear 47.
    if (colorPassLines[slot] == "WHITE" &&
        colorPassSlotMinClear[slot] <= color_pass_black_clear_threshold()) {
      colorPassLines[slot] = "BLACK";
    }
  }

  // สนามมีสีครบ 6 สีและไม่ซ้ำกัน: เก็บ "สีที่พบครั้งแรก" ของแต่ละสีไว้ก่อน.
  // หากมีสีหาย ให้แทนเฉพาะช่องสีซ้ำเท่านั้น. BLACK ที่หายจะเลือก WHITE ซ้ำ
  // ที่มี minClear ต่ำสุด เพราะดำสะท้อนแสงต่ำกว่าพื้นขาว.
  uint8_t counts[COLOR_PASS_MAX_LINES] = {};
  for (uint8_t slot = 0; slot < COLOR_PASS_MAX_LINES; ++slot) {
    const int8_t index = color_pass_index(colorPassLines[slot]);
    if (index >= 0) ++counts[index];
  }
  for (uint8_t missing = 0; missing < COLOR_PASS_MAX_LINES; ++missing) {
    if (counts[missing] != 0) continue;

    int8_t replacementSlot = -1;
    uint16_t bestScore = 0xFFFF;
    for (uint8_t slot = 0; slot < COLOR_PASS_MAX_LINES; ++slot) {
      const int8_t current = color_pass_index(colorPassLines[slot]);
      if (current < 0 || counts[current] < 2) continue;

      // ห้ามแทนการพบสีครั้งแรก: เลือกเฉพาะ occurrence ซ้ำลำดับหลัง.
      bool seenEarlier = false;
      for (uint8_t earlier = 0; earlier < slot; ++earlier) {
        if (colorPassLines[earlier] == colorPassLines[slot]) {
          seenEarlier = true;
          break;
        }
      }
      if (!seenEarlier) continue;

      const uint16_t score = missing == 4  // BLACK
          ? colorPassSlotMinClear[slot]
          : static_cast<uint16_t>(colorPassSlotVotes[slot][current]);
      if (score < bestScore) {
        bestScore = score;
        replacementSlot = slot;
      }
    }

    if (replacementSlot >= 0) {
      Serial.print(F("Replace duplicate at slot "));
      Serial.print(replacementSlot + 1);
      Serial.print(F(" with missing "));
      Serial.println(color_pass_name(missing));
      const int8_t old = color_pass_index(colorPassLines[replacementSlot]);
      if (old >= 0) --counts[old];
      colorPassLines[replacementSlot] = color_pass_name(missing);
      ++counts[missing];
    }
  }

  Serial.println(F("Saved color order (distance slots):"));
  for (uint8_t slot = 0; slot < COLOR_PASS_MAX_LINES; ++slot) {
    const int8_t colorIndex = color_pass_index(colorPassLines[slot]);
    const uint8_t votes = colorIndex >= 0 ? colorPassSlotVotes[slot][colorIndex] : 0;
    Serial.print(slot + 1);
    Serial.print(F(": "));
    Serial.print(colorPassLines[slot]);
    Serial.print(F(" (votes="));
    Serial.print(votes);
    Serial.print(F(" minClear="));
    Serial.print(colorPassSlotMinClear[slot] == 0xFFFF ? 0 : colorPassSlotMinClear[slot]);
    Serial.println(F(")"));
  }
}

// อ่านและปรับผลสีหนึ่งครั้งด้วยกฎเดียวกับโหมดสแกนระหว่างวิ่ง.
static String color_pass_read_once() {
  if (!tcs0.readFast()) return "UNKNOWN";
  String color = tcs0.getColor();

  if (tcs0.g > 0 && tcs0.b > 0 &&
      static_cast<float>(tcs0.r) >= static_cast<float>(tcs0.g) * 1.30f &&
      static_cast<float>(tcs0.r) >= static_cast<float>(tcs0.b) * 1.30f) {
    color = "RED";
  }
  if (color == "UNKNOWN" && tcs0.c <= 70) color = "BLACK";
  if (color == "BLACK" && tcs0.c >= 70) return "UNKNOWN";
  return color;
}

// อ่านซ้ำ 5 รอบขณะรถหยุดนิ่ง แล้วเลือกสีที่ได้คะแนนสูงสุด.
static String color_pass_read_stable() {
  uint8_t votes[COLOR_PASS_MAX_LINES] = {};
  for (uint8_t sample = 0; sample < 5; ++sample) {
    const String color = color_pass_read_once();
    const int8_t index = color_pass_index(color);
    if (index >= 0 && votes[index] < 255) ++votes[index];
    delay(30);  // มากกว่า integration time 24 ms ของ TCS
  }
  uint8_t best = 0;
  for (uint8_t color = 1; color < COLOR_PASS_MAX_LINES; ++color) {
    if (votes[color] > votes[best]) best = color;
  }
  return votes[best] > 0 ? color_pass_name(best) : "UNKNOWN";
}

// โหมดแม่นยำ: วิ่งเร็วระหว่างจุด แล้วหยุดอ่านสี 5 ครั้งที่กึ่งกลางของแต่ละช่อง.
// ใช้จุดกึ่งกลางที่วัดจากสนามจริง: 9, 16, 23, 30, 37, 44 ซม.
void f_line_color_pass_precise(int sl, int sr, float kp, uint8_t stopPull) {
  color_pass_reset();
  for (uint8_t slot = 0; slot < COLOR_PASS_MAX_LINES; ++slot) {
    const float moveCm = slot == 0 ? COLOR_SLOT_FIRST_CENTER_CM
                                   : COLOR_SLOT_PITCH_CM;
    f_line(sl, sr, kp, moveCm, 0);
    motor(1, 1);  // เบรกให้นิ่งก่อนอ่านสี
    delay(25);

    colorPassLines[slot] = color_pass_read_stable();
    Serial.print(F("Color slot "));
    Serial.print(slot + 1);
    Serial.print(F(": "));
    Serial.println(colorPassLines[slot]);
  }
  colorPassCount = COLOR_PASS_MAX_LINES;

  if (stopPull > 0) {
    motor(-sl, -sr);
    delay(stopPull);
  }
  motor(-1, -1);

  Serial.println(F("Saved color order (precise):"));
  for (uint8_t index = 0; index < colorPassCount; ++index) {
    Serial.print(index + 1);
    Serial.print(F(": "));
    Serial.println(colorPassLines[index]);
  }
}

// เรียกก่อนเริ่ม f_line(..., ระยะ, ...)
void color_pass_reset() {
  colorPassCount = 0;
  colorPassOnLine = false;
  colorPassCandidate = "";
  colorPassLastSaved = "";
  colorPassCandidateSamples = 0;
  colorPassClearSamples = 0;
  colorPassNextReadMs = 0;
  colorPassLastDebugMs = 0;
  colorPassStarted = false;
  colorPassLastSavedCm = -1000.0f;
  colorPassSlotMode = false;
  memset(colorPassSlotVotes, 0, sizeof(colorPassSlotVotes));
  colorPassWhiteClearSum = 0;
  colorPassWhiteClearSamples = 0;
  for (uint8_t slot = 0; slot < COLOR_PASS_MAX_LINES; ++slot) {
    colorPassSlotMinClear[slot] = 0xFFFF;
  }
}

// อ่านทุก 4 ms เพื่อไม่พลาดแถบสีแคบ; TCS ถูกตั้ง integration 24 ms ใน setup().
void color_pass_scan(float travelledCm) {
  const uint32_t now = millis();
  if (now < colorPassNextReadMs) return;
  colorPassNextReadMs = now + 4;

  if (!tcs0.readFast()) {
    if (now - colorPassLastDebugMs >= 500) {
      colorPassLastDebugMs = now;
      Serial.println(F("TCS: waiting for a valid color sample"));
    }
    return;
  }

  // หุ่นเริ่มก่อนชุดสี 5 ซม. ช่วงนี้จึงเป็นพื้นขาวสำหรับอ้างอิงแสงรอบปัจจุบัน.
  if (travelledCm <= COLOR_PASS_WHITE_REFERENCE_END_CM &&
      colorPassWhiteClearSamples < 255) {
    colorPassWhiteClearSum += tcs0.c;
    ++colorPassWhiteClearSamples;
  }
  String color = tcs0.getColor();

  // สีแดงจริงของสนามมี R เด่นกว่า G/B ชัดเจน; กัน fallback สลับ RED เป็น YELLOW.
  if (tcs0.g > 0 && tcs0.b > 0 &&
      static_cast<float>(tcs0.r) >= static_cast<float>(tcs0.g) * 1.30f &&
      static_cast<float>(tcs0.r) >= static_cast<float>(tcs0.b) * 1.30f) {
    color = "RED";
  }

  // แนวคิดเดียวกับโค้ดอาจารย์: ดู clear เพื่อจับ BLACK. ในการตั้งค่านี้
  // ดำจริงที่วิ่งผ่านมี clear แกว่งได้ถึงราว 79 (ไม่ใช่ >5000).
  // ทำเฉพาะ UNKNOWN จึงไม่เปลี่ยน GREEN/BLUE ที่จำแนกได้แล้ว.
  if (color == "UNKNOWN" && tcs0.c <= color_pass_black_clear_threshold()) {
    color = "BLACK";
    Serial.print(F("Force BLACK by clear="));
    Serial.println(tcs0.c);
  }

  // BLACK จาก fallback ที่สว่างเกินไปคือการสับสนกับ BLUE; ข้ามมันไป.
  if (color == "BLACK" && tcs0.c >= color_pass_black_clear_threshold()) return;

  // เก็บคะแนนสีเฉพาะในหน้าต่างของทั้ง 6 ตำแหน่ง จึงไม่สนว่าขาวอยู่ต้น/ท้าย
  // และพื้นขาวระหว่างแถบจะไม่ไปกินช่องอื่น.
  if (colorPassSlotMode) {
    color_pass_vote_slot(color, travelledCm);
    return;
  }

  // ระหว่างวิ่งจะแสดงเฉพาะตอนบันทึก Color line เพื่อไม่ให้ Serial ช้าหรือ log รก.

  if (!isTrackColor(color)) {
    colorPassCandidate = "";
    colorPassCandidateSamples = 0;
    if (colorPassOnLine && ++colorPassClearSamples >= 2) {
      colorPassOnLine = false;
      colorPassClearSamples = 0;
    }
    return;
  }

  colorPassClearSamples = 0;
  if (color != colorPassCandidate) {
    colorPassCandidate = color;
    colorPassCandidateSamples = 1;
    return;
  }
  if (colorPassCandidateSamples < 255) ++colorPassCandidateSamples;
  // หนึ่ง sample ที่พร้อมแล้วก็พอ: ที่ speed 30 สอง sample อาจผ่านแถบแคบไม่ทัน.
  if (colorPassCandidateSamples < 1) return;

  // สีแถบติดกันได้ จึงนับทันทีเมื่อ "สีที่นิ่ง" เปลี่ยนจากสีล่าสุด.
  // หากเป็นสีเดิมต่อเนื่อง จะไม่ถูกนับซ้ำ.
  if (color == colorPassLastSaved) return;

  // เลือกตำแหน่ง WHITE ตามทิศที่วิ่ง:
  // ปกติ WHITE เป็นสีแรก; ถ้าวิ่งย้อนจาก YELLOW จะเป็นสีสุดท้าย.
  if (color == "WHITE") {
    if (!colorPassStarted && !colorPassWhiteIsFirst) return;  // พื้นขาวก่อน YELLOW
    if (colorPassStarted && colorPassWhiteIsFirst) return;    // พื้นขาวระหว่างแถบ
    if (colorPassStarted && !colorPassWhiteIsFirst && colorPassCount < 5) return;
  }

  // เส้นนำทางก่อนถึงแถบแรกมักเป็นดำ: อย่านับ BLACK แรกนั้น.
  // หลังเจอ WHITE/RED/BLUE/GREEN/YELLOW ครั้งแรกแล้ว BLACK จะถูกนับตามปกติ.
  if (!colorPassStarted && color == "BLACK") return;

  // กันสีเดิม/สีรบกวนที่อ่านสลับกันขณะรถยังคร่อมแถบเดียวกัน.
  if (travelledCm - colorPassLastSavedCm < COLOR_PASS_MIN_GAP_CM) return;

  colorPassOnLine = true;
  if (colorPassCount < COLOR_PASS_MAX_LINES) {
    colorPassLines[colorPassCount++] = color;
    colorPassLastSaved = color;
    colorPassStarted = true;
    colorPassLastSavedCm = travelledCm;
    Serial.print(F("Color line "));
    Serial.print(colorPassCount);
    Serial.print(F(": "));
    Serial.print(color);
    Serial.print(F(" at "));
    Serial.print(travelledCm, 1);
    Serial.print(F(" cm | C/R/G/B = "));
    Serial.print(tcs0.c);
    Serial.print('/');
    Serial.print(tcs0.r);
    Serial.print('/');
    Serial.print(tcs0.g);
    Serial.print('/');
    Serial.println(tcs0.b);
    if (tcs0.c > 0) {
      Serial.print(F("  ratio R/G/B = "));
      Serial.print(static_cast<float>(tcs0.r) / tcs0.c, 3);
      Serial.print('/');
      Serial.print(static_cast<float>(tcs0.g) / tcs0.c, 3);
      Serial.print('/');
      Serial.println(static_cast<float>(tcs0.b) / tcs0.c, 3);
    }
  }
}

// ถูกเรียกจาก f_line มาตรฐานของไลบรารีทุกครั้งที่วิ่งแบบกำหนดระยะ.
void line_follower_distance_progress(float travelledCm) {
  if (colorPassScanEnabled) color_pass_scan(travelledCm);
}

// ใช้แทน f_line ปกติเมื่อต้องเก็บสีของเส้นตัดตลอดระยะที่วิ่ง.
// ใช้ f_line แบบ 6 พารามิเตอร์เพื่อให้ผ่านลูปควบคุมใน F.ino ซึ่งเรียก callback ได้.
static void f_line_color_pass_core(int sl, int sr, float kp, float distanceCm,
                                   uint8_t stopPull, bool whiteIsFirst) {
  color_pass_reset();
  colorPassWhiteIsFirst = whiteIsFirst;
  colorPassSlotMode = true;
  colorPassScanEnabled = true;
  set_line_distance_progress_callback(line_follower_distance_progress);
  // เรียก f_line 5 พารามิเตอร์: เป็นตัวมาตรฐานที่รถใช้งานและเกาะเส้นได้ดีอยู่แล้ว.
  f_line(sl, sr, kp, distanceCm, stopPull);
  set_line_distance_progress_callback(nullptr);
  colorPassScanEnabled = false;
  color_pass_finish_slots();
}

// อ่าน 6 ช่องตามระยะ จึงรองรับการเรียงสีแบบสุ่ม.
void f_line_color_pass(int sl, int sr, float kp, float distanceCm,
                       uint8_t stopPull) {
  f_line_color_pass_core(sl, sr, kp, distanceCm, stopPull, true);
}

// ใช้ได้เช่นกันเมื่อเริ่มจากอีกด้าน; ลำดับเก็บตามทิศที่รถวิ่ง.
void f_line_color_pass_reverse(int sl, int sr, float kp, float distanceCm,
                               uint8_t stopPull) {
  f_line_color_pass_core(sl, sr, kp, distanceCm, stopPull, false);
}

// ---------- ทำภารกิจตามรายการสีที่สแกนไว้ ----------
// ให้ tored/toyellow/toblue/togreen แต่ละฟังก์ชันเริ่มและจบที่ "จุดกลางสนาม".
// ภายในนั้นจึงเขียนเส้นทางไปหยิบของ, ไปวางของ และกลับจุดกลางได้เต็มชุด.
String currentTargetColor;
uint8_t currentTargetIndex = 0;

bool run_saved_color(uint8_t index) {
  if (index >= colorPassCount) return false;

  currentTargetIndex = index;
  currentTargetColor = colorPassLines[index];
  Serial.print(F("Run color #"));
  Serial.print(index + 1);
  Serial.print(F(": "));
  Serial.println(currentTargetColor);

  if (currentTargetColor == "RED") {
    tored();
  } else if (currentTargetColor == "YELLOW") {
    toyellow();
  } else if (currentTargetColor == "BLUE") {
    toblue();
  } else if (currentTargetColor == "GREEN") {
    togreen();
  } else {
    return false;
  }
  return true;
}

// เรียกหลังรถมาถึงจุดกลางสนามแล้ว; ทำทีละสีตามลำดับที่สแกนพบ.
void run_all_saved_colors() {
  for (uint8_t index = 0; index < colorPassCount; ++index) {
    if (!run_saved_color(index)) {
      Serial.println(F("Unknown saved color; mission stopped"));
      motor(-1, -1);
      return;
    }
  }
}

void ch_color() 
  {
    if (tcs0.readFast()) 
      {
        Serial.print("TCS0: "); Serial.println(tcs0.getColor());
        if(tcs0.getColor() == "RED")
          {
            tored();// จุดมาร์คไปยังสีแดง
          } 
        else if(tcs0.getColor() == "YELLOW")
          {
            toyellow();// จุดมาร์คไปยังสีเหลือง
          }
        else if(tcs0.getColor() == "BLUE")
          {
            toblue(); // จุดมาร์คไปยังสีน้ำเงิน 
          }
        else if(tcs0.getColor() == "GREEN")
          {
            togreen(); // จุดมาร์คไปยังสีเขียว
          } 
        else 
          {
            if(tcs0._Raw() > 5000)
              {
                towhite();// จุดมาร์คไปยังสีขาว
              } 
            else 
              {
                toblack();// จุดมาร์คไปยังสีดำ
              }
          }           
      }
  }


















void keep_z() {
  arm_u_lr();
  f_line(20, 20, 1.5f, f15, 20);
  do { motor(30, 30); } while (readADC_F(15) < 500);  // F0 ยังเป็นพื้นขาว ให้เดินต่อ
  motor(-1, -1);                                      // เบรกเมื่อ F0 เจอเส้นดำ
  delay(10);
  do { motor(30, -1); } while (readADC_F(15) > 500);  // F0 ยังเป็นพื้นขาว ให้เดินต่อ
  motor(-1, -1);                                      // เบรกเมื่อ F0 เจอเส้นดำ
  delay(10);
  do { motor(-1, 30); } while (readADC_F(15) < 500);  // F0 ยังเป็นพื้นขาว ให้เดินต่อ
  motor(-1, -1);                                      // เบรกเมื่อ F0 เจอเส้นดำ
  delay(10);
  do { motor(-1, 30); } while (readADC_F(15) > 500);  // F0 ยังเป็นพื้นขาว ให้เดินต่อ
  motor(-1, -1);                                      // เบรกเมื่อ F0 เจอเส้นดำ
  delay(10);
  do { motor(-1, 30); } while (readADC_F(15) < 500);  // F0 ยังเป็นพื้นขาว ให้เดินต่อ
  motor(-1, -1);                                      // เบรกเมื่อ F0 เจอเส้นดำ
  delay(10);
  arm_kib();
  delay(200);
  do { motor(1, -30); } while (readADC_F(15) > 500);  // F0 ยังเป็นพื้นขาว ให้เดินต่อ
  motor(-1, -1);                                      // เบรกเมื่อ F0 เจอเส้นดำ
  delay(10);
  do { motor(1, -30); } while (readADC_F(15) < 500);  // F0 ยังเป็นพื้นขาว ให้เดินต่อ
  motor(-1, -1);                                      // เบรกเมื่อ F0 เจอเส้นดำ
  delay(10);
  do { motor(1, -30); } while (readADC_F(15) > 500);  // F0 ยังเป็นพื้นขาว ให้เดินต่อ
  motor(-1, -1);                                      // เบรกเมื่อ F0 เจอเส้นดำ
  delay(10);
  do { motor(1, -30); } while (readADC_F(7) > 500);  // F0 ยังเป็นพื้นขาว ให้เดินต่อ
  motor(-1, -1);                                     // เบรกเมื่อ F0 เจอเส้นดำ
  delay(10);
}

void t_r() {
  do { motor(50, -50); } while (readADC_F(0) > 500);  // F0 ยังเป็นพื้นขาว ให้เดินต่อ
  motor(-1, 1);                                      // เบรกเมื่อ F0 เจอเส้นดำ
  delay(10);
  do { motor(50, -50); } while (readADC_F(15) > 500);  // F0 ยังเป็นพื้นขาว ให้เดินต่อ
  motor(-1, 1);                                      // เบรกเมื่อ F0 เจอเส้นดำ
  delay(10);
  do { motor(50, -50); } while (readADC_F(9) > 500);  // F0 ยังเป็นพื้นขาว ให้เดินต่อ
  motor(-5, 5);                                      // เบรกเมื่อ F0 เจอเส้นดำ
  delay(10);


}
