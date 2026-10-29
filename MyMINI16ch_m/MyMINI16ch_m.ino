#include <MyMINI_Pro.h>
#include <Wire.h>
#include <my_TCS.h>

#define TCS34725_ADDRESS 0x29  // ปกติคือ 0x29 (ถ้าเป็น 0x39 ให้เปลี่ยนตรงนี้)
// เก็บคาลิเบรตสีที่ EEPROM 0x0180 (0x0080 + 0x0100) เพื่อไม่ชนกับ
// คาลิเบรตเซ็นเซอร์เส้นของ MyMINI_Pro ที่ใช้ EEPROM 0x0000..0x0095.
my_TCS tcs0(0x0100);  // เซ็นเซอร์ตัวแรก บน I2C0 (GP4 SDA, GP5 SCL)



void calibrateSensor(my_TCS& sensor, const char* name) {
  Serial.print("\n=== Calibrate ");
  Serial.print(name);
  Serial.println(" ===");
  Serial.println("เตรียมแสงและวัตถุสีมาตรฐานให้คงที่ก่อนเริ่ม");

  // ฟังก์ชันย่อยสำหรับรอ Enter อย่างปลอดภัย
  auto waitForEnter = []() {
    Serial.println("   กด Enter เมื่อพร้อม...");

    // ล้าง buffer ก่อนรอ
    while (Serial.available() > 0) {
      Serial.read();  // อ่านและทิ้งทุกอย่างที่ค้างอยู่
    }

    // รอจนกว่าจะมีข้อมูลใหม่
    while (Serial.available() == 0) {
      delay(100);  // รอ 100ms แล้วเช็คใหม่ (ประหยัด CPU)
    }

    // อ่านและล้าง buffer ให้หมด (ป้องกัน \r\n หรือตัวอักษรอื่น)
    while (Serial.available() > 0) {
      Serial.read();
    }
  };
  Serial.println("   → Calibrate RED ");
  waitForEnter();
  sensor.calibrateRed();
  Serial.println("   → Calibrate RED เสร็จ");
  Serial.println("   → Calibrate GREEN ");
  waitForEnter();
  sensor.calibrateGreen();
  Serial.println("   → Calibrate GREEN เสร็จ");
  Serial.println("   → Calibrate BLUE ");
  waitForEnter();
  sensor.calibrateBlue();
  Serial.println("   → Calibrate BLUE เสร็จ");
  Serial.println("   → Calibrate YELLOW ");
  waitForEnter();
  sensor.calibrateYellow();
  Serial.println("   → Calibrate YELLOW เสร็จ");
  Serial.println("   → Calibrate WHITE ");
  waitForEnter();
  sensor.calibrateWhite();
  Serial.println("   → Calibrate WHITE เสร็จ");
  Serial.println("   → Calibrate BLACK  ");
  waitForEnter();
  sensor.calibrateBlack();
  Serial.println("   → Calibrate BLACK เสร็จ");

  sensor.calibrateDone();
  Serial.print(name);
  Serial.println(" Calibrate เสร็จสิ้น! ค่าเก็บใน EEPROM แล้ว");
}


bool exitBlackLineStraight(int speed, uint32_t timeoutMs = 5000);

int servo_down = 43;
int servo_L = 165;
int servo_R = 156;

void setup() {
  robot_setup();
  set_line_error_monitor(false);  // ทดสอบสี: เก็บ Serial ไว้แสดงผล Color line ให้ดูง่าย


  Serial.println("=== TCS34725 Dual Calibration Tool ===");
  Serial.println("I2C0: GP4 SDA, GP5 SCL");
  Serial.println("-------------------------------------");

  // ตั้งค่า I2C0
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  // เริ่มเซ็นเซอร์ตัวแรก
  Serial.print("เริ่มต้น TCS0 (I2C0): ");
  if (tcs0.begin(TCS34725_ADDRESS, &Wire)) {
    Serial.println("OK");
    // แถบสีมีความกว้างสั้น: 24 ms เร็วกว่าเดิม 101 ms มาก และ 60x
    // ทำให้ระดับความสว่างใกล้เคียงการตั้งค่าเดิม 101 ms / 16x.
    tcs0.setConfig(TCS_IT_24MS, TCS_GAIN_60X);
  } else {
    Serial.println("ไม่พบ! ตรวจสอบสาย I2C0");
    while (1)
      ;
  }

  wait_button();
  set_line_pid_tuning(0.00f);  // Kd ของ f_line/b_line; Kp ใช้ค่าที่ส่งในคำสั่ง
  // arm_up_close();
  Serial.println(F("START: scan color lines for 50 cm"));
  // วิ่งผ่านต่อเนื่อง: อ่านสีในขณะวิ่ง ไม่หยุดที่แต่ละช่อง.
  f_line_color_pass(20, 20, 0.25f, 50.0f, 10);
  Serial.print(F("Scan finished; colors found: "));
  Serial.println(color_pass_count());
  // //เช็คเสร็จวิ่งต่อปกลางสนาม
  // //

  // run_all_saved_colors();

  
  // คาลิเบรตเสร็จแล้วจึงปิดไว้; เปิดบรรทัดนี้เฉพาะเมื่อต้องการคาลิเบรตสีใหม่.
  // calibrateSensor(tcs0, "TCS0");
  ///////////////////////////////----------------------------//
  // wait_button();
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void loop() {
  // การทดสอบวิ่งทำใน setup() ครั้งเดียว แล้วรถหยุดรอผลใน Serial Monitor.
}

// เดินหน้าออกจากเส้นดำและจัดแนวด้วย F0/F15 ก่อนหยุด
bool exitBlackLineStraight(int speed, uint32_t timeoutMs) {
  // ไลบรารี normalize พื้นขาวไว้ราว 630-660; 300 จะไวเกินตรงขอบเส้น
  const int WHITE_THRESHOLD = 600;
  // ADC แถวขอบเส้นอาจแกว่ง จึงต้องเจอขาวต่อเนื่องก่อนหยุด
  const uint8_t WHITE_CONFIRM_SAMPLES = 3;
  const uint8_t SAMPLE_INTERVAL_MS = 2;
  const int MIN_ALIGN_SPEED = 2;
  const int MAX_ALIGN_CORRECTION = max(1, (speed * 3) / 4);

  const uint32_t startMs = millis();
  uint8_t bothWhiteSamples = 0;

  while (millis() - startMs < timeoutMs) {
    const int32_t leftValue = readADC_F(0);    // F0: ซ้ายสุด
    const int32_t rightValue = readADC_F(15);  // F15: ขวาสุด

    if (leftValue == ADC_VALUE_INVALID || rightValue == ADC_VALUE_INVALID) {
      motor(1, 1);
      return false;
    }

    if (leftValue >= WHITE_THRESHOLD && rightValue >= WHITE_THRESHOLD) {
      if (bothWhiteSamples < WHITE_CONFIRM_SAMPLES) ++bothWhiteSamples;
    } else {
      bothWhiteSamples = 0;
    }
    if (bothWhiteSamples >= WHITE_CONFIRM_SAMPLES) {
      motor(1, 1);  // เบรกทั้งสองล้อ
      return true;
    }

    // อย่าล็อกล้อทันที: ถ้าอ่านขาวก่อนเพราะ ADC แกว่งจะกลับมาแก้ได้เอง.
    // ค่าที่ขาวกว่าจะช้าลง เพื่อให้ล้ออีกฝั่งตามมาถึงพื้นขาวพร้อมกัน.
    const int difference = leftValue - rightValue;
    const int correction = constrain((difference * MAX_ALIGN_CORRECTION) / (1000 - WHITE_THRESHOLD),
                                     -MAX_ALIGN_CORRECTION,
                                     MAX_ALIGN_CORRECTION);
    const int leftSpeed = constrain(speed - correction, MIN_ALIGN_SPEED, speed);
    const int rightSpeed = constrain(speed + correction, MIN_ALIGN_SPEED, speed);

    motor(leftSpeed, rightSpeed);
    delay(SAMPLE_INTERVAL_MS);
  }
  motor(1, 1);  // หมดเวลาแล้วหยุด เพื่อไม่ให้วิ่งต่อโดยไม่ตั้งใจ
  return false;
}
