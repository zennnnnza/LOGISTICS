void arm_openDown() {
  servoL(servo_L);
  servoR(servo_R);
  // delay(300);
  servoUpDown(servo_down);
  // delay(100);
}

void arm_down_open() {
  servoL(servo_L);
  servoR(servo_R);
  // delay(300);
  servoUpDown(servo_down);
  // delay(100);
}

void ready() {
  servoL(servo_L - 40);
  servoR(servo_R - 40);
  // delay(300);
  servoUpDown(servo_down);
  // delay(100);
}


void place() {
  delay(100);
  arm_u_lr();
  delay(150);
}

void arm_R() {
  servoL(servo_L - 90);
  servoR(servo_R);
  // delay(300);
  servoUpDown(servo_down);
  // delay(100);
}

void arm_L() {
  servoL(servo_L);
  servoR(servo_R - 90);
  // delay(300);
  servoUpDown(servo_down);
  // delay(100);
}

void arm_u_lr() {
  // delay(100);
  servoUpDown(servo_down + 6);
  // delay(100);
  servoL(servo_L + 35);
  servoR(servo_R + 35);
  // delay(300);
  // delay(100);
}

void arm_u_kib() {
  servoUpDown(servo_down + 5);
  servoL(servo_L - 130);
  servoR(servo_R - 130);
  // delay(300);
  // delay(100);
}
void arm_readyy() {
  servoL(servo_L - 70);
  servoR(servo_R - 70);
  // delay(300);
  servoUpDown(servo_down);
  // delay(100);
}
void arm_closeDown() {
  servoL(servo_L - 130);
  servoR(servo_R - 130);
  // delay(100);
  servoUpDown(servo_down);
  // delay(100);
}

void arm_kib() {
  servoL(servo_L - 130);
  servoR(servo_R - 130);
  // delay(100);
  servoUpDown(servo_down);
  // delay(100);
}

void arm_closeUp() {
  servoL(servo_L - 120);
  servoR(servo_R - 120);
  delay(150);
  servoUpDown(servo_down + 90);
  // delay(100);
}

void up_a() {
  delay(100);
  servoL(servo_L - 90);
  servoR(servo_R - 90);
  delay(200);
  servoUpDown(servo_down + 90);
  delay(100);
}

void down_a() {
  servoUpDown(servo_down);
  // delay(100);
  servoL(servo_L - 90);
  servoR(servo_R - 90);
  // delay(200);
  // delay(100);
}

void arm_upClose() {
  servoUpDown(servo_down + 90);
  // delay(300);
  servoL(servo_L - 130);
  servoR(servo_R - 130);
  // delay(100);
}

void arm_up_close() {
  servoUpDown(servo_down + 90);
  // delay(300);
  servoL(servo_L - 130);
  servoR(servo_R - 130);
  // delay(100);
}

void slow_close() {
  for (int i = 120; i > 30; i--) {
    servoL(i);
    servoR(i);
    delay(5);
  }

}

void servoL(int sl) {
  servo(18, sl);  // GP18: มุมซ้าย = 180° - sl
}
void servoR(int sr) {
  servo(22, 180 - sr);  // GP18: ตั้งมุมตามค่า sr
}
void servoUpDown(int updown) {
  servo(28, 180 - updown);  // GP28: มุมยก = 180° - updown
}