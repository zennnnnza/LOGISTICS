
void towhite()// จุดมาร์คไปยังสีขาว
  {
  robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
  robot.bline(23, 23, 0.5, "br", 'c', 'r', 10, "a4", 1);
  robot.fline(23, 23, 0.5, "fr", 'c', 'p', 20, "a3", 0);
  robot.fline(23, 23, 0.5, "fr", 'c', 'r', 10, "a4", 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 's', 20, "a3", 1);
   robot.fw_gyro(23, 23, 1.5,  18, 1);
   delay(500);
   down();
   delay(500);
   robot.bw_gyro(23, 23, 1.5,  18, 1);
   robot.bline(23, 23, 0.5, "br", 'c', 'r', 20, "a3", 1);
   robot.fline(25, 25, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(25, 25, 0.5, "fr", 'c', 'r', 20, "a3", 1);
   robot.fline(25, 25, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(25, 25, 0.5, "fr", 'c', 'r', 20, "a4", 1);

  }

void toblack()// จุดมาร์คไปยังสีดำ
  {
  robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
  robot.bline(23, 23, 0.5, "br", 'c', 'r', 10, "a4", 1);
  robot.fline(23, 23, 0.5, "fr", 'c', 'r', 10, "a4", 1);
  robot.fline(23, 23, 0.5, "fl", 'c', 's', 20, "a3", 1);
  robot.fw_gyro(23, 23, 1.5,  18, 1);
  delay(500);
  down();
  delay(500);
  robot.bw_gyro(23, 23, 1.5,  18, 1);
  robot.bline(23, 23, 0.5, "br", 'c', 'r', 20, "a3", 1);
  robot.fline(25, 25, 0.5, "fr", 'c', 'r', 20, "a3", 1);
  robot.fline(25, 25, 0.5, "fl", 'c', 'p', 20, "a3", 0);
  robot.fline(25, 25, 0.5, "fr", 'c', 'r', 20, "a4", 1);

  }

void toyellow()// จุดมาร์คไปยังสีเหลือง
  {
  robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
  robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
  robot.bline(23, 23, 0.5, "bl", 'c', 's', 20, "a3", 1);
  robot.bw_gyro(23, 23, 1.5,  18, 1);
  delay(500);
  down();
  delay(500);
  robot.fw_gyro(23, 23, 1.5,  18, 1);
  robot.fline(25, 25, 0.5, "fl", 'c', 'p', 20, "a3", 0);
  robot.fline(25, 25, 0.5, "fl", 'c', 'p', 20, "a3", 0);
  robot.fline(25, 25, 0.5, "fr", 'c', 'r', 20, "a3", 1);

  }

void tored()// จุดมาร์คไปยังสีแดง
  {
  robot.fline(25, 25, 0.5, "fl", 'c', 'p', 20, "a3", 0);
  robot.fline(25, 25, 0.5, "fl", 'c', 'p', 20, "a3", 0);
  robot.fline(23, 23, 0.5, "fl", 'c', 's', 20, "a3", 1);
  robot.fw_gyro(23, 23, 1.5,  18, 1);
  delay(500);
  down();
  delay(500);
  robot.bw_gyro(23, 23, 1.5,  18, 1);
  robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
  robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
  robot.bline(23, 23, 0.5, "br", 'c', 'r', 20, "a3", 1);

  }




































































































































