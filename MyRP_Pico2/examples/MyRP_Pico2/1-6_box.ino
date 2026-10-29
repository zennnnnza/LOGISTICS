
void box_1()// 1ไปยังจุดมาร์ค
  {
   robot.fw_gyro(23, 23, 1.5,  15, 1);
   robot.fline(25, 25, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(25, 25, 0.5, "fl", 'c', 'l', 20, "a4", 20);
   robot.fline(25, 25, 0.5, "fr", 'c', 'r', 20, "a3", 10);
   stop();
   delay(500);
   up();
   delay(800);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 10, "a3", 10);
   robot.fline(23, 23, 0.5, "fl", 'c', 's', 10, "a4", 1);

  }

void box_2()// 2ไปยังจุดมาร์ค

  {
   robot.fline(27, 27, 0.5, "fl", 'c', 's', 10, "a4", 10);
   stop();
   delay(500);
   up();
   delay(800);
   robot.bline(23, 23, 0.5, "bl", 'c', 'l', 10, "a4", 1);

  }

void box_3()// 3ไปยังจุดมาร์ค

  {
   robot.fline(25, 25, 0.5, "fl", 'c', 'l', 20, "a4", 1);
   robot.fline(25, 25, 0.5, "fl", 'c', 'l', 20, "a4", 1);
   stop();
   delay(500);
   up();
   delay(800);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 10, "a3", 10);
   robot.bline(23, 23, 0.5, "bl", 'c', 's', 10, "a4", 0);

  }


void box_4()// 4ไปยังจุดมาร์ค

  {
  robot.fline(25, 25, 0.5, "fl", 'c', 'p', 20, "a3", 0);
  robot.fline(25, 25, 0.5, "fr", 'c', 'r', 10, "a3", 10);
  robot.fline(25, 25, 0.5, "fr", 'c', 'r', 10, "a3", 10);
  stop();
  delay(500);
  up();
  delay(800);
  robot.fline(25, 25, 0.5, "fl", 'c', 'p', 20, "a3", 0);
  robot.fline(23, 23, 0.5, "fr", 'c', 'r', 10, "a3", 10);
  robot.fline(27, 27, 0.5, "fl", 'c', 's', 10, "a4", 1);

  }

void box_5()// 5ไปยังจุดมาร์ค

  {

  robot.fline(30, 30, 0.5, "fl", 'c', 'p', 20, "a3", 0);
  robot.fline(30, 30, 0.5, "fl", 'c', 's', 10, "a4", 20);
  stop();
  delay(500);
  up();
  delay(800);
  robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
  robot.bline(23, 23, 0.5, "bl", 'c', 'l', 10, "a4", 1);

  }

void box_6()// 6ไปยังจุดมาร์ค

  {
  robot.fline(30, 30, 0.5, "fl", 'c', 'p', 20, "a3", 0);
  robot.fline(30, 30, 0.5, "fl", 'c', 'l', 10, "a4", 1);
  robot.fline(30, 30, 0.5, "fl", 'c', 's', 10, "a4", 20);
  stop();
  delay(500);
  up();
  delay(800);
  robot.fline(23, 23, 0.5, "fr", 'c', 'r', 10, "a4", 1);
  robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
  robot.bline(23, 23, 0.5, "bl", 'c', 'l', 10, "a4", 1);
  robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
  robot.bline(23, 23, 0.5, "bl", 'c', 's', 10, "a4", 0);

  }








