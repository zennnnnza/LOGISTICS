
void whiteto_1() 
  {
  
   robot.fw_gyro(20, 20, 1.5,  15, 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 'l', 20, "a3", 20);
   robot.fline(23, 23, 0.5, "fl", 'c', 's', 20, "a3", 20);
   stop();
   delay(500);
   up();
   delay(800);
   robot.fline(23, 23, 0.5, "fl", 'c', 'l', 10, "a4", 20);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 10, "a3", 20);
   robot.fline(23, 23, 0.5, "fl", 'c', 's', 10, "a3", 1);
   robot.fw_gyro(20, 20, 1.5,  18, 1);
   delay(500);
   down();
   delay(500);
   robot.bw_gyro(20, 20, 1.5,  18, 1);
   robot.bline(23, 23, 0.5, "br", 'c', 'r', 20, "a3", 20);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 20, "a3", 20);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 'l', 20, "a4", 20);
   stop();

  }

void blackto_1() 
  {
  
   robot.fw_gyro(20, 20, 1.5,  15, 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 'l', 20, "a3", 20);
   robot.fline(23, 23, 0.5, "fl", 'c', 's', 20, "a3", 20);
   stop();
   delay(500);
   up();
   delay(800);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 's', 20, "a3", 1);
   robot.fw_gyro(20, 20, 1.5,  18, 1);
   delay(500);
   down();
   delay(500);
   robot.bw_gyro(20, 20, 1.5,  18, 1);
   robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
   robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
   robot.bline(23, 23, 0.5, "br", 'c', 'r', 20, "a3", 20);

  }


void yellowto_1() 
  {
  
   robot.fw_gyro(20, 20, 1.5,  15, 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 'l', 20, "a3", 20);
   robot.fline(23, 23, 0.5, "fl", 'c', 's', 20, "a3", 20);
   stop();
   delay(500);
   up();
   delay(800);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 15, "a3", 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 'l', 15, "a3", 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 's', 20, "a3", 20);
   robot.fw_gyro(20, 20, 1.5,  18, 1);
   delay(500);
   down();
   delay(500);
   robot.bw_gyro(20, 20, 1.5,  18, 1);
   robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
   robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
   robot.bline(23, 23, 0.5, "br", 'c', 'r', 20, "a3", 1);
   robot.bline(23, 23, 0.5, "br", 'c', 's', 20, "a3", 20);


  }

void redto_1() 
  {
  
   robot.fw_gyro(20, 20, 1.5,  15, 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 'l', 20, "a3", 1);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 20, "a3", 20);
   stop();
   delay(500);
   up();
   delay(800);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 10, "a3", 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 's', 20, "a3", 20);
   robot.fw_gyro(20, 20, 1.5,  18, 1);
   delay(500);
   down();
   delay(500);
   robot.bw_gyro(20, 20, 1.5,  18, 1);
   robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
   robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
   robot.bline(23, 23, 0.5, "bl", 'c', 'l', 20, "a3", 1);
   robot.bline(23, 23, 0.5, "br", 'c', 's', 20, "a3", 20);

  }

void blueto_1() 
  {
  
   robot.fw_gyro(20, 20, 1.5,  15, 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 'l', 20, "a3", 20);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 20, "a3", 20);
   stop();
   delay(500);
   up();
   delay(800);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 10, "a3", 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 10, "a3", 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 'l', 10, "a4", 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 's', 20, "a3", 1);
   robot.fw_gyro(20, 20, 1.5,  18, 1);
   delay(500);
   down();
   delay(500);
   robot.bw_gyro(20, 20, 1.5,  18, 1);
   robot.bline(23, 23, 0.5, "bl", 'c', 'l', 20, "a3", 1);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 20, "a4", 1);
   robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
   robot.bline(23, 23, 0.5, "bl", 'c', 'l', 20, "a3", 1);
   robot.bline(23, 23, 0.5, "br", 'c', 's', 20, "a3", 20);


  }

void greento_1() 
  {
  
   robot.fw_gyro(20, 20, 1.5,  15, 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 'l', 20, "a3", 20);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 20, "a3", 20);
   stop();
   delay(500);
   up();
   delay(800);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 10, "a3", 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 10, "a3", 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fl", 'c', 'l', 10, "a4", 1);
   robot.fline(23, 23, 0.5, "fl", 'c', 's', 20, "a3", 1);
   robot.fw_gyro(20, 20, 1.5,  18, 1);
   delay(500);
   down();
   delay(500);
   robot.bw_gyro(20, 20, 1.5,  18, 1);
   robot.bline(23, 23, 0.5, "bl", 'c', 'l', 20, "a3", 1);
   robot.fline(23, 23, 0.5, "fr", 'c', 'p', 20, "a3", 0);
   robot.fline(23, 23, 0.5, "fr", 'c', 'r', 20, "a4", 1);
   robot.bline(23, 23, 0.5, "br", 'c', 'p', 20, "a3", 0);
   robot.bline(23, 23, 0.5, "bl", 'c', 'l', 20, "a3", 1);
   robot.bline(23, 23, 0.5, "br", 'c', 's', 20, "a3", 20);
   stop();

  }












