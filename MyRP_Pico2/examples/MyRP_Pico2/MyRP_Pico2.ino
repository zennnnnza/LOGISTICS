#include <MyRP_Pico2.h>
MyRP_Pico2 robot;

int last_servo_28 = 90;
//----------------------------------------------------------------------------->> ตั้งค่ามือจับ
int servo_down = 125;        //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servoL_open = 117;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 120;      //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา
//----------------------------------------------------------------------------->> ตั้งค่ามือจับ
//----------------------------------------------------------------------------->> ตั้งค่ามือจับ


void setup() 
  {
    robot.begin();
    set_motor();
    down();
  
    robot.sw();


box_1();
//box_2();
//box_3();
//box_4();
//box_5();
//box_6();
//towhite();
//toblack();
tored();


















    // robot.fline(40, 40, 0.65, "fl", 'c', 'l', 60, "a2", 40);
    // robot.fline(40, 40, 0.65, "fl", 'c', 'p', 60, "a2", 0); ข้ามเส้น

 

    
  }

void loop() 
  {
  
  //  arm_down_close(); delay(1000);
  //  arm_up_open();delay(1000);
   Serial.print( " my.gyro('z')  " );   Serial.print( robot.gyro('z')); 
    Serial.print( analogRead(26) );   Serial.print( "  26 " );   
    Serial.println( analogRead(27) ); Serial.print( "  27 " );   
    //Serial.println( analogRead(28) ); 
    delay(10);

  }
