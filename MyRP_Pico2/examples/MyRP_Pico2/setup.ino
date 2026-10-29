void set_motor()
  {    
    robot.set_Freq("Coreless_Motors");      // หรือ "DC_Motors"
    robot.sp_factor_cm_s_forFW(1.65);  
    robot.sp_factor_cm_s_forBW(1.65);
    
    
    robot.set_slow_motor(15, 15);     
    robot.set_turn_center_l(-80, 80);
    robot.set_turn_center_r(80, -80);
    robot.set_turn_front_l(-10, 80);
    robot.set_turn_front_r(80, -10);
    robot.set_brake_fc(10, 30);
    robot.set_brake_bc(10, 30);
    robot.set_delay_f(10);



    
  }