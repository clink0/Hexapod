#pragma once

//***********************************************************************
// control.h - Manual body control modes driven by joy_ variables.
//
//  translate_control() - mode 2: shift body XYZ, optionally lock offsets
//  rotate_control()    - mode 3: pitch/roll/yaw body, optionally lock offsets
//  one_leg_lift()      - mode 4: manually position legs 1 & 6; sets step height
//  set_all_90()        - mode 99: drive all servos to 90° for mechanical calibration
//***********************************************************************

void translate_control()
{
  translateX = map(joy_RY, 0,255, -2*TRAVEL, 2*TRAVEL);
  for(leg_num=0; leg_num<6; leg_num++)
    current_X[leg_num] = HOME_X[leg_num] + translateX;

  translateY = map(joy_RX, 0,255, 2*TRAVEL, -2*TRAVEL);
  for(leg_num=0; leg_num<6; leg_num++)
    current_Y[leg_num] = HOME_Y[leg_num] + translateY;

  int rawLY = joy_LY;
  translateZ = (rawLY > 127) ? map(rawLY, 128,255, 0,  TRAVEL)
                              : map(rawLY,   0,127, -3*TRAVEL, 0);
  for(leg_num=0; leg_num<6; leg_num++)
    current_Z[leg_num] = HOME_Z[leg_num] + translateZ;

  if(capture_offsets == true)
  {
    for(leg_num=0; leg_num<6; leg_num++)
    {
      offset_X[leg_num] += translateX;
      offset_Y[leg_num] += translateY;
      offset_Z[leg_num] += translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
    capture_offsets = false;
    mode = 0;
  }
}


void rotate_control()
{
  sinRotX = sin((map(joy_RX, 0,255,  A12DEG, -A12DEG)) / 1000000.0);
  cosRotX = cos((map(joy_RX, 0,255,  A12DEG, -A12DEG)) / 1000000.0);
  sinRotY = sin((map(joy_RY, 0,255,  A12DEG, -A12DEG)) / 1000000.0);
  cosRotY = cos((map(joy_RY, 0,255,  A12DEG, -A12DEG)) / 1000000.0);
  sinRotZ = sin((map(joy_LX, 0,255, -A30DEG,  A30DEG)) / 1000000.0);
  cosRotZ = cos((map(joy_LX, 0,255, -A30DEG,  A30DEG)) / 1000000.0);

  int rawLY = joy_LY;
  translateZ = (rawLY > 127) ? map(rawLY, 128,255, 0,  TRAVEL)
                              : map(rawLY,   0,127, -3*TRAVEL, 0);

  for(int ln=0; ln<6; ln++)
  {
    totalX = HOME_X[ln] + BODY_X[ln];
    totalY = HOME_Y[ln] + BODY_Y[ln];
    totalZ = HOME_Z[ln] + BODY_Z[ln];

    rotOffsetX =  totalX*cosRotY*cosRotZ + totalY*sinRotX*sinRotY*cosRotZ
               +  totalY*cosRotX*sinRotZ - totalZ*cosRotX*sinRotY*cosRotZ
               +  totalZ*sinRotX*sinRotZ - totalX;
    rotOffsetY = -totalX*cosRotY*sinRotZ - totalY*sinRotX*sinRotY*sinRotZ
               +  totalY*cosRotX*cosRotZ + totalZ*cosRotX*sinRotY*sinRotZ
               +  totalZ*sinRotX*cosRotZ - totalY;
    rotOffsetZ =  totalX*sinRotY         - totalY*sinRotX*cosRotY
               +  totalZ*cosRotX*cosRotY - totalZ;

    current_X[ln] = HOME_X[ln] + rotOffsetX;
    current_Y[ln] = HOME_Y[ln] + rotOffsetY;
    current_Z[ln] = HOME_Z[ln] + rotOffsetZ + translateZ;

    if(capture_offsets == true)
    {
      offset_X[ln] += rotOffsetX;
      offset_Y[ln] += rotOffsetY;
      offset_Z[ln] += rotOffsetZ + translateZ;
      current_X[ln] = HOME_X[ln];
      current_Y[ln] = HOME_Y[ln];
      current_Z[ln] = HOME_Z[ln];
    }
  }

  if(capture_offsets == true)
  {
    capture_offsets = false;
    mode = 0;
  }
}


void one_leg_lift()
{
  // latch servo positions on first entry so relative moves work correctly
  if(leg1_IK_control == true)
  {
    leg1_coxa  = coxa1_servo.read();
    leg1_femur = femur1_servo.read();
    leg1_tibia = tibia1_servo.read();
    leg1_IK_control = false;
  }
  if(leg6_IK_control == true)
  {
    leg6_coxa  = coxa6_servo.read();
    leg6_femur = femur6_servo.read();
    leg6_tibia = tibia6_servo.read();
    leg6_IK_control = false;
  }

  // right stick L/R → leg 1 coxa
  temp = map(joy_RX, 0,255, 45,-45);
  coxa1_servo.write(constrain(int(leg1_coxa + temp), 45, 135));

  // right stick U/D → leg 1 femur/tibia lift, or record Z height
  temp = joy_RY;
  if(temp < 117)
  {
    temp = map(temp, 116,0, 0,24);
    femur1_servo.write(constrain(int(leg1_femur + temp),     0, 170));
    tibia1_servo.write(constrain(int(leg1_tibia + 4*temp),   0, 170));
  }
  else
  {
    z_height_right = map(constrain(temp,140,255), 140,255, 1,8);
  }

  // left stick L/R → leg 6 coxa
  temp = map(joy_LX, 0,255, 45,-45);
  coxa6_servo.write(constrain(int(leg6_coxa + temp), 45, 135));

  // left stick U/D → leg 6 femur/tibia lift, or record Z height
  temp = joy_LY;
  if(temp < 117)
  {
    temp = map(temp, 116,0, 0,24);
    femur6_servo.write(constrain(int(leg6_femur + temp),     0, 170));
    tibia6_servo.write(constrain(int(leg6_tibia + 4*temp),   0, 170));
  }
  else
  {
    z_height_left = map(constrain(temp,140,255), 140,255, 1,8);
  }

  if(z_height_left > z_height_right) z_height_right = z_height_left;
  z_height_LED_color = (batt_LEDs > 3) ? 0 : 1;
  LED_Bar(z_height_LED_color, z_height_right);

  if(capture_offsets == true)
  {
    step_height_multiplier = 1.0 + ((z_height_right - 1.0) / 3.0);
    capture_offsets = false;
  }
}


void set_all_90()
{
  coxa1_servo.write(90 + COXA_CAL[0]);
  femur1_servo.write(90 + FEMUR_CAL[0]);
  tibia1_servo.write(90 + TIBIA_CAL[0]);

  coxa2_servo.write(90 + COXA_CAL[1]);
  femur2_servo.write(90 + FEMUR_CAL[1]);
  tibia2_servo.write(90 + TIBIA_CAL[1]);

  coxa3_servo.write(90 + COXA_CAL[2]);
  femur3_servo.write(90 + FEMUR_CAL[2]);
  tibia3_servo.write(90 + TIBIA_CAL[2]);

  coxa4_servo.write(90 + COXA_CAL[3]);
  femur4_servo.write(90 + FEMUR_CAL[3]);
  tibia4_servo.write(90 + TIBIA_CAL[3]);

  coxa5_servo.write(90 + COXA_CAL[4]);
  femur5_servo.write(90 + FEMUR_CAL[4]);
  tibia5_servo.write(90 + TIBIA_CAL[4]);

  coxa6_servo.write(90 + COXA_CAL[5]);
  femur6_servo.write(90 + FEMUR_CAL[5]);
  tibia6_servo.write(90 + TIBIA_CAL[5]);
}
