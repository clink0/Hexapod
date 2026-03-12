//***********************************************************************
// Hexapod Program
// Code for Arduino Mega
// Serial command control (USB)
// by Mark W, modified for serial control + autonomous obstacle avoidance
//***********************************************************************
//
// IK and gait references:
//  https://www.projectsofdan.com/?cat=4
//  http://www.gperco.com/2015/06/hex-inverse-kinematics.html
//  http://virtual-shed.blogspot.com/2012/12/hexapod-inverse-kinematics-part-1.html
//  http://virtual-shed.blogspot.com/2013/01/hexapod-inverse-kinematics-part-2.html
//  https://www.robotshop.com/community/forum/t/inverse-kinematic-equations-for-lynxmotion-3dof-legs/21336
//  http://arduin0.blogspot.com/2012/01/inverse-kinematics-ik-implementation.html
//
// File layout:
//  config.h      - pin assignments, geometry constants, calibration trims
//  globals.h     - all shared variables and Servo objects
//  leds.h        - LED_Bar(), battery_monitor()
//  sensors.h     - get_distance(), autonomous_explore()
//  kinematics.h  - leg_IK()
//  gait.h        - tripod / wave / ripple / tetrapod gaits + helpers
//  control.h     - translate, rotate, one_leg_lift, set_all_90
//  serial_cmd.h  - process_serial(), parse_command()
//  debug.h       - print_debug()
//***********************************************************************

#include <Servo.h>
#include <math.h>

#include "config.h"
#include "globals.h"
#include "leds.h"
#include "sensors.h"
#include "kinematics.h"
#include "gait.h"
#include "control.h"
#include "serial_cmd.h"
#include "debug.h"


//***********************************************************************
// Initialization
//***********************************************************************
void setup()
{
  Serial.begin(115200);
  Serial.println("Hexapod Serial Control Ready");
  Serial.println("Send STATUS for command reference.");

  // attach all 18 servos (pulse range 610-2400 µs)
  coxa1_servo.attach(COXA1_SERVO,610,2400);   femur1_servo.attach(FEMUR1_SERVO,610,2400);   tibia1_servo.attach(TIBIA1_SERVO,610,2400);
  coxa2_servo.attach(COXA2_SERVO,610,2400);   femur2_servo.attach(FEMUR2_SERVO,610,2400);   tibia2_servo.attach(TIBIA2_SERVO,610,2400);
  coxa3_servo.attach(COXA3_SERVO,610,2400);   femur3_servo.attach(FEMUR3_SERVO,610,2400);   tibia3_servo.attach(TIBIA3_SERVO,610,2400);
  coxa4_servo.attach(COXA4_SERVO,610,2400);   femur4_servo.attach(FEMUR4_SERVO,610,2400);   tibia4_servo.attach(TIBIA4_SERVO,610,2400);
  coxa5_servo.attach(COXA5_SERVO,610,2400);   femur5_servo.attach(FEMUR5_SERVO,610,2400);   tibia5_servo.attach(TIBIA5_SERVO,610,2400);
  coxa6_servo.attach(COXA6_SERVO,610,2400);   femur6_servo.attach(FEMUR6_SERVO,610,2400);   tibia6_servo.attach(TIBIA6_SERVO,610,2400);

  // sensor pins
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(IR_L,   INPUT);  pinMode(IR_R,   INPUT);

  // LED pins
  for(int i=0; i<8; i++)
  {
    pinMode((RED_LED1   + (4*i)), OUTPUT);
    pinMode((GREEN_LED1 + (4*i)), OUTPUT);
  }

  // battery monitor rolling-average array
  for(batt_voltage_index=0; batt_voltage_index<50; batt_voltage_index++)
    batt_voltage_array[batt_voltage_index] = 0;
  batt_voltage_sum   = 0;
  batt_voltage_index = 0;

  // clear leg offsets
  for(leg_num=0; leg_num<6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }

  // initial state
  capture_offsets        = false;
  step_height_multiplier = 1.0;
  mode                   = 0;
  gait                   = 0;
  gait_speed             = 0;
  reset_position         = true;
  leg1_IK_control        = true;
  leg6_IK_control        = true;
}


//***********************************************************************
// Main Loop  (runs at ~50 Hz, gated by FRAME_TIME_MS)
//***********************************************************************
void loop()
{
  currentTime = millis();
  if((currentTime - previousTime) > FRAME_TIME_MS)
  {
    previousTime = currentTime;

    process_serial();

    // snap legs to home when requested (mode change, GAIT, HOME commands)
    if(reset_position == true)
    {
      for(leg_num=0; leg_num<6; leg_num++)
      {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false;
    }

    // compute servo angles from current positions (skipped in mode 99)
    if(mode < 99)
    {
      for(leg_num=0; leg_num<6; leg_num++)
        leg_IK(leg_num, current_X[leg_num] + offset_X[leg_num],
                        current_Y[leg_num] + offset_Y[leg_num],
                        current_Z[leg_num] + offset_Z[leg_num]);
    }

    // reset leg-lift latch flags whenever we leave mode 4
    if(mode != 4)
    {
      leg1_IK_control = true;
      leg6_IK_control = true;
    }

    battery_monitor();
    print_debug();

    // --- mode dispatch ---
    // mode 0: idle (home position, no movement)
    if(mode == 1)
    {
      if(gait == 0) tripod_gait();
      if(gait == 1) wave_gait();
      if(gait == 2) ripple_gait();
      if(gait == 3) tetrapod_gait();
    }
    if(mode == 2) translate_control();
    if(mode == 3) rotate_control();
    if(mode == 4) one_leg_lift();
    if(mode == 5)
    {
      autonomous_explore();
      // wave gait when navigating an obstacle (slow & stable),
      // tripod when the path is clear (fast)
      if(auto_obstacle) wave_gait();
      else              tripod_gait();
    }
    if(mode == 99) set_all_90();
  }
}
