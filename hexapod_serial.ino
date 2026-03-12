//***********************************************************************
// Hexapod Program
// Code for Arduino Mega 2560
// Serial command control (USB)
// by Mark W, modified for serial control + autonomous obstacle avoidance
//          + 9DoF IMU body leveling + adaptive terrain + recovery mode
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
// Required libraries (install via Arduino Library Manager):
//  - "Servo"  (built-in)
//  - "SparkFun 9DoF IMU Breakout - ICM 20948"  by SparkFun
//
// File layout:
//  config.h       - pin assignments, geometry constants, tuning values
//  globals.h      - all shared variables and Servo objects
//  leds.h         - LED_Bar(), battery_monitor()
//  sensors.h      - get_distance(), autonomous_explore()
//  kinematics.h   - leg_IK()
//  gait.h         - tripod / wave / ripple / tetrapod gaits + helpers
//  control.h      - translate, rotate, one_leg_lift, set_all_90
//  serial_cmd.h   - process_serial(), parse_command()
//  debug.h        - print_debug()
//  foot_sensors.h - adaptive foot placement for uneven terrain
//  imu.h          - ICM-20948 orientation, body leveling, gait advisor
//  recovery.h     - stuck detection and autonomous recovery (mode 6)
//***********************************************************************

#include <Servo.h>
#include <math.h>
#include <Wire.h>

// Local headers — order matters: each file can use symbols from those above it
#include "config.h"
#include "globals.h"
#include "leds.h"
#include "sensors.h"
#include "kinematics.h"
#include "gait.h"
#include "control.h"
#include "serial_cmd.h"
#include "debug.h"
#include "foot_sensors.h"
#include "imu.h"
#include "recovery.h"


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

  // obstacle sensors (ultrasonic + IR)
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(IR_L,   INPUT);  pinMode(IR_R,   INPUT);

  // LED bar
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

  // foot contact sensors (sets INPUT_PULLUP, inits foot_target_Z to HOME_Z)
  setup_foot_sensors();

  // IMU — Wire.begin() is called inside setup_imu()
  setup_imu();

  // initial robot state
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
// Main Loop  (targets ~50 Hz, gated by FRAME_TIME_MS = 20 ms)
//
// Frame order rationale:
//   1. Serial — pick up any new commands before acting on them
//   2. Reset   — apply home position if flagged (clears after one frame)
//   3. IMU     — fresh orientation before IK so leveling has no lag
//   4. IK      — write servo angles using this frame's positions + offsets
//   5. Gait    — update leg positions for the NEXT frame's IK
//   6. Post    — foot contact override, terrain advisor, stuck check
//***********************************************************************
void loop()
{
  currentTime = millis();
  if((currentTime - previousTime) > FRAME_TIME_MS)
  {
    previousTime = currentTime;

    // ---- 1. Serial ----
    process_serial();

    // ---- 2. Reset ----
    if(reset_position == true)
    {
      for(leg_num=0; leg_num<6; leg_num++)
      {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_foot_contact();   // clear probe/ground state alongside leg positions
      reset_position = false;
    }

    // ---- 3. IMU ----
    // Read sensor and run complementary filter.
    // Compute body-leveling offsets from fresh pitch/roll.
    update_imu();
    apply_body_leveling();

    // ---- 4. IK ----
    // Combine: gait position + capture offsets + IMU leveling + body height.
    // z_body_offset is 0 normally; set to RECOVERY_STANCE_Z during mode 6.
    if(mode < 99)
    {
      for(leg_num=0; leg_num<6; leg_num++)
        leg_IK(leg_num,
               current_X[leg_num] + offset_X[leg_num] + level_offset_X[leg_num],
               current_Y[leg_num] + offset_Y[leg_num] + level_offset_Y[leg_num],
               current_Z[leg_num] + offset_Z[leg_num] + level_offset_Z[leg_num] + z_body_offset);
    }

    // Reset leg-lift latch flags whenever we leave mode 4
    if(mode != 4)
    {
      leg1_IK_control = true;
      leg6_IK_control = true;
    }

    battery_monitor();
    print_debug();

    // ---- 5. Gait / mode dispatch ----
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
      // wave when obstacle detected (careful), tripod when clear (fast)
      if(auto_obstacle) wave_gait();
      else              tripod_gait();
    }
    if(mode == 6) update_recovery();   // runs its own wave_gait() call internally
    if(mode == 99) set_all_90();

    // ---- 6. Post-gait ----
    // Foot contact: override Z to match actual terrain height
    if(mode == 1 || mode == 5 || mode == 6)
      apply_foot_contact();

    // IMU terrain advisor: scale step height + promote gait on slopes
    imu_gait_advisor();

    // Stuck detection: enter recovery if tilted while walking too long
    if(mode == 1 || mode == 5)
      check_stuck_and_recover();
  }
}
