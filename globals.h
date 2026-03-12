#pragma once

#include <Servo.h>

//***********************************************************************
// globals.h - All shared state variables and Servo objects.
//             Declared here once; every other file sees them through
//             the single #include chain in the .ino.
//***********************************************************************

// --- Virtual joystick axes (0-255, 128 = center/neutral) ---
// RX/RY = right stick, LX/LY = left stick
int joy_RX = 128;
int joy_RY = 128;
int joy_LX = 128;
int joy_LY = 128;

// --- Serial input buffer ---
String serialBuffer = "";

// --- Frame timing ---
unsigned long currentTime;
unsigned long previousTime;

// --- Autonomous mode ---
unsigned long sensorTimer;            // throttles sensor pings
bool auto_obstacle = false;           // true while an obstacle is in view

// --- Mode / gait control ---
int temp;
int mode;
int gait;
int gait_speed;
int gait_LED_color;
int reset_position;
int capture_offsets;

// --- Battery monitor ---
int  batt_LEDs;
int  batt_voltage;
int  batt_voltage_index;
int  batt_voltage_array[50];
long batt_voltage_sum;

// --- Inverse kinematics scratch variables ---
float L0, L3;
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

// --- One-leg-lift mode ---
int   leg1_IK_control, leg6_IK_control;
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;

// --- Walking / positioning ---
int   leg_num;
int   z_height_LED_color;
int   totalX, totalY, totalZ;
int   tick, duration, numTicks;
int   z_height_left, z_height_right;
int   commandedX, commandedY, commandedR;
int   translateX, translateY, translateZ;
float step_height_multiplier;
float strideX, strideY, strideR;
float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6],  offset_Y[6],  offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];

// --- IMU state (managed by imu.h) ---
float imu_pitch     =  0.0;   // degrees, + = nose up
float imu_roll      =  0.0;   // degrees, + = right side down
float imu_yaw       =  0.0;   // degrees, 0-360 magnetic heading
float imu_accel_rms =  0.0;   // smoothed terrain roughness (mg deviation from 1g)
bool  imu_ok        = false;  // false if IMU not found at startup

// Per-leg body-leveling offsets computed each frame from imu_pitch/roll.
// Added to offset_X/Y/Z in the IK call so they don't pollute the capture offsets.
float level_offset_X[6] = {0,0,0,0,0,0};
float level_offset_Y[6] = {0,0,0,0,0,0};
float level_offset_Z[6] = {0,0,0,0,0,0};

// Extra Z offset applied to all legs in the IK call.
// 0 normally; set negative (e.g. -40) to extend legs and stand taller.
float z_body_offset = 0.0;

// --- Recovery state (managed by recovery.h) ---
// State IDs (mirrored as constants in recovery.h)
int           recovery_state    = 0;    // 0 = REC_IDLE
unsigned long recovery_phase_ms = 0;    // millis() when current phase started
unsigned long recovery_start_ms = 0;    // millis() when recovery was entered
unsigned long stuck_confirm_ms  = 0;    // accumulated ms in stuck condition
int           pre_recovery_mode = 0;    // mode to report from (not restored; safety goes to 0)

// --- Foot contact state (managed by foot_sensors.h) ---
bool  foot_grounded[6] = {false, false, false, false, false, false};
float foot_ground_Z[6];   // Z depth where contact was last detected (init to HOME_Z in setup)
float foot_target_Z[6];   // floor the gait cannot drive below for this step (init to HOME_Z in setup)

// --- Gait phase trackers (reset by GAIT command) ---
int tripod_case[6]   = {1,2,1,2,1,2};
int ripple_case[6]   = {2,6,4,1,3,5};
int wave_case[6]     = {1,2,3,4,5,6};
int tetrapod_case[6] = {1,3,2,1,2,3};

// --- Servo objects (18 total, 3 per leg) ---
Servo coxa1_servo;  Servo femur1_servo;  Servo tibia1_servo;
Servo coxa2_servo;  Servo femur2_servo;  Servo tibia2_servo;
Servo coxa3_servo;  Servo femur3_servo;  Servo tibia3_servo;
Servo coxa4_servo;  Servo femur4_servo;  Servo tibia4_servo;
Servo coxa5_servo;  Servo femur5_servo;  Servo tibia5_servo;
Servo coxa6_servo;  Servo femur6_servo;  Servo tibia6_servo;
