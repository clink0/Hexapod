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
