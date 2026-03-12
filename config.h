#pragma once

//***********************************************************************
// config.h - Hardware pin assignments, physical dimensions, and
//            tuning constants.  Edit this file to match your wiring
//            and robot geometry; nothing else should need to change.
//***********************************************************************

// --- Battery ---
const int BATT_VOLTAGE = 0;           // 2S LiPo analog input pin (A0)

// --- Ultrasonic & IR Sensor Pins ---
const int TRIG_L = 14; const int ECHO_L = 15;
const int TRIG_R = 16; const int ECHO_R = 17;
const int IR_L   = 18; const int IR_R   = 20;

// --- Servo Pins (odd digital pins 19-53) ---
const int COXA1_SERVO  = 19;
const int FEMUR1_SERVO = 21;
const int TIBIA1_SERVO = 23;
const int COXA2_SERVO  = 25;
const int FEMUR2_SERVO = 27;
const int TIBIA2_SERVO = 29;
const int COXA3_SERVO  = 31;
const int FEMUR3_SERVO = 33;
const int TIBIA3_SERVO = 35;
const int COXA4_SERVO  = 37;
const int FEMUR4_SERVO = 39;
const int TIBIA4_SERVO = 41;
const int COXA5_SERVO  = 43;
const int FEMUR5_SERVO = 45;
const int TIBIA5_SERVO = 47;
const int COXA6_SERVO  = 49;
const int FEMUR6_SERVO = 51;
const int TIBIA6_SERVO = 53;

// --- LED Pins (even digital pins 22-52, red/green pairs every 4 pins) ---
const int RED_LED1   = 22;
const int GREEN_LED1 = 24;
const int RED_LED2   = 26;
const int GREEN_LED2 = 28;
const int RED_LED3   = 30;
const int GREEN_LED3 = 32;
const int RED_LED4   = 34;
const int GREEN_LED4 = 36;
const int RED_LED5   = 38;
const int GREEN_LED5 = 40;
const int RED_LED6   = 42;
const int GREEN_LED6 = 44;
const int RED_LED7   = 46;
const int GREEN_LED7 = 48;
const int RED_LED8   = 50;
const int GREEN_LED8 = 52;

// --- Leg Geometry (mm) ---
const int COXA_LENGTH  = 51;
const int FEMUR_LENGTH = 65;
const int TIBIA_LENGTH = 121;

// --- Motion Limits ---
const int TRAVEL    = 30;             // translate / rotate travel cap (mm / deg)
const long A12DEG   = 209440;         // 12 deg in radians * 1,000,000
const long A30DEG   = 523599;         // 30 deg in radians * 1,000,000

// --- Timing ---
const int FRAME_TIME_MS   = 20;       // main loop period (20 ms = 50 Hz)
const int SENSOR_INTERVAL = 60;       // ms between autonomous sensor pings

// --- Autonomous Obstacle Detection ---
const int SAFE_DIST = 30;             // obstacle threshold (cm)

// --- Home Positions: coxa-to-toe XYZ for each leg (mm) ---
const float HOME_X[6] = {  82.0,   0.0, -82.0,  -82.0,    0.0,  82.0};
const float HOME_Y[6] = {  82.0, 116.0,  82.0,  -82.0, -116.0, -82.0};
const float HOME_Z[6] = { -80.0, -80.0, -80.0,  -80.0,  -80.0, -80.0};

// --- Body Geometry: body-center-to-coxa distances (mm) ---
const float BODY_X[6] = { 110.4,   0.0, -110.4, -110.4,    0.0, 110.4};
const float BODY_Y[6] = {  58.4,  90.8,   58.4,  -58.4,  -90.8, -58.4};
const float BODY_Z[6] = {   0.0,   0.0,    0.0,    0.0,    0.0,   0.0};

// --- Servo Trim Calibration (degrees, applied at write time) ---
const int COXA_CAL[6]  = { 2, -1, -1, -3, -2, -3};
const int FEMUR_CAL[6] = { 4, -2,  0, -1,  0,  0};
const int TIBIA_CAL[6] = { 0, -3, -3, -2, -3, -1};
