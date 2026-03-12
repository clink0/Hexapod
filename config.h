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
const int IR_L   = 18; const int IR_R   =  9;  // NOTE: pin 20 reserved for I2C SDA

// --- Servo Pins ---
// Legs 2-6 use odd digital pins 23-53.
// Leg 1: COXA1=19, TIBIA1=23 follow the pattern.
//         FEMUR1=8  (pin 21 is I2C SCL — reserved for IMU, do not use)
const int COXA1_SERVO  = 19;
const int FEMUR1_SERVO =  8;  // was 21; moved — pin 21 is I2C SCL
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

// --- IMU (ICM-20948 via Qwiic / I2C) ---
// Arduino Mega I2C: SDA = pin 20, SCL = pin 21  (both reserved, do not assign)
// SparkFun Qwiic default I2C address: AD0 high → 0x69
const int  IMU_AD0_VAL          =  1;     // 1 = address 0x69, 0 = 0x68

// Complementary filter: fraction of weight given to gyro integration vs accel.
// Higher = smoother but slower to correct drift.  0.96 is a good starting point.
const float IMU_COMP_ALPHA      = 0.96;

// Body-leveling gain: 0.0 = no correction, 1.0 = full correction.
// Values around 0.7 give smooth leveling without over-rotating on steep slopes.
const float IMU_LEVEL_GAIN      = 0.70;

// Magnetic declination for your location (degrees, + = East, - = West).
// Find yours at: ngdc.noaa.gov/geomag/calculators/magcalc.shtml
const float IMU_MAG_DECLINATION =  0.0;

// Terrain-roughness → step height scaling.
// Accel deviation below ROUGH_LOW (mg) = smooth ground → 1.0x step height.
// Accel deviation above ROUGH_HIGH (mg) = rough ground → IMU_STEP_HEIGHT_MAX.
const float IMU_ROUGH_LOW       =  30.0;  // mg
const float IMU_ROUGH_HIGH      = 200.0;  // mg
const float IMU_STEP_HEIGHT_MAX =   2.0;  // multiplier ceiling

// Tilt thresholds for automatic gait promotion (degrees of combined pitch+roll).
// Hysteresis prevents rapid gait flapping near the threshold.
const float IMU_GAIT_TILT_THRESH =  12.0; // promote tripod → wave above this
const float IMU_GAIT_TILT_HYST   =   4.0; // demote wave → tripod below (thresh - hyst)

// --- Recovery Mode ---
// Stuck is declared when the robot is commanded to walk but stays tilted beyond
// STUCK_TILT_DEG for at least STUCK_CONFIRM_MS milliseconds.
const float STUCK_TILT_DEG       =  20.0;
const int   STUCK_CONFIRM_MS     =  2000; // ms tilted before triggering recovery

// During recovery the robot stands taller and lifts its feet higher.
const float RECOVERY_STANCE_Z    = -40.0; // mm added to all leg Z (extend legs down)
const float RECOVERY_STEP_HEIGHT =   2.5; // step_height_multiplier during recovery

// Recovery phase durations (ms) and overall timeout.
const int RECOVERY_STAND_MS    =  1000;
const int RECOVERY_FORWARD_MS  =  2500;
const int RECOVERY_TURN_MS     =  2000;
const int RECOVERY_BACKWARD_MS =  2500;
const int RECOVERY_FORWARD2_MS =  2500;
const int RECOVERY_TIMEOUT_MS  = 15000; // give up after this many ms total

// Tilt must fall below this to declare success and exit recovery.
const float RECOVERY_EXIT_TILT_DEG = 8.0;

// --- Foot Contact Sensors (momentary switches, active LOW with INPUT_PULLUP) ---
// Leg order: 0=front-right, 1=mid-right, 2=rear-right, 3=rear-left, 4=mid-left, 5=front-left
const int FOOT_PIN[6] = {2, 3, 4, 5, 6, 7};

// How far the foot must rise above the last contact point before the
// grounded state resets for the next step (mm).
const float FOOT_LIFT_CLEARANCE = 5.0;

// Maximum distance to probe below HOME_Z when no ground is found (mm).
// Increase this for very uneven or soft terrain.
const float FOOT_PROBE_DEPTH = 25.0;

// How far to extend the leg per frame while probing (mm).
// At 50 Hz this equals 50 mm/s of probe speed.
const float FOOT_PROBE_RATE = 1.0;

// --- Servo Trim Calibration (degrees, applied at write time) ---
const int COXA_CAL[6]  = { 2, -1, -1, -3, -2, -3};
const int FEMUR_CAL[6] = { 4, -2,  0, -1,  0,  0};
const int TIBIA_CAL[6] = { 0, -3, -3, -2, -3, -1};
