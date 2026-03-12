#pragma once

#include <Wire.h>
#include "ICM_20948.h"

//***********************************************************************
// imu.h  —  ICM-20948 9DoF IMU integration
//
// Provides three services:
//
//  1. ORIENTATION  (update_imu)
//       Complementary filter fusing gyro integration with accelerometer
//       tilt for pitch and roll.  Tilt-compensated magnetometer for yaw.
//       Results written to: imu_pitch, imu_roll, imu_yaw
//
//  2. BODY LEVELING  (apply_body_leveling)
//       Converts measured pitch/roll into per-leg offsets (level_offset_X/Y/Z)
//       that counter-rotate the body to stay horizontal on slopes.
//       Uses the same rotation math as rotate_control() with no yaw component.
//       Gain controlled by IMU_LEVEL_GAIN in config.h.
//
//  3. TERRAIN ADAPTATION  (imu_gait_advisor)
//       - Scales step_height_multiplier with smoothed accelerometer noise
//         so steps are higher on rough ground.
//       - Auto-promotes gait tripod → wave when tilt > IMU_GAIT_TILT_THRESH,
//         and demotes back to tripod once level (with hysteresis).
//
// Call order in loop():
//   update_imu()          ← before IK
//   apply_body_leveling() ← before IK
//   leg_IK(...)           ← uses level_offset_* and z_body_offset
//   [gait runs]
//   imu_gait_advisor()    ← after gait
//***********************************************************************

ICM_20948_I2C _imu;   // underscore prefix avoids collision with the imu_ globals


//-----------------------------------------------------------------------
// Initialise I2C bus and ICM-20948.  Must be called after Serial.begin.
// Sets imu_ok = true on success, false on failure (graceful degradation).
//-----------------------------------------------------------------------
void setup_imu()
{
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz fast-mode

  _imu.begin(Wire, IMU_AD0_VAL);

  if(_imu.status != ICM_20948_Stat_Ok)
  {
    Serial.println("WARN IMU: ICM-20948 not found — leveling and gait advisor disabled");
    imu_ok = false;
    return;
  }

  // Enable the AK09916 magnetometer (needed for yaw/heading)
  _imu.startupMagnetometer();

  imu_ok = true;
  Serial.println("IMU: ICM-20948 ready");
}


//-----------------------------------------------------------------------
// Read the ICM-20948 and run the complementary filter.
// Non-blocking: returns immediately if no new data is ready.
// Should be called once per main loop frame (every FRAME_TIME_MS ms).
//-----------------------------------------------------------------------
void update_imu()
{
  if(!imu_ok)         return;
  if(!_imu.dataReady()) return;

  _imu.getAGMT();

  const float dt = FRAME_TIME_MS / 1000.0f;   // seconds per frame

  // ---- Accelerometer (mg) ----
  float ax = _imu.accX();
  float ay = _imu.accY();
  float az = _imu.accZ();

  // Tilt angles from gravity vector
  float accel_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;
  float accel_roll  = atan2( ay, az)                  * RAD_TO_DEG;

  // ---- Gyroscope (degrees/sec) ----
  float gx = _imu.gyrX();
  float gy = _imu.gyrY();
  float gz = _imu.gyrZ();

  // ---- Complementary filter ----
  // Gyro term: fast response, corrected each frame by the accel reference
  imu_pitch = IMU_COMP_ALPHA * (imu_pitch + gy * dt) + (1.0f - IMU_COMP_ALPHA) * accel_pitch;
  imu_roll  = IMU_COMP_ALPHA * (imu_roll  + gx * dt) + (1.0f - IMU_COMP_ALPHA) * accel_roll;

  // ---- Tilt-compensated magnetometer heading (yaw) ----
  float mx = _imu.magX();  // µT
  float my = _imu.magY();
  float mz = _imu.magZ();

  float cosR = cos(imu_roll  * DEG_TO_RAD);
  float sinR = sin(imu_roll  * DEG_TO_RAD);
  float cosP = cos(imu_pitch * DEG_TO_RAD);
  float sinP = sin(imu_pitch * DEG_TO_RAD);

  // Project magnetometer into the horizontal plane
  float mx_h = mx * cosP + mz * sinP;
  float my_h = mx * sinR * sinP + my * cosR - mz * sinR * cosP;

  imu_yaw = atan2(-my_h, mx_h) * RAD_TO_DEG + IMU_MAG_DECLINATION;
  if(imu_yaw <   0.0f) imu_yaw += 360.0f;
  if(imu_yaw > 360.0f) imu_yaw -= 360.0f;

  // ---- Terrain roughness ----
  // Deviation of total acceleration magnitude from 1g (= 1000 mg).
  // Smoothed with a heavy low-pass to give a stable terrain texture estimate.
  float accel_mag  = sqrt(sq(ax) + sq(ay) + sq(az));
  float deviation  = abs(accel_mag - 1000.0f);
  imu_accel_rms    = 0.95f * imu_accel_rms + 0.05f * deviation;
}


//-----------------------------------------------------------------------
// Compute per-leg body-leveling offsets from the current imu_pitch/roll.
// Writes level_offset_X/Y/Z[6] which are added to the IK call each frame.
// Zeros all offsets if the IMU is unavailable.
//-----------------------------------------------------------------------
void apply_body_leveling()
{
  if(!imu_ok)
  {
    for(int i = 0; i < 6; i++)
      level_offset_X[i] = level_offset_Y[i] = level_offset_Z[i] = 0.0f;
    return;
  }

  // Counter-rotate: if body tilts +pitch forward, we rotate it back -pitch
  float rX = -imu_roll  * IMU_LEVEL_GAIN * DEG_TO_RAD;  // correction roll
  float rY = -imu_pitch * IMU_LEVEL_GAIN * DEG_TO_RAD;  // correction pitch

  float sRX = sin(rX);  float cRX = cos(rX);
  float sRY = sin(rY);  float cRY = cos(rY);
  // No yaw correction: sRZ = 0, cRZ = 1
  // Simplified rotation (from rotate_control with sRZ=0, cRZ=1):
  //   offX = tX*cRY + tY*sRX*sRY - tZ*cRX*sRY - tX
  //   offY = tY*cRX + tZ*sRX     - tY
  //   offZ = tX*sRY - tY*sRX*cRY + tZ*cRX*cRY - tZ

  for(int ln = 0; ln < 6; ln++)
  {
    float tX = HOME_X[ln] + BODY_X[ln];
    float tY = HOME_Y[ln] + BODY_Y[ln];
    float tZ = HOME_Z[ln] + BODY_Z[ln];

    level_offset_X[ln] =  tX*cRY + tY*sRX*sRY - tZ*cRX*sRY - tX;
    level_offset_Y[ln] =  tY*cRX + tZ*sRX - tY;
    level_offset_Z[ln] =  tX*sRY - tY*sRX*cRY + tZ*cRX*cRY - tZ;
  }
}


//-----------------------------------------------------------------------
// Terrain adaptation — called after the active gait updates positions.
//
//  Step height: scales step_height_multiplier with imu_accel_rms so the
//  robot lifts its feet higher on rough terrain.  Uses a slew-rate limit
//  to prevent sudden lurching.
//
//  Gait promotion: on steep terrain (combined tilt > IMU_GAIT_TILT_THRESH)
//  promotes gait 0 (tripod) to gait 1 (wave) for stability, then demotes
//  back to tripod once level.  Only active in manual walk mode 1 to avoid
//  interfering with autonomous or recovery modes.
//-----------------------------------------------------------------------
void imu_gait_advisor()
{
  if(!imu_ok || (mode != 1 && mode != 5)) return;

  // ---- Step height scaling ----
  float t       = constrain(imu_accel_rms, IMU_ROUGH_LOW, IMU_ROUGH_HIGH);
  float frac    = (t - IMU_ROUGH_LOW) / (IMU_ROUGH_HIGH - IMU_ROUGH_LOW);
  float target  = 1.0f + frac * (IMU_STEP_HEIGHT_MAX - 1.0f);

  // Slew: max 0.05 change per frame (2.5 per second) to avoid lurching
  step_height_multiplier += constrain(target - step_height_multiplier, -0.05f, 0.05f);
  step_height_multiplier  = constrain(step_height_multiplier, 1.0f, IMU_STEP_HEIGHT_MAX);

  // ---- Gait promotion (manual walk mode only) ----
  if(mode != 1) return;

  float tilt = sqrt(sq(imu_pitch) + sq(imu_roll));

  if(tilt > IMU_GAIT_TILT_THRESH && gait == 0)
  {
    // Promote: tripod → wave
    gait = 1;
    int w[6] = {1,2,3,4,5,6};
    memcpy(wave_case, w, sizeof(w));
    tick = 0;
    Serial.println("IMU: steep terrain, promoted to wave gait");
  }
  else if(tilt < (IMU_GAIT_TILT_THRESH - IMU_GAIT_TILT_HYST) && gait == 1)
  {
    // Demote: wave → tripod (only if the user originally had tripod selected)
    gait = 0;
    int t2[6] = {1,2,1,2,1,2};
    memcpy(tripod_case, t2, sizeof(t2));
    tick = 0;
    Serial.println("IMU: level ground, returned to tripod gait");
  }
}
