#pragma once

//***********************************************************************
// debug.h - Serial telemetry printed every frame.
//
// Output format (CSV):  <frame_ms>,<batt_volts>
//   frame_ms   - elapsed time since frame start (should be ~0, spikes = overrun)
//   batt_volts - smoothed battery voltage in volts (e.g. 7.82)
//***********************************************************************

void print_debug()
{
  // CSV format: frame_ms, batt_V, pitch, roll, yaw, accel_rms, step_mult, mode
  currentTime = millis();
  Serial.print(currentTime - previousTime);   Serial.print(",");
  Serial.print(float(batt_voltage) / 100.0);  Serial.print(",");
  Serial.print(imu_pitch, 1);                 Serial.print(",");
  Serial.print(imu_roll,  1);                 Serial.print(",");
  Serial.print(imu_yaw,   1);                 Serial.print(",");
  Serial.print(imu_accel_rms, 0);             Serial.print(",");
  Serial.print(step_height_multiplier, 2);    Serial.print(",");
  Serial.print(mode);
  Serial.print("\n");
}
