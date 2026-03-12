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
  currentTime = millis();
  Serial.print(currentTime - previousTime);
  Serial.print(",");
  Serial.print(float(batt_voltage) / 100.0);
  Serial.print("\n");
}
