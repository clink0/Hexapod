#pragma once

//***********************************************************************
// foot_sensors.h - Adaptive foot placement for uneven terrain.
//
// Each leg has a momentary switch at the tip that closes when the foot
// touches the ground (wired active LOW; pins defined in config.h).
//
// apply_foot_contact() is called every frame AFTER the active gait has
// updated current_X/Y/Z.  It overrides current_Z[i] to:
//
//   1. STOP EARLY  — if the foot hits high ground before HOME_Z during
//      the downstroke, the leg locks at that contact depth instead of
//      pushing into the terrain.
//
//   2. PROBE DEEPER — if the leg returns to HOME_Z with no contact
//      (depression or hole), it keeps extending at FOOT_PROBE_RATE
//      mm/frame up to FOOT_PROBE_DEPTH below HOME_Z.
//
// State machine per leg (all using foot_target_Z as the floor):
//
//   SEEKING  (foot_grounded = false)
//     - In swing upstroke: target_Z held at last HOME_Z, no action.
//     - In swing downstroke hitting high ground: latch contact → GROUNDED.
//     - In stance at HOME_Z, no contact: decrement target_Z (probe).
//     - In stance, contact found during probe: latch → GROUNDED.
//
//   GROUNDED (foot_grounded = true)
//     - current_Z clamped to foot_target_Z (gait cannot push deeper).
//     - When foot rises FOOT_LIFT_CLEARANCE above foot_ground_Z with
//       no sensor signal → back to SEEKING, target_Z reset to HOME_Z.
//***********************************************************************


// Returns true when leg i is in its swing (airborne) phase for the
// currently running gait.  Reads the gait case arrays directly.
bool leg_is_swinging(int i)
{
  int active_gait = (mode == 5) ? (auto_obstacle ? 1 : 0) : gait;
  switch(active_gait)
  {
    case 0:  return (tripod_case[i]   == 1);
    case 1:  return (wave_case[i]     == 1);
    case 2:  return (ripple_case[i]   == 1 || ripple_case[i] == 2);
    case 3:  return (tetrapod_case[i] == 1);
    default: return false;
  }
}


// Initialise foot sensor pins and per-leg state.
// Called once from setup().
void setup_foot_sensors()
{
  for(int i = 0; i < 6; i++)
  {
    pinMode(FOOT_PIN[i], INPUT_PULLUP);
    foot_grounded[i]  = false;
    foot_ground_Z[i]  = HOME_Z[i];
    foot_target_Z[i]  = HOME_Z[i];
  }
}


// Reset foot state for all legs — call whenever legs return to home
// (e.g. on GAIT / HOME commands, when reset_position fires).
void reset_foot_contact()
{
  for(int i = 0; i < 6; i++)
  {
    foot_grounded[i]  = false;
    foot_ground_Z[i]  = HOME_Z[i];
    foot_target_Z[i]  = HOME_Z[i];
  }
}


// Override current_Z[i] for all legs based on foot switch readings.
// Must be called AFTER the gait function has written current_Z.
// Only has effect in walking modes (call only when mode == 1 or 5).
void apply_foot_contact()
{
  for(int i = 0; i < 6; i++)
  {
    bool contact  = !digitalRead(FOOT_PIN[i]);  // active LOW
    bool swinging = leg_is_swinging(i);

    // ----------------------------------------------------------------
    // RESET: once the foot clears FOOT_LIFT_CLEARANCE above where it
    // last touched down (with no sensor signal), it is truly airborne.
    // Reset so the next descent starts probing from HOME_Z.
    // ----------------------------------------------------------------
    if(swinging && !contact && current_Z[i] > foot_ground_Z[i] + FOOT_LIFT_CLEARANCE)
    {
      foot_grounded[i] = false;
      foot_target_Z[i] = HOME_Z[i];
    }

    // ----------------------------------------------------------------
    // LATCH: first moment the sensor fires, record the contact depth.
    // This handles both early contact (high ground, during swing) and
    // normal / late contact (flat or low ground, during probe).
    // ----------------------------------------------------------------
    if(contact && !foot_grounded[i])
    {
      foot_grounded[i] = true;
      foot_ground_Z[i] = current_Z[i];
      foot_target_Z[i] = current_Z[i];
    }

    // ----------------------------------------------------------------
    // PROBE: leg is in stance, no contact yet — step deeper each frame
    // to find low or soft ground (up to FOOT_PROBE_DEPTH below HOME_Z).
    // ----------------------------------------------------------------
    if(!foot_grounded[i] && !swinging &&
       foot_target_Z[i] > HOME_Z[i] - FOOT_PROBE_DEPTH)
    {
      foot_target_Z[i] -= FOOT_PROBE_RATE;
    }

    // ----------------------------------------------------------------
    // CLAMP: never let the gait drive the foot lower than the current
    // target floor.  This both prevents pushing into terrain (when
    // grounded) and enforces the probed depth (when seeking).
    // ----------------------------------------------------------------
    if(current_Z[i] < foot_target_Z[i])
      current_Z[i] = foot_target_Z[i];
  }
}
