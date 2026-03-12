#pragma once

//***********************************************************************
// recovery.h  —  Stuck detection and autonomous recovery mode (mode 6)
//
// STUCK DETECTION
//   While the robot is walking (mode 1 or 5) with an active joystick
//   command, if the IMU reports a combined tilt (sqrt(pitch²+roll²))
//   greater than STUCK_TILT_DEG for STUCK_CONFIRM_MS milliseconds the
//   robot is considered stuck (likely high-centred on a rock or unable
//   to get foot purchase).
//
// RECOVERY SEQUENCE  (mode 6)
//   Phase 1  STAND  — centre joystick, stand at maximum height, hold 1 s
//   Phase 2  FORWARD  — wave gait forward at tall stance for 2.5 s
//   Phase 3  TURN  — turn right in place for 2 s
//   Phase 4  BACKWARD  — wave gait backward for 2.5 s
//   Phase 5  FORWARD2  — wave gait forward again for 2.5 s
//   FAILED  — timed out; stop and go idle, print alert
//
//   At any point: if combined tilt falls below RECOVERY_EXIT_TILT_DEG
//   the robot is considered unstuck → exit to mode 0 (idle, success).
//   If total time exceeds RECOVERY_TIMEOUT_MS → exit to mode 0 (failure).
//
// DESIGN NOTES
//   • z_body_offset is set to RECOVERY_STANCE_Z so all six legs extend
//     further, raising the body and giving maximum ground clearance.
//   • step_height_multiplier is set to RECOVERY_STEP_HEIGHT so the wave
//     gait picks feet up much higher than normal.
//   • wave_gait() is called directly in update_recovery() regardless of
//     the gait variable so the user's gait setting is not disturbed.
//   • All constants are in config.h so they are easy to tune.
//***********************************************************************

// Recovery state IDs
const int REC_IDLE      = 0;
const int REC_STAND     = 1;
const int REC_FORWARD   = 2;
const int REC_TURN      = 3;
const int REC_BACKWARD  = 4;
const int REC_FORWARD2  = 5;
const int REC_FAILED    = 6;


//-----------------------------------------------------------------------
// Returns true when the robot's tilt has dropped below the exit threshold.
// Requires imu_ok; always returns false if IMU is unavailable.
//-----------------------------------------------------------------------
bool recovery_is_clear()
{
  if(!imu_ok) return false;
  return sqrt(sq(imu_pitch) + sq(imu_roll)) < RECOVERY_EXIT_TILT_DEG;
}


//-----------------------------------------------------------------------
// Cleanly exit recovery: restore neutral state, go to mode 0 (idle).
// The user must then manually re-enter walk mode; this is intentional
// so the robot does not blindly resume walking after a recovery event.
//-----------------------------------------------------------------------
void exit_recovery(bool success)
{
  mode                   = 0;
  recovery_state         = REC_IDLE;
  stuck_confirm_ms       = 0;
  z_body_offset          = 0.0f;
  step_height_multiplier = 1.0f;
  joy_RX = 128; joy_RY = 128;
  joy_LX = 128; joy_LY = 128;
  reset_position         = true;

  if(success) Serial.println("RECOVERY: unstuck — returning to idle (re-send MODE to resume)");
  else        Serial.println("RECOVERY: timeout — stopped for safety (re-send MODE to resume)");
}


//-----------------------------------------------------------------------
// Enter recovery mode from a normal walking mode.
//-----------------------------------------------------------------------
void enter_recovery()
{
  pre_recovery_mode       = mode;
  mode                    = 6;
  recovery_state          = REC_STAND;
  recovery_start_ms       = millis();
  recovery_phase_ms       = millis();
  stuck_confirm_ms        = 0;

  // Tall stance: extend all legs further to lift the body
  z_body_offset           = RECOVERY_STANCE_Z;
  step_height_multiplier  = RECOVERY_STEP_HEIGHT;

  // Centre joystick so previous motion stops before the sequence starts
  joy_RX = 128; joy_RY = 128; joy_LX = 128; joy_LY = 128;

  // Reset wave gait to a clean state
  int w[6] = {1,2,3,4,5,6};
  memcpy(wave_case, w, sizeof(w));
  tick           = 0;
  reset_position = true;

  Serial.print("RECOVERY: stuck detected (mode ");
  Serial.print(pre_recovery_mode);
  Serial.println(") — entering recovery sequence");
}


//-----------------------------------------------------------------------
// Run the recovery state machine.  Called every frame while mode == 6.
// Drives joystick values and calls wave_gait() directly.
//-----------------------------------------------------------------------
void update_recovery()
{
  unsigned long now          = millis();
  unsigned long phase_elapsed = now - recovery_phase_ms;

  // ---- Global timeout ----
  if((now - recovery_start_ms) > (unsigned long)RECOVERY_TIMEOUT_MS)
  {
    exit_recovery(false);
    return;
  }

  // ---- Success check (skip during the initial stand phase so the robot
  //      has a moment to stabilise before we evaluate tilt) ----
  if(recovery_state != REC_STAND && recovery_is_clear())
  {
    exit_recovery(true);
    return;
  }

  // ---- Phase transitions ----
  switch(recovery_state)
  {
    case REC_STAND:
      // Stand tall, hold still — let the robot settle
      joy_RX = 128; joy_RY = 128; joy_LX = 128; joy_LY = 128;
      if(phase_elapsed >= (unsigned long)RECOVERY_STAND_MS)
      {
        recovery_state    = REC_FORWARD;
        recovery_phase_ms = now;
        Serial.println("RECOVERY: phase → FORWARD");
      }
      break;

    case REC_FORWARD:
      joy_RY = 50;  joy_RX = 128; joy_LX = 128; joy_LY = 128;  // forward
      if(phase_elapsed >= (unsigned long)RECOVERY_FORWARD_MS)
      {
        recovery_state    = REC_TURN;
        recovery_phase_ms = now;
        Serial.println("RECOVERY: phase → TURN");
      }
      break;

    case REC_TURN:
      joy_RY = 128; joy_RX = 128; joy_LX = 200; joy_LY = 128;  // turn right
      if(phase_elapsed >= (unsigned long)RECOVERY_TURN_MS)
      {
        recovery_state    = REC_BACKWARD;
        recovery_phase_ms = now;
        Serial.println("RECOVERY: phase → BACKWARD");
      }
      break;

    case REC_BACKWARD:
      joy_RY = 200; joy_RX = 128; joy_LX = 128; joy_LY = 128;  // backward
      if(phase_elapsed >= (unsigned long)RECOVERY_BACKWARD_MS)
      {
        recovery_state    = REC_FORWARD2;
        recovery_phase_ms = now;
        Serial.println("RECOVERY: phase → FORWARD2");
      }
      break;

    case REC_FORWARD2:
      joy_RY = 50;  joy_RX = 128; joy_LX = 128; joy_LY = 128;  // forward again
      if(phase_elapsed >= (unsigned long)RECOVERY_FORWARD2_MS)
      {
        // All phases exhausted without clearing tilt
        exit_recovery(false);
        return;
      }
      break;

    default:
      exit_recovery(false);
      return;
  }

  // Always use wave gait during recovery — most stable
  wave_gait();
}


//-----------------------------------------------------------------------
// Stuck detector — call every frame while in a walking mode.
// Accumulates time spent in the stuck condition; triggers recovery when
// the threshold is reached.  Resets the accumulator as soon as conditions
// are no longer met so brief stumbles don't accumulate toward a trigger.
//-----------------------------------------------------------------------
void check_stuck_and_recover()
{
  // Only run if IMU is available and we are not already recovering
  if(!imu_ok || recovery_state != REC_IDLE) return;

  float tilt     = sqrt(sq(imu_pitch) + sq(imu_roll));
  bool  walking  = (mode == 1 || mode == 5);
  // commandedX/Y/R are set by the gait functions — non-zero means the
  // user actually wants to move (not just sitting still)
  bool  moving   = (abs(commandedX) > 15 || abs(commandedY) > 15 || abs(commandedR) > 15);

  if(walking && moving && tilt > STUCK_TILT_DEG)
  {
    stuck_confirm_ms += FRAME_TIME_MS;
    if(stuck_confirm_ms >= (unsigned long)STUCK_CONFIRM_MS)
      enter_recovery();
  }
  else
  {
    stuck_confirm_ms = 0;  // reset accumulator on any frame that doesn't qualify
  }
}
