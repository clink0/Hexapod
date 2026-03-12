#pragma once

//***********************************************************************
// gait.h - Walking gait generators and their shared helpers.
//
//  compute_strides()    - converts joystick commands to stride lengths
//  compute_amplitudes() - per-leg swing amplitudes (called inside gaits)
//
//  tripod_gait()   - 3+3 legs alternating; fastest, least stable
//  wave_gait()     - one leg at a time; slowest, most stable
//  ripple_gait()   - 2-phase ripple; medium speed
//  tetrapod_gait() - 4-leg support; stable medium gait
//
//  All gaits read joy_RY (forward/back), joy_RX (strafe), joy_LX (turn)
//  and update current_X/Y/Z[] for the next IK frame.
//***********************************************************************

void compute_strides()
{
  strideX = 90 * commandedX / 127;
  strideY = 90 * commandedY / 127;
  strideR = 35 * commandedR / 127;

  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));

  duration = (gait_speed == 0) ? 1080 : 3240;
}


void compute_amplitudes()
{
  totalX = HOME_X[leg_num] + BODY_X[leg_num];
  totalY = HOME_Y[leg_num] + BODY_Y[leg_num];

  rotOffsetX = totalY * sinRotZ + totalX * cosRotZ - totalX;
  rotOffsetY = totalY * cosRotZ - totalX * sinRotZ - totalY;

  amplitudeX = constrain((strideX + rotOffsetX) / 2.0, -50, 50);
  amplitudeY = constrain((strideY + rotOffsetY) / 2.0, -50, 50);

  if(abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0;
  else
    amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0;
}


//-----------------------------------------------------------------------
// Tripod gait — legs {0,2,4} and {1,3,5} swing alternately.
// 2 phases per cycle; fastest forward speed.
//-----------------------------------------------------------------------
void tripod_gait()
{
  commandedX = map(joy_RY, 0,255,  127,-127);
  commandedY = map(joy_RX, 0,255, -127, 127);
  commandedR = map(joy_LX, 0,255,  127,-127);

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 2.0);
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(tripod_case[leg_num])
      {
        case 1:   // swing phase
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) tripod_case[leg_num] = 2;
          break;
        case 2:   // stance phase
          current_X[leg_num] = HOME_X[leg_num] + amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) tripod_case[leg_num] = 1;
          break;
      }
    }
    if(tick < numTicks-1) tick++; else tick = 0;
  }
}


//-----------------------------------------------------------------------
// Wave gait — one leg swings at a time in sequence.
// 6 phases per cycle; slowest, most stable — used in autonomous mode
// when an obstacle is detected.
//-----------------------------------------------------------------------
void wave_gait()
{
  commandedX = map(joy_RY, 0,255,  127,-127);
  commandedY = map(joy_RX, 0,255, -127, 127);
  commandedR = map(joy_LX, 0,255,  127,-127);

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0);
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(wave_case[leg_num])
      {
        case 1:   // this leg is swinging
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) wave_case[leg_num] = 6;
          break;
        case 2:   // stance — creep rearward
          current_X[leg_num] -= amplitudeX/numTicks/2.5;
          current_Y[leg_num] -= amplitudeY/numTicks/2.5;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 1;
          break;
        case 3:
          current_X[leg_num] -= amplitudeX/numTicks/2.5;
          current_Y[leg_num] -= amplitudeY/numTicks/2.5;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 2;
          break;
        case 4:
          current_X[leg_num] -= amplitudeX/numTicks/2.5;
          current_Y[leg_num] -= amplitudeY/numTicks/2.5;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 3;
          break;
        case 5:
          current_X[leg_num] -= amplitudeX/numTicks/2.5;
          current_Y[leg_num] -= amplitudeY/numTicks/2.5;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 4;
          break;
        case 6:
          current_X[leg_num] -= amplitudeX/numTicks/2.5;
          current_Y[leg_num] -= amplitudeY/numTicks/2.5;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 5;
          break;
      }
    }
    if(tick < numTicks-1) tick++; else tick = 0;
  }
}


//-----------------------------------------------------------------------
// Ripple gait — pairs of legs swing in a rolling ripple pattern.
// 6 phases; medium speed, good stability.
//-----------------------------------------------------------------------
void ripple_gait()
{
  commandedX = map(joy_RY, 0,255,  127,-127);
  commandedY = map(joy_RX, 0,255, -127, 127);
  commandedR = map(joy_LX, 0,255,  127,-127);

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0);
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(ripple_case[leg_num])
      {
        case 1:   // first half of swing
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/(numTicks*2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/(numTicks*2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/(numTicks*2));
          if(tick >= numTicks-1) ripple_case[leg_num] = 2;
          break;
        case 2:   // second half of swing
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*(numTicks+tick)/(numTicks*2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*(numTicks+tick)/(numTicks*2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*(numTicks+tick)/(numTicks*2));
          if(tick >= numTicks-1) ripple_case[leg_num] = 3;
          break;
        case 3:   // stance — creep rearward
          current_X[leg_num] -= amplitudeX/numTicks/2.0;
          current_Y[leg_num] -= amplitudeY/numTicks/2.0;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 4;
          break;
        case 4:
          current_X[leg_num] -= amplitudeX/numTicks/2.0;
          current_Y[leg_num] -= amplitudeY/numTicks/2.0;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 5;
          break;
        case 5:
          current_X[leg_num] -= amplitudeX/numTicks/2.0;
          current_Y[leg_num] -= amplitudeY/numTicks/2.0;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 6;
          break;
        case 6:
          current_X[leg_num] -= amplitudeX/numTicks/2.0;
          current_Y[leg_num] -= amplitudeY/numTicks/2.0;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 1;
          break;
      }
    }
    if(tick < numTicks-1) tick++; else tick = 0;
  }
}


//-----------------------------------------------------------------------
// Tetrapod gait — 4 legs on the ground at all times.
// 3 phases; good stability at medium speed.
//-----------------------------------------------------------------------
void tetrapod_gait()
{
  commandedX = map(joy_RY, 0,255,  127,-127);
  commandedY = map(joy_RX, 0,255, -127, 127);
  commandedR = map(joy_LX, 0,255,  127,-127);

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 3.0);
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(tetrapod_case[leg_num])
      {
        case 1:   // swing
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 2;
          break;
        case 2:   // stance phase 1
          current_X[leg_num] -= amplitudeX/numTicks;
          current_Y[leg_num] -= amplitudeY/numTicks;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 3;
          break;
        case 3:   // stance phase 2
          current_X[leg_num] -= amplitudeX/numTicks;
          current_Y[leg_num] -= amplitudeY/numTicks;
          current_Z[leg_num]  = HOME_Z[leg_num];
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 1;
          break;
      }
    }
    if(tick < numTicks-1) tick++; else tick = 0;
  }
}
