#pragma once

//***********************************************************************
// serial_cmd.h - USB serial command parser.
//
//  process_serial() - call every frame; accumulates bytes into a line
//                     buffer and dispatches complete commands
//  parse_command()  - executes one null-terminated command string
//
// Command reference (also printed by STATUS):
//
//  MODE <0-5,99>  - 0=idle  1=walk  2=translate  3=rotate
//                   4=one_leg_lift  5=autonomous  99=set_all_90
//  GAIT <0-3>     - 0=tripod  1=wave  2=ripple  3=tetrapod
//                   (resets position and stops movement)
//  SPEED <0-1>    - 0=fast  1=slow
//  JOY <RX> <RY> <LX> <LY>  - axes 0-255, 128=center
//  CAPTURE        - lock current translate/rotate offsets
//  CLEAR          - clear all offsets, reset step height
//  HOME           - return to home position, center joysticks
//  STATUS         - print state + command reference
//***********************************************************************

void process_serial()
{
  while(Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if(c == '\n' || c == '\r')
    {
      if(serialBuffer.length() > 0)
      {
        parse_command(serialBuffer);
        serialBuffer = "";
      }
    }
    else
    {
      serialBuffer += c;
    }
  }
}


void parse_command(String cmd)
{
  cmd.trim();
  cmd.toUpperCase();

  // ---- MODE <n> ----
  if(cmd.startsWith("MODE"))
  {
    mode = cmd.substring(5).toInt();
    reset_position = true;
    Serial.print("OK MODE "); Serial.println(mode);
  }

  // ---- GAIT <n> ----
  // Stops movement, selects gait, resets position and all phase counters.
  else if(cmd.startsWith("GAIT"))
  {
    gait = constrain(cmd.substring(5).toInt(), 0, 3);
    mode = 0;
    reset_position = true;
    int t[6] = {1,2,1,2,1,2}; memcpy(tripod_case,   t, sizeof(t));
    int r[6] = {2,6,4,1,3,5}; memcpy(ripple_case,   r, sizeof(r));
    int w[6] = {1,2,3,4,5,6}; memcpy(wave_case,     w, sizeof(w));
    int p[6] = {1,3,2,1,2,3}; memcpy(tetrapod_case, p, sizeof(p));
    tick = 0;
    Serial.print("OK GAIT "); Serial.println(gait);
  }

  // ---- SPEED <n> ----
  else if(cmd.startsWith("SPEED"))
  {
    gait_speed = constrain(cmd.substring(6).toInt(), 0, 1);
    Serial.print("OK SPEED "); Serial.println(gait_speed);
  }

  // ---- JOY <RX> <RY> <LX> <LY> ----
  else if(cmd.startsWith("JOY"))
  {
    String args = cmd.substring(4);
    args.trim();
    int v[4] = {128,128,128,128};
    int idx = 0, start = 0;
    for(int i=0; i<=(int)args.length() && idx<4; i++)
    {
      if(i == (int)args.length() || args[i] == ' ')
      {
        if(i > start)
          v[idx++] = constrain(args.substring(start, i).toInt(), 0, 255);
        start = i + 1;
      }
    }
    joy_RX = v[0]; joy_RY = v[1]; joy_LX = v[2]; joy_LY = v[3];
    Serial.print("OK JOY ");
    Serial.print(joy_RX); Serial.print(" ");
    Serial.print(joy_RY); Serial.print(" ");
    Serial.print(joy_LX); Serial.print(" ");
    Serial.println(joy_LY);
  }

  // ---- CAPTURE ----
  else if(cmd == "CAPTURE")
  {
    capture_offsets = true;
    Serial.println("OK CAPTURE");
  }

  // ---- CLEAR ----
  else if(cmd == "CLEAR")
  {
    for(leg_num=0; leg_num<6; leg_num++)
    {
      offset_X[leg_num] = 0;
      offset_Y[leg_num] = 0;
      offset_Z[leg_num] = 0;
    }
    leg1_IK_control = true;
    leg6_IK_control = true;
    step_height_multiplier = 1.0;
    Serial.println("OK CLEAR");
  }

  // ---- HOME ----
  else if(cmd == "HOME")
  {
    mode = 0;
    reset_position = true;
    joy_RX = 128; joy_RY = 128;
    joy_LX = 128; joy_LY = 128;
    tick = 0;
    Serial.println("OK HOME");
  }

  // ---- STATUS ----
  else if(cmd == "STATUS")
  {
    Serial.println("--- Hexapod Status ---");
    Serial.print("Mode:      "); Serial.println(mode);
    Serial.print("Gait:      "); Serial.println(gait);
    Serial.print("Speed:     "); Serial.println(gait_speed == 0 ? "fast" : "slow");
    Serial.print("Joy RX:    "); Serial.println(joy_RX);
    Serial.print("Joy RY:    "); Serial.println(joy_RY);
    Serial.print("Joy LX:    "); Serial.println(joy_LX);
    Serial.print("Joy LY:    "); Serial.println(joy_LY);
    Serial.print("Batt (V):  "); Serial.println(float(batt_voltage) / 100.0);
    Serial.println("--- Commands ---");
    Serial.println("MODE <0-5,99> | GAIT <0-3> | SPEED <0-1>");
    Serial.println("  Mode 5 = autonomous (wave gait near obstacles, tripod when clear)");
    Serial.println("JOY <RX 0-255> <RY 0-255> <LX 0-255> <LY 0-255>  (128=center)");
    Serial.println("  Walk fwd: JOY 128 50 128 128");
    Serial.println("  Walk bck: JOY 128 200 128 128");
    Serial.println("  Turn L:   JOY 128 128 50 128");
    Serial.println("  Turn R:   JOY 128 128 200 128");
    Serial.println("  Stop:     JOY 128 128 128 128");
    Serial.println("CAPTURE | CLEAR | HOME | STATUS");
  }

  else
  {
    Serial.print("ERR Unknown command: ");
    Serial.println(cmd);
  }
}
