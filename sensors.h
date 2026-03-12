#pragma once

//***********************************************************************
// sensors.h - Ultrasonic distance sensing and autonomous exploration.
//
//  get_distance(trig, echo) - fires HC-SR04, returns median of 3 pings (cm)
//  autonomous_explore()     - reads sensors, steers via joy_ variables,
//                             sets auto_obstacle flag for gait selection
//***********************************************************************

// Returns the median of three ultrasonic distance readings (cm).
// Timeout is 5800 µs (~100 cm max range) — enough for obstacle avoidance
// and keeps worst-case blocking to 35 ms instead of 120 ms per measurement.
float get_distance(int trig, int echo)
{
  float readings[3];
  for(int i=0; i<3; i++)
  {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long dur = pulseIn(echo, HIGH, 5800);
    readings[i] = (dur == 0) ? 200.0 : dur * 0.034 / 2.0;  // cap at 200 cm on timeout
  }
  // sort three values to isolate the median
  if(readings[0] > readings[1]) { float t = readings[0]; readings[0] = readings[1]; readings[1] = t; }
  if(readings[1] > readings[2]) { float t = readings[1]; readings[1] = readings[2]; readings[2] = t; }
  if(readings[0] > readings[1]) { float t = readings[0]; readings[0] = readings[1]; readings[1] = t; }
  return readings[1];
}


// Called every frame in mode 5.  Throttled to SENSOR_INTERVAL ms so
// ultrasonic pings don't dominate the CPU.  Updates joy_RY / joy_LX to
// steer and sets auto_obstacle so the loop can pick the right gait:
//   auto_obstacle == false  →  tripod (fast, open ground)
//   auto_obstacle == true   →  wave   (slow, careful maneuvering)
void autonomous_explore()
{
  if(millis() - sensorTimer < SENSOR_INTERVAL) return;
  sensorTimer = millis();

  float distL = get_distance(TRIG_L, ECHO_L);
  float distR = get_distance(TRIG_R, ECHO_R);
  bool  irL   = !digitalRead(IR_L);   // sensors are active LOW
  bool  irR   = !digitalRead(IR_R);

  if((distL < SAFE_DIST || irL) && (distR < SAFE_DIST || irR))
  {
    // blocked on both sides — back up and turn left
    auto_obstacle = true;
    joy_RY = 200; joy_LX = 50;
    LED_Bar(0, 8);
  }
  else if(distL < SAFE_DIST || irL)
  {
    // obstacle on the left — turn right
    auto_obstacle = true;
    joy_RY = 128; joy_LX = 200;
    LED_Bar(0, 4);
  }
  else if(distR < SAFE_DIST || irR)
  {
    // obstacle on the right — turn left
    auto_obstacle = true;
    joy_RY = 128; joy_LX = 50;
    LED_Bar(0, 4);
  }
  else
  {
    // path clear — walk forward
    auto_obstacle = false;
    joy_RY = 50; joy_LX = 128;
    LED_Bar(1, 8);
  }
}
