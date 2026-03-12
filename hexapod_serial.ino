//***********************************************************************
// Hexapod Program
// Code for Arduino Mega
// Serial command control (USB) - PS2 controller removed
// by Mark W, modified for serial control
//***********************************************************************

//***********************************************************************
// IK and Hexapod gait references:
//  https://www.projectsofdan.com/?cat=4
//  http://www.gperco.com/2015/06/hex-inverse-kinematics.html
//  http://virtual-shed.blogspot.com/2012/12/hexapod-inverse-kinematics-part-1.html
//  http://virtual-shed.blogspot.com/2013/01/hexapod-inverse-kinematics-part-2.html
//  https://www.robotshop.com/community/forum/t/inverse-kinematic-equations-for-lynxmotion-3dof-legs/21336
//  http://arduin0.blogspot.com/2012/01/inverse-kinematics-ik-implementation.html?utm_source=rb-community&utm_medium=forum&utm_campaign=inverse-kinematic-equations-for-lynxmotion-3dof-legs
//***********************************************************************

//***********************************************************************
// Serial Command Reference:
//
//  MODE <n>     - Set mode: 0=idle, 1=walk, 2=translate, 3=rotate,
//                           4=one_leg_lift, 99=set_all_90
//  GAIT <n>     - Select gait: 0=tripod, 1=wave, 2=ripple, 3=tetrapod
//                 (also stops movement and resets position)
//  SPEED <n>    - Set gait speed: 0=fast, 1=slow
//  JOY <RX> <RY> <LX> <LY>
//               - Set joystick axes, all values 0-255 (128 = center/neutral)
//                 RX/RY = right stick (walk Y/X, translate Y/X, rotate pitch/roll)
//                 LX/LY = left stick  (walk rotation, translate Z, rotate yaw/Z)
//  CAPTURE      - Capture/lock current offsets (like L1/R1)
//  CLEAR        - Clear all offsets and reset step height (like L2/R2)
//  HOME         - Reset all legs to home position
//  STATUS       - Print current mode, gait, speed, and joystick values
//
// Example Python usage (pyserial):
//   ser.write(b"MODE 1\n")          # enter walk mode
//   ser.write(b"GAIT 0\n")          # select tripod gait
//   ser.write(b"JOY 128 50 128 128\n")  # walk forward
//   ser.write(b"JOY 128 128 128 128\n") # stop (all centered)
//***********************************************************************

//***********************************************************************
// Includes
//***********************************************************************
#include <Servo.h>
#include <math.h>


//***********************************************************************
// Constant Declarations
//***********************************************************************
const int BATT_VOLTAGE = 0;           //12V Battery analog voltage input port

const int COXA1_SERVO  = 19;          //servo port definitions
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

const int RED_LED1   = 22;            //LED port definitions
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

const int COXA_LENGTH = 51;           //leg part lengths
const int FEMUR_LENGTH = 65;
const int TIBIA_LENGTH = 121;

const int TRAVEL = 30;                //translate and rotate travel limit constant

const long A12DEG = 209440;           //12 degrees in radians x 1,000,000
const long A30DEG = 523599;           //30 degrees in radians x 1,000,000

const int FRAME_TIME_MS = 20;         //frame time (20msec = 50Hz)

const float HOME_X[6] = {  82.0,   0.0, -82.0,  -82.0,    0.0,  82.0};  //coxa-to-toe home positions
const float HOME_Y[6] = {  82.0, 116.0,  82.0,  -82.0, -116.0, -82.0};
const float HOME_Z[6] = { -80.0, -80.0, -80.0,  -80.0,  -80.0, -80.0};

const float BODY_X[6] = { 110.4,  0.0, -110.4, -110.4,    0.0, 110.4};  //body center-to-coxa servo distances
const float BODY_Y[6] = {  58.4, 90.8,   58.4,  -58.4,  -90.8, -58.4};
const float BODY_Z[6] = {   0.0,  0.0,    0.0,    0.0,    0.0,   0.0};

const int COXA_CAL[6]  = {2, -1, -1, -3, -2, -3};                       //servo calibration constants
const int FEMUR_CAL[6] = {4, -2,  0, -1,  0,  0};
const int TIBIA_CAL[6] = {0, -3, -3, -2, -3, -1};


//***********************************************************************
// Variable Declarations
//***********************************************************************

// --- Virtual joystick axes (0-255, 128 = center/neutral) ---
// These replace ps2x.Analog() calls throughout the code.
// RX/RY = right stick, LX/LY = left stick
int joy_RX = 128;
int joy_RY = 128;
int joy_LX = 128;
int joy_LY = 128;

// --- Serial input buffer ---
String serialBuffer = "";

unsigned long currentTime;            //frame timer variables
unsigned long previousTime;

int temp;                             //mode and control variables
int mode;
int gait;
int gait_speed;
int gait_LED_color;
int reset_position;
int capture_offsets;

int batt_LEDs;                        //battery monitor variables
int batt_voltage;
int batt_voltage_index;
int batt_voltage_array[50];
long batt_voltage_sum;

float L0, L3;                         //inverse kinematics variables
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

int leg1_IK_control, leg6_IK_control; //leg lift mode variables
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;

int leg_num;                          //positioning and walking variables
int z_height_LED_color;
int totalX, totalY, totalZ;
int tick, duration, numTicks;
int z_height_left, z_height_right;
int commandedX, commandedY, commandedR;
int translateX, translateY, translateZ;
float step_height_multiplier;
float strideX, strideY, strideR;
float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6], offset_Y[6], offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];

int tripod_case[6]   = {1,2,1,2,1,2};     //for tripod gait walking
int ripple_case[6]   = {2,6,4,1,3,5};     //for ripple gait
int wave_case[6]     = {1,2,3,4,5,6};     //for wave gait
int tetrapod_case[6] = {1,3,2,1,2,3};     //for tetrapod gait


//***********************************************************************
// Object Declarations
//***********************************************************************
Servo coxa1_servo;      //18 servos
Servo femur1_servo;
Servo tibia1_servo;
Servo coxa2_servo;
Servo femur2_servo;
Servo tibia2_servo;
Servo coxa3_servo;
Servo femur3_servo;
Servo tibia3_servo;
Servo coxa4_servo;
Servo femur4_servo;
Servo tibia4_servo;
Servo coxa5_servo;
Servo femur5_servo;
Servo tibia5_servo;
Servo coxa6_servo;
Servo femur6_servo;
Servo tibia6_servo;


//***********************************************************************
// Initialization Routine
//***********************************************************************
void setup()
{
  Serial.begin(115200);
  Serial.println("Hexapod Serial Control Ready");
  Serial.println("Send STATUS for command reference.");

  //attach servos
  coxa1_servo.attach(COXA1_SERVO,610,2400);
  femur1_servo.attach(FEMUR1_SERVO,610,2400);
  tibia1_servo.attach(TIBIA1_SERVO,610,2400);
  coxa2_servo.attach(COXA2_SERVO,610,2400);
  femur2_servo.attach(FEMUR2_SERVO,610,2400);
  tibia2_servo.attach(TIBIA2_SERVO,610,2400);
  coxa3_servo.attach(COXA3_SERVO,610,2400);
  femur3_servo.attach(FEMUR3_SERVO,610,2400);
  tibia3_servo.attach(TIBIA3_SERVO,610,2400);
  coxa4_servo.attach(COXA4_SERVO,610,2400);
  femur4_servo.attach(FEMUR4_SERVO,610,2400);
  tibia4_servo.attach(TIBIA4_SERVO,610,2400);
  coxa5_servo.attach(COXA5_SERVO,610,2400);
  femur5_servo.attach(FEMUR5_SERVO,610,2400);
  tibia5_servo.attach(TIBIA5_SERVO,610,2400);
  coxa6_servo.attach(COXA6_SERVO,610,2400);
  femur6_servo.attach(FEMUR6_SERVO,610,2400);
  tibia6_servo.attach(TIBIA6_SERVO,610,2400);

  //set up LED pins as outputs
  for(int i=0; i<8; i++)
  {
    pinMode((RED_LED1+(4*i)),OUTPUT);
    pinMode((GREEN_LED1+(4*i)),OUTPUT);
  }

  //set up battery monitor average array
  for(batt_voltage_index=0; batt_voltage_index<50; batt_voltage_index++)
    batt_voltage_array[batt_voltage_index] = 0;
  batt_voltage_sum = 0;
  batt_voltage_index = 0;

  //clear offsets
  for(leg_num=0; leg_num<6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }
  capture_offsets = false;
  step_height_multiplier = 1.0;

  //initialize mode and gait variables
  mode = 0;
  gait = 0;
  gait_speed = 0;
  reset_position = true;
  leg1_IK_control = true;
  leg6_IK_control = true;
}


//***********************************************************************
// Main Program
//***********************************************************************
void loop()
{
  //set up frame time
  currentTime = millis();
  if((currentTime - previousTime) > FRAME_TIME_MS)
  {
    previousTime = currentTime;

    //read and process any incoming serial commands
    process_serial();

    //reset legs to home position when commanded
    if(reset_position == true)
    {
      for(leg_num=0; leg_num<6; leg_num++)
      {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false;
    }

    //position legs using IK calculations - unless set all to 90 degrees mode
    if(mode < 99)
    {
      for(leg_num=0; leg_num<6; leg_num++)
        leg_IK(leg_num, current_X[leg_num]+offset_X[leg_num],
                        current_Y[leg_num]+offset_Y[leg_num],
                        current_Z[leg_num]+offset_Z[leg_num]);
    }

    //reset leg lift first pass flags if needed
    if(mode != 4)
    {
      leg1_IK_control = true;
      leg6_IK_control = true;
    }

    battery_monitor();
    print_debug();

    //process modes (mode 0 is default 'home idle' do-nothing mode)
    if(mode == 1)
    {
      if(gait == 0) tripod_gait();
      if(gait == 1) wave_gait();
      if(gait == 2) ripple_gait();
      if(gait == 3) tetrapod_gait();
    }
    if(mode == 2) translate_control();
    if(mode == 3) rotate_control();
    if(mode == 4) one_leg_lift();
    if(mode == 99) set_all_90();
  }
}


//***********************************************************************
// Process incoming serial commands
// Called every frame; reads all available bytes into a line buffer,
// then parses complete newline-terminated commands.
//***********************************************************************
void process_serial()
{
  //read all available bytes into the line buffer
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


//***********************************************************************
// Parse and execute a single command string
//***********************************************************************
void parse_command(String cmd)
{
  cmd.trim();
  cmd.toUpperCase();

  // ---- MODE <n> ----
  if(cmd.startsWith("MODE"))
  {
    int newMode = cmd.substring(5).toInt();
    mode = newMode;
    reset_position = true;
    Serial.print("OK MODE ");
    Serial.println(mode);
  }

  // ---- GAIT <n> ----
  // Stops movement, selects gait, resets position (mirrors D-pad behavior)
  else if(cmd.startsWith("GAIT"))
  {
    int newGait = cmd.substring(5).toInt();
    gait = constrain(newGait, 0, 3);
    mode = 0;
    reset_position = true;
    // reset gait case arrays to their defaults
    int t[6] = {1,2,1,2,1,2}; memcpy(tripod_case,   t, sizeof(t));
    int r[6] = {2,6,4,1,3,5}; memcpy(ripple_case,   r, sizeof(r));
    int w[6] = {1,2,3,4,5,6}; memcpy(wave_case,     w, sizeof(w));
    int p[6] = {1,3,2,1,2,3}; memcpy(tetrapod_case, p, sizeof(p));
    tick = 0;
    Serial.print("OK GAIT ");
    Serial.println(gait);
  }

  // ---- SPEED <n> ----
  else if(cmd.startsWith("SPEED"))
  {
    gait_speed = constrain(cmd.substring(6).toInt(), 0, 1);
    Serial.print("OK SPEED ");
    Serial.println(gait_speed);
  }

  // ---- JOY <RX> <RY> <LX> <LY> ----
  // All values 0-255, 128 = center/neutral
  else if(cmd.startsWith("JOY"))
  {
    String args = cmd.substring(4);
    args.trim();
    int v[4] = {128,128,128,128};
    int idx = 0;
    int start = 0;
    for(int i=0; i<=args.length() && idx<4; i++)
    {
      if(i == (int)args.length() || args[i] == ' ')
      {
        if(i > start)
          v[idx++] = constrain(args.substring(start, i).toInt(), 0, 255);
        start = i + 1;
      }
    }
    joy_RX = v[0];
    joy_RY = v[1];
    joy_LX = v[2];
    joy_LY = v[3];
    Serial.print("OK JOY ");
    Serial.print(joy_RX); Serial.print(" ");
    Serial.print(joy_RY); Serial.print(" ");
    Serial.print(joy_LX); Serial.print(" ");
    Serial.println(joy_LY);
  }

  // ---- CAPTURE ----
  // Lock in current translate/rotate offsets (mirrors L1/R1)
  else if(cmd == "CAPTURE")
  {
    capture_offsets = true;
    Serial.println("OK CAPTURE");
  }

  // ---- CLEAR ----
  // Clear all offsets and reset step height (mirrors L2/R2)
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
  // Reset all legs to home position
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
    Serial.print("Batt (V):  "); Serial.println(float(batt_voltage)/100.0);
    Serial.println("--- Commands ---");
    Serial.println("MODE <0-4,99> | GAIT <0-3> | SPEED <0-1>");
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


//***********************************************************************
// Leg IK Routine
//***********************************************************************
void leg_IK(int leg_number, float X, float Y, float Z)
{
  //compute target femur-to-toe (L3) length
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
  L3 = sqrt(sq(L0) + sq(Z));

  //process only if reach is within possible range
  if((L3 < (TIBIA_LENGTH+FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH-FEMUR_LENGTH)))
  {
    //compute tibia angle
    phi_tibia = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3))/(2*FEMUR_LENGTH*TIBIA_LENGTH));
    theta_tibia = phi_tibia*RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia,0.0,180.0);

    //compute femur angle
    gamma_femur = atan2(Z,L0);
    phi_femur = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH))/(2*FEMUR_LENGTH*L3));
    theta_femur = (phi_femur + gamma_femur)*RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur,0.0,180.0);

    //compute coxa angle
    theta_coxa = atan2(X,Y)*RAD_TO_DEG + COXA_CAL[leg_number];

    //output to the appropriate leg
    switch(leg_number)
    {
      case 0:
        if(leg1_IK_control == true)
        {
          theta_coxa = theta_coxa + 45.0;
          theta_coxa = constrain(theta_coxa,0.0,180.0);
          coxa1_servo.write(int(theta_coxa));
          femur1_servo.write(int(theta_femur));
          tibia1_servo.write(int(theta_tibia));
        }
        break;
      case 1:
        theta_coxa = theta_coxa + 90.0;
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa2_servo.write(int(theta_coxa));
        femur2_servo.write(int(theta_femur));
        tibia2_servo.write(int(theta_tibia));
        break;
      case 2:
        theta_coxa = theta_coxa + 135.0;
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa3_servo.write(int(theta_coxa));
        femur3_servo.write(int(theta_femur));
        tibia3_servo.write(int(theta_tibia));
        break;
      case 3:
        if(theta_coxa < 0)
          theta_coxa = theta_coxa + 225.0;
        else
          theta_coxa = theta_coxa - 135.0;
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa4_servo.write(int(theta_coxa));
        femur4_servo.write(int(theta_femur));
        tibia4_servo.write(int(theta_tibia));
        break;
      case 4:
        if(theta_coxa < 0)
          theta_coxa = theta_coxa + 270.0;
        else
          theta_coxa = theta_coxa - 90.0;
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa5_servo.write(int(theta_coxa));
        femur5_servo.write(int(theta_femur));
        tibia5_servo.write(int(theta_tibia));
        break;
      case 5:
        if(leg6_IK_control == true)
        {
          if(theta_coxa < 0)
            theta_coxa = theta_coxa + 315.0;
          else
            theta_coxa = theta_coxa - 45.0;
          theta_coxa = constrain(theta_coxa,0.0,180.0);
          coxa6_servo.write(int(theta_coxa));
          femur6_servo.write(int(theta_femur));
          tibia6_servo.write(int(theta_tibia));
        }
        break;
    }
  }
}


//***********************************************************************
// Tripod Gait
//***********************************************************************
void tripod_gait()
{
  commandedX = map(joy_RY, 0,255, 127,-127);
  commandedY = map(joy_RX, 0,255, -127,127);
  commandedR = map(joy_LX, 0,255, 127,-127);

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 2.0);
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(tripod_case[leg_num])
      {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) tripod_case[leg_num] = 2;
          break;
        case 2:
          current_X[leg_num] = HOME_X[leg_num] + amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) tripod_case[leg_num] = 1;
          break;
      }
    }
    if(tick < numTicks-1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Wave Gait
//***********************************************************************
void wave_gait()
{
  commandedX = map(joy_RY, 0,255, 127,-127);
  commandedY = map(joy_RX, 0,255, -127,127);
  commandedR = map(joy_LX, 0,255, 127,-127);

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0);
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(wave_case[leg_num])
      {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) wave_case[leg_num] = 6;
          break;
        case 2:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 1;
          break;
        case 3:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 2;
          break;
        case 4:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 3;
          break;
        case 5:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 4;
          break;
        case 6:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 5;
          break;
      }
    }
    if(tick < numTicks-1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Ripple Gait
//***********************************************************************
void ripple_gait()
{
  commandedX = map(joy_RY, 0,255, 127,-127);
  commandedY = map(joy_RX, 0,255, -127,127);
  commandedR = map(joy_LX, 0,255, 127,-127);

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0);
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(ripple_case[leg_num])
      {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/(numTicks*2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/(numTicks*2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/(numTicks*2));
          if(tick >= numTicks-1) ripple_case[leg_num] = 2;
          break;
        case 2:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*(numTicks+tick)/(numTicks*2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*(numTicks+tick)/(numTicks*2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*(numTicks+tick)/(numTicks*2));
          if(tick >= numTicks-1) ripple_case[leg_num] = 3;
          break;
        case 3:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 4;
          break;
        case 4:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 5;
          break;
        case 5:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 6;
          break;
        case 6:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 1;
          break;
      }
    }
    if(tick < numTicks-1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Tetrapod Gait
//***********************************************************************
void tetrapod_gait()
{
  commandedX = map(joy_RY, 0,255, 127,-127);
  commandedY = map(joy_RX, 0,255, -127,127);
  commandedR = map(joy_LX, 0,255, 127,-127);

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 3.0);
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(tetrapod_case[leg_num])
      {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 2;
          break;
        case 2:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 3;
          break;
        case 3:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 1;
          break;
      }
    }
    if(tick < numTicks-1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Compute walking stride lengths
//***********************************************************************
void compute_strides()
{
  strideX = 90*commandedX/127;
  strideY = 90*commandedY/127;
  strideR = 35*commandedR/127;

  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));

  if(gait_speed == 0) duration = 1080;
  else duration = 3240;
}


//***********************************************************************
// Compute walking amplitudes
//***********************************************************************
void compute_amplitudes()
{
  totalX = HOME_X[leg_num] + BODY_X[leg_num];
  totalY = HOME_Y[leg_num] + BODY_Y[leg_num];

  rotOffsetX = totalY*sinRotZ + totalX*cosRotZ - totalX;
  rotOffsetY = totalY*cosRotZ - totalX*sinRotZ - totalY;

  amplitudeX = ((strideX + rotOffsetX)/2.0);
  amplitudeY = ((strideY + rotOffsetY)/2.0);
  amplitudeX = constrain(amplitudeX,-50,50);
  amplitudeY = constrain(amplitudeY,-50,50);

  if(abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0;
  else
    amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0;
}


//***********************************************************************
// Body translate control (xyz axes) - driven by joy_ variables
//***********************************************************************
void translate_control()
{
  translateX = map(joy_RY, 0,255, -2*TRAVEL, 2*TRAVEL);
  for(leg_num=0; leg_num<6; leg_num++)
    current_X[leg_num] = HOME_X[leg_num] + translateX;

  translateY = map(joy_RX, 0,255, 2*TRAVEL, -2*TRAVEL);
  for(leg_num=0; leg_num<6; leg_num++)
    current_Y[leg_num] = HOME_Y[leg_num] + translateY;

  int rawLY = joy_LY;
  if(rawLY > 127)
    translateZ = map(rawLY, 128,255, 0, TRAVEL);
  else
    translateZ = map(rawLY, 0,127, -3*TRAVEL, 0);
  for(leg_num=0; leg_num<6; leg_num++)
    current_Z[leg_num] = HOME_Z[leg_num] + translateZ;

  if(capture_offsets == true)
  {
    for(leg_num=0; leg_num<6; leg_num++)
    {
      offset_X[leg_num] = offset_X[leg_num] + translateX;
      offset_Y[leg_num] = offset_Y[leg_num] + translateY;
      offset_Z[leg_num] = offset_Z[leg_num] + translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
    capture_offsets = false;
    mode = 0;
  }
}


//***********************************************************************
// Body rotate control (xyz axes) - driven by joy_ variables
//***********************************************************************
void rotate_control()
{
  sinRotX = sin((map(joy_RX, 0,255, A12DEG,-A12DEG))/1000000.0);
  cosRotX = cos((map(joy_RX, 0,255, A12DEG,-A12DEG))/1000000.0);
  sinRotY = sin((map(joy_RY, 0,255, A12DEG,-A12DEG))/1000000.0);
  cosRotY = cos((map(joy_RY, 0,255, A12DEG,-A12DEG))/1000000.0);
  sinRotZ = sin((map(joy_LX, 0,255, -A30DEG,A30DEG))/1000000.0);
  cosRotZ = cos((map(joy_LX, 0,255, -A30DEG,A30DEG))/1000000.0);

  int rawLY = joy_LY;
  if(rawLY > 127)
    translateZ = map(rawLY, 128,255, 0, TRAVEL);
  else
    translateZ = map(rawLY, 0,127, -3*TRAVEL, 0);

  for(int ln=0; ln<6; ln++)
  {
    totalX = HOME_X[ln] + BODY_X[ln];
    totalY = HOME_Y[ln] + BODY_Y[ln];
    totalZ = HOME_Z[ln] + BODY_Z[ln];

    rotOffsetX =  totalX*cosRotY*cosRotZ + totalY*sinRotX*sinRotY*cosRotZ + totalY*cosRotX*sinRotZ - totalZ*cosRotX*sinRotY*cosRotZ + totalZ*sinRotX*sinRotZ - totalX;
    rotOffsetY = -totalX*cosRotY*sinRotZ - totalY*sinRotX*sinRotY*sinRotZ + totalY*cosRotX*cosRotZ + totalZ*cosRotX*sinRotY*sinRotZ + totalZ*sinRotX*cosRotZ - totalY;
    rotOffsetZ =  totalX*sinRotY         - totalY*sinRotX*cosRotY                                  + totalZ*cosRotX*cosRotY                                  - totalZ;

    current_X[ln] = HOME_X[ln] + rotOffsetX;
    current_Y[ln] = HOME_Y[ln] + rotOffsetY;
    current_Z[ln] = HOME_Z[ln] + rotOffsetZ + translateZ;

    if(capture_offsets == true)
    {
      offset_X[ln] = offset_X[ln] + rotOffsetX;
      offset_Y[ln] = offset_Y[ln] + rotOffsetY;
      offset_Z[ln] = offset_Z[ln] + rotOffsetZ + translateZ;
      current_X[ln] = HOME_X[ln];
      current_Y[ln] = HOME_Y[ln];
      current_Z[ln] = HOME_Z[ln];
    }
  }

  if(capture_offsets == true)
  {
    capture_offsets = false;
    mode = 0;
  }
}


//***********************************************************************
// One leg lift mode - driven by joy_ variables
//***********************************************************************
void one_leg_lift()
{
  if(leg1_IK_control == true)
  {
    leg1_coxa  = coxa1_servo.read();
    leg1_femur = femur1_servo.read();
    leg1_tibia = tibia1_servo.read();
    leg1_IK_control = false;
  }
  if(leg6_IK_control == true)
  {
    leg6_coxa  = coxa6_servo.read();
    leg6_femur = femur6_servo.read();
    leg6_tibia = tibia6_servo.read();
    leg6_IK_control = false;
  }

  //right stick L/R -> leg 1 coxa
  temp = map(joy_RX, 0,255, 45,-45);
  coxa1_servo.write(constrain(int(leg1_coxa+temp),45,135));

  //right stick U/D -> leg 1 femur/tibia or Z height
  temp = joy_RY;
  if(temp < 117)
  {
    temp = map(temp, 116,0, 0,24);
    femur1_servo.write(constrain(int(leg1_femur+temp),0,170));
    tibia1_servo.write(constrain(int(leg1_tibia+4*temp),0,170));
  }
  else
  {
    z_height_right = constrain(temp,140,255);
    z_height_right = map(z_height_right,140,255,1,8);
  }

  //left stick L/R -> leg 6 coxa
  temp = map(joy_LX, 0,255, 45,-45);
  coxa6_servo.write(constrain(int(leg6_coxa+temp),45,135));

  //left stick U/D -> leg 6 femur/tibia or Z height
  temp = joy_LY;
  if(temp < 117)
  {
    temp = map(temp, 116,0, 0,24);
    femur6_servo.write(constrain(int(leg6_femur+temp),0,170));
    tibia6_servo.write(constrain(int(leg6_tibia+4*temp),0,170));
  }
  else
  {
    z_height_left = constrain(temp,140,255);
    z_height_left = map(z_height_left,140,255,1,8);
  }

  if(z_height_left > z_height_right) z_height_right = z_height_left;
  if(batt_LEDs > 3) z_height_LED_color=0;
  else z_height_LED_color=1;
  LED_Bar(z_height_LED_color, z_height_right);

  if(capture_offsets == true)
  {
    step_height_multiplier = 1.0 + ((z_height_right - 1.0) / 3.0);
    capture_offsets = false;
  }
}


//***********************************************************************
// Set all servos to 90 degrees (calibration mode)
//***********************************************************************
void set_all_90()
{
  coxa1_servo.write(90+COXA_CAL[0]);
  femur1_servo.write(90+FEMUR_CAL[0]);
  tibia1_servo.write(90+TIBIA_CAL[0]);

  coxa2_servo.write(90+COXA_CAL[1]);
  femur2_servo.write(90+FEMUR_CAL[1]);
  tibia2_servo.write(90+TIBIA_CAL[1]);

  coxa3_servo.write(90+COXA_CAL[2]);
  femur3_servo.write(90+FEMUR_CAL[2]);
  tibia3_servo.write(90+TIBIA_CAL[2]);

  coxa4_servo.write(90+COXA_CAL[3]);
  femur4_servo.write(90+FEMUR_CAL[3]);
  tibia4_servo.write(90+TIBIA_CAL[3]);

  coxa5_servo.write(90+COXA_CAL[4]);
  femur5_servo.write(90+FEMUR_CAL[4]);
  tibia5_servo.write(90+TIBIA_CAL[4]);

  coxa6_servo.write(90+COXA_CAL[5]);
  femur6_servo.write(90+FEMUR_CAL[5]);
  tibia6_servo.write(90+TIBIA_CAL[5]);
}


//***********************************************************************
// Battery monitor routine
//***********************************************************************
void battery_monitor()
{
  batt_voltage_sum = batt_voltage_sum - batt_voltage_array[batt_voltage_index];
  batt_voltage_array[batt_voltage_index] = map(analogRead(BATT_VOLTAGE),0,1023,0,1497);
  batt_voltage_sum = batt_voltage_sum + batt_voltage_array[batt_voltage_index];
  batt_voltage_index = batt_voltage_index + 1;
  if(batt_voltage_index > 49) batt_voltage_index = 0;

  batt_voltage = batt_voltage_sum / 50;
  batt_LEDs = map(constrain(batt_voltage,1020,1230),1020,1230,1,8);
  if(batt_LEDs > 3) LED_Bar(1,batt_LEDs);
  else LED_Bar(0,batt_LEDs);
}


//***********************************************************************
// LED Bar Graph Routine
//***********************************************************************
void LED_Bar(int LED_color, int LED_count)
{
  if(LED_color == 0)
  {
    for(int i=0; i<LED_count; i++)
    {
      digitalWrite((RED_LED1+(4*i)),HIGH);
      digitalWrite((GREEN_LED1+(4*i)),LOW);
    }
    for(int i=LED_count; i<8; i++)
    {
      digitalWrite((RED_LED1+(4*i)),LOW);
      digitalWrite((GREEN_LED1+(4*i)),LOW);
    }
  }
  else
  {
    for(int i=0; i<LED_count; i++)
    {
      digitalWrite((GREEN_LED1+(4*i)),HIGH);
      digitalWrite((RED_LED1+(4*i)),LOW);
    }
    for(int i=LED_count; i<8; i++)
    {
      digitalWrite((GREEN_LED1+(4*i)),LOW);
      digitalWrite((RED_LED1+(4*i)),LOW);
    }
  }
}


//***********************************************************************
// Print Debug Data
//***********************************************************************
void print_debug()
{
  //display elapsed frame time (ms) and battery voltage (V)
  currentTime = millis();
  Serial.print(currentTime-previousTime);
  Serial.print(",");
  Serial.print(float(batt_voltage)/100.0);
  Serial.print("\n");
}
