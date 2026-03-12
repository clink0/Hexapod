#pragma once

#include <math.h>

//***********************************************************************
// kinematics.h - 3-DOF inverse kinematics for a single leg.
//
//  leg_IK(leg_number, X, Y, Z)
//    Given a target toe position in the leg's local frame (mm),
//    computes coxa / femur / tibia servo angles and writes them.
//    Silently skips legs whose target is out of reach.
//
//  Leg numbering:  0 = front-right,  1 = mid-right,  2 = rear-right
//                  3 = rear-left,    4 = mid-left,    5 = front-left
//***********************************************************************

void leg_IK(int leg_number, float X, float Y, float Z)
{
  // horizontal distance from coxa pivot to target, minus coxa length
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
  // straight-line femur-to-toe distance
  L3 = sqrt(sq(L0) + sq(Z));

  // only solve if the target is physically reachable
  if((L3 < (TIBIA_LENGTH + FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH - FEMUR_LENGTH)))
  {
    // tibia angle (law of cosines)
    phi_tibia   = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3))
                       / (2 * FEMUR_LENGTH * TIBIA_LENGTH));
    theta_tibia = phi_tibia * RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia, 0.0, 180.0);

    // femur angle (law of cosines + elevation angle)
    gamma_femur = atan2(Z, L0);
    phi_femur   = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH))
                       / (2 * FEMUR_LENGTH * L3));
    theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur, 0.0, 180.0);

    // coxa angle (horizontal plane)
    theta_coxa = atan2(X, Y) * RAD_TO_DEG + COXA_CAL[leg_number];

    // write to the correct servo set; coxa offset accounts for mounting angle
    switch(leg_number)
    {
      case 0:   // front-right
        if(leg1_IK_control == true)
        {
          theta_coxa = constrain(theta_coxa + 45.0, 0.0, 180.0);
          coxa1_servo.write(int(theta_coxa));
          femur1_servo.write(int(theta_femur));
          tibia1_servo.write(int(theta_tibia));
        }
        break;
      case 1:   // mid-right
        theta_coxa = constrain(theta_coxa + 90.0, 0.0, 180.0);
        coxa2_servo.write(int(theta_coxa));
        femur2_servo.write(int(theta_femur));
        tibia2_servo.write(int(theta_tibia));
        break;
      case 2:   // rear-right
        theta_coxa = constrain(theta_coxa + 135.0, 0.0, 180.0);
        coxa3_servo.write(int(theta_coxa));
        femur3_servo.write(int(theta_femur));
        tibia3_servo.write(int(theta_tibia));
        break;
      case 3:   // rear-left
        theta_coxa = (theta_coxa < 0) ? theta_coxa + 225.0 : theta_coxa - 135.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa4_servo.write(int(theta_coxa));
        femur4_servo.write(int(theta_femur));
        tibia4_servo.write(int(theta_tibia));
        break;
      case 4:   // mid-left
        theta_coxa = (theta_coxa < 0) ? theta_coxa + 270.0 : theta_coxa - 90.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa5_servo.write(int(theta_coxa));
        femur5_servo.write(int(theta_femur));
        tibia5_servo.write(int(theta_tibia));
        break;
      case 5:   // front-left
        if(leg6_IK_control == true)
        {
          theta_coxa = (theta_coxa < 0) ? theta_coxa + 315.0 : theta_coxa - 45.0;
          theta_coxa = constrain(theta_coxa, 0.0, 180.0);
          coxa6_servo.write(int(theta_coxa));
          femur6_servo.write(int(theta_femur));
          tibia6_servo.write(int(theta_tibia));
        }
        break;
    }
  }
}
