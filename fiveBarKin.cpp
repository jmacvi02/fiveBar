/*
  Name:           Joel MacVicar
  Last Updated:   4/17/2025
  Purpose:        The code is this file implements a variety of functions needed to perform 
                  kinematics operations for a five bar robot. Several of these functions are
                  not written using the most efficient c++ methods to account for ME students
                  lack of experience with the language (e.g. pass by reference functions)
  TODO:           Convert multi use function (e.g returns left or right depending on paramter) 
                  into functions that return vectors of values with eigen library. 
*/

#include "fiveBarKin.h"

/*
  Function Name:     forKin
  Purpose:           Determine the end effector position given angles of base joint angles
  Returns:           the end effector position for the given direction (either x or y)
  Inputs:            the base joint angles, 'x' for x coord of end effector, or 'y' for y coord of end effector
  Helper functinos:  exteriorAngle
  Other:             returns 1000 if an invalid direction is choosen
*/
float fiveBarKin::forKin(float thetaR, float thetaL, char axis) {
  float phiR = exteriorAngle(thetaR,thetaL,'r');
  float phiL = exteriorAngle(thetaR,thetaL,'l');
  if (tolower(axis) == 'x') {
    float xr = L0/2 + L1*cos(thetaR) + L3*cos(thetaR+phiR); // right arm forwardkin
    float xl = -L0/2 + L2*cos(thetaL) + L4*cos(thetaL-phiL); // left arm forward kin
    return (xr + xl)/2;      //returning average of forwardkin from left and right
  }
  else if (tolower(axis) == 'y') {
    float yr = L1*sin(thetaR) + L3*sin(thetaR+phiR);
    float yl = L2*sin(thetaL) + L4*sin(thetaL-phiL);
    return (yr + yl)/2;      //returning average of forwardkin from left and right
  }
  return 1000; //return if incorrect char input
}

/*
  Function Name:     invKin
  Purpose:           Determine the angles of the base joints given an end effector position
  Returns:           the angle of the specified motor 
  Inputs:            the end effect or position in cartesian coords, 'r' for the right side angle,
                     or 'l' for the left side angle
  Helper functinos:  magnitude 
*/
float fiveBarKin::invKin(float x, float y, char side) {
  if (tolower(side) == 'l')
    return atan2(y,x+(L0/2)) + acos( (sq(L2) + sq(magnitude(x,y,side)) - sq(L4)) / (2*L2*magnitude(x,y,side) ));
  else if (tolower(side) == 'r')
    return atan2(y,x-(L0/2)) - acos((sq(L1) + sq(magnitude(x,y,side)) - sq(L3))/(2*L1*magnitude(x,y,side)));
  else
    return 1000;
}

/*
  Function Name:     distalAngle
  Purpose:           Determine the absolute angle of the distal joints of the robot
  Returns:           the absolute angle of the specified distanl joint 
  Inputs:            the two base joint angles, and side of the desired distal joint 
  Helper functinos:  exteriorAngle 
*/
float fiveBarKin::distalAngle(float thetaR, float thetaL, char side) {
  if (tolower(side == 'r')) {
    return thetaR + exteriorAngle(thetaR, thetaL, side);
  }
  else if (tolower(side == 'l')) {
    return thetaL - exteriorAngle(thetaR, thetaL, side);
  }
  return 1000;
}


/*
  Function Name:     setPointPIDContorl
  Purpose:           calculate the effort to send the motors to reach a set point with the end effect
  Returns:           P*error+I*errorint+D*errordot
  Inputs:            the current end effector position, current motor angles, and the motor to find the effort of
  Helper functions:  set point error
  NOTES:  NEED to CHANGE TIME FOr ThetA DOT SO IT UPDAteS iwth VArY*NG LOOPS


float fiveBarKin::setPointPIDControl(float x, float y, float thetaRCurr, float thetaLCurr, char motor) {
  
  if (tolower(motor) == 'r') {
    setPointError(x, y, thetaRCurr, thetaLCurr, motor); //updates error, errordot, and interror
    //difference between right motor angle at desired set point and current angle reading
    return Kp * errorR + Kd * errorRdot; 
  }
  else if (tolower(motor) == 'l') {
    setPointError(x, y, thetaRCurr, thetaLCurr, motor);
    //difference between left motor angle at desired set point and current angle reading
    return Kp * errorL + Kd * errorLdot; 
  }
  else
    //if incorrect input, error zero to prevent movement
    return 0.0;
}
*/

/*
  Function Name:     baseJointVelocity
  Purpose:           determine the angular velocity of the desired base joint
  Returns:           the angular velocity of the specified base joint
  inputs:            the base joints' angles, 'r' for the right elbow joint ext angle, or 'l' r' 
                     for the right elbow joint ext angle
  Helper functinos:  N/A
  Notes:             This calculation current takes the 10 point moving average of the velocities in order
                     to smooth out the signal. 
*/
float fiveBarKin::baseVelocity(float thetaR, float thetaL, char motor) {
  // for right motor angular velocity
  if(tolower(motor) == 'r') {
    for(int i = 9; i > 0; i--) { //moving back all values one spot in array
      thetaRs[i] = thetaRs[i-1];
    }
    thetaRs[0] = thetaR; // updating new value of thetaL
    return (thetaRs[0] - thetaRs[9]) / (0.0022 * 10 ); // 10 point moving average of velocity
  }
  //for left motor angular velocity
  else if(tolower(motor) == 'l') {  
    for(int i = 9; i > 0; i--) { //moving back all values one spot in array
      thetaLs[i] = thetaLs[i-1];
    }
    thetaLs[0] = thetaL; // updating new value of thetaL
    return (thetaLs[0] - thetaLs[9]) / (0.002 * 10); // 10 point moving average of velocity, dt=2ms
  }
  return 1000;
}

/*
  Function Name:     elbowVelocity
  Purpose:           determine the angular velocity of an elbow joint
  Returns:           the angular velocity of the elbow joint
  inputs:            all four absolute angles, the base joint angular velocities, 'r' for the 
                     right elbow joint ext angle, or 'l' r'. 
  Helper functinos:  N/A
*/
float fiveBarKin::distalVelocity(float q1, float q2, float q3, float q4, float q1d, float q2d, char side) {

  if(tolower(side) == 'r') {
    return (-L1*q1d*sin(q1-q4) + L2*q2d*sin(q2-q4)) / (L3*sin(q3-q4));
  }
  else if(tolower(side) == 'l') {
    return (-L1*q1d*sin(q1-q3) + L2*q2d*sin(q2-q3)) / (L4*sin(q3-q4));
  }
  else {
    return 1000;
  }
}


/*
  Function Name:     endEffVelocity
  Purpose:           determine the x and y velocities of the end effector 
  Returns:           a 2x1 matrix containing the x position (index 0) and y position (index 1)
  inputs:            all for absolute angles, and all 4 angle velocities.
  Helper functinos:  N/A
  Notes:             
*/
Eigen::Vector2f fiveBarKin::endEffVelocity(float q1, float q2, float q3, float q4, float q1d, float q2d, float q3d, float q4d) {
  Eigen::Matrix2f JR, JL;
  Eigen::Vector2f qdR, qdL, endR, endL;

  // the two jacobians, one for each arm
  JR << -L1*sin(q1), -L3*sin(q3), L1*cos(q1), L3*cos(q3);
  JL << -L2*sin(q2), -L4*sin(q4), L2*cos(q2), L4*cos(q4);

  qdR << q1d, q3d;
  qdL << q2d, q4d;

  endR = JR * qdR;
  endL = JL * qdL;

  // average of both results returned, smooths return if one calculation is noisy. 
  return 0.5*(endR+endL);
}





///////////////////////////////
////// Private functions //////
///////////////////////////////

/*
  Function Name:     exteriorAngle
  Purpose:           determine the exterior angle of the left or right joint for forward kinematics
  Returns:           the exterior angle of the desired joint
  inputs:            the base joints' angles, 'r' for the right elbow joint ext angle, or 'l' r' 
                     for the right elbow joint ext angle
  Helper functinos:  lawofCosLen nad lawofCosAng
*/
float fiveBarKin::exteriorAngle(float thetaR, float thetaL, char elbowJoint) {
  //need to use 3 inscribed triangles within the robot to determine the interior angle
  float intAngle = 0;
  float d1 = 0;
  float alpha = 0;
  float d2 = 0;
  float beta = 0;
  float c = 0;

  if (tolower(elbowJoint) == 'r') {
    d1 = lawofCosLen(L0,L1,pi-thetaR); // dist. from left motor to right joint
    alpha = lawofCosAng(d1,L1,L0); // angle formed by d1 at right joint
    d2 = lawofCosLen(d1,L2,thetaL-thetaR+alpha); // dist. from left join to right joint
    beta = lawofCosAng(d1,d2,L2); // angle formed by d1 and d2 at right joint
    c = lawofCosAng(d2, L3, L4); // angle formed by d2 and L3 at right joint
  }
  else if (tolower(elbowJoint) == 'l') {
    d1 = lawofCosLen(L0,L1,thetaL); // dist. from right motor to left joint
    alpha = lawofCosAng(d1,L2,L0); // angle formed by d1 at right joint
    d2 = lawofCosLen(d1,L1,thetaL-thetaR+alpha); // dist. from left join to right joint
    beta = lawofCosAng(d1,d2,L1); // angle formed by d1 and d2 at right joint
    c = lawofCosAng(d2, L4, L3); // angle formed by d2 and L3 at right joint
  }
  else {
    return 10000;
  }

  intAngle = pi - (alpha + beta + c);
  return intAngle;
}

/*
  Function Name:     magnitude
  Purpose:           to get encoder counts to line up with a desired zero and flip the direction of counts
  Helper functinos:  N/A
*/
float fiveBarKin::magnitude(float x, float y, char motor) {
  if (tolower(motor) == 'l')
    return sqrt(sq(x+(L0/2))+sq(y));
  else if (tolower(motor) == 'r')
    return sqrt(sq(x-(L0/2))+sq(y));
  else
    return 0;
}

/*
  Function Name:     lawOfCosLen
  Purpose:           find the length opposite of angle theta where a, b, c forms a triangle
  Helper functinos:  N/A
*/
float fiveBarKin::lawofCosLen(float a, float b, float theta) {
  return sqrt(sq(a)+sq(b)-2*a*b*cos(theta));
}

/*
  Function Name:     lawOfCosAng
  Purpose:           find the angle opposite of length c where a, b, c forms a triangle
  Helper functinos:  N/A
*/
float fiveBarKin::lawofCosAng(float a, float b, float c) {
  return acos((sq(a) + sq(b) - sq(c)) / (2*a*b));
}

/*
  Function Name:     setPointError
  Purpose:           calculate error, errordot, and interror for PIDcontrol to a set point
  Returns:           N?A
  Inputs:            the current end effector position, current motor angles, and the motor to find the effort of
  Helper functinos:  invKin

void fiveBarKin::setPointError(float x, float y, float thetaRCurr, float thetaLCurr, char motor) {
  
  if (tolower(motor) == 'r') {
    errorR = thetaRCurr - invKin(x,y,motor);
    errorRdot = ((errorR - errorRPrev) / 10 ) * 1000; // CHANGE THIS TO 
    errorRPrev = errorR;
    //difference between right motor angle at desired set point and current angle reading 
  }
  else if (tolower(motor) == 'l') {
    errorL = thetaLCurr - invKin(x,y,motor);
    errorLdot = ((errorL- errorLPrev) / 10 ) * 1000; // CHANGE THIS TO 
    errorLPrev = errorL;
    //difference between left motor angle at desired set point and current angle reading
  } 
}
  */