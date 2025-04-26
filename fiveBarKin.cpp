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
Eigen::Vector2d fiveBarKin::forKin(double q1, double q2) {
  double q3 = q1 + exteriorAngle(q1,q2)(0);
  double q4 = q2 - exteriorAngle(q1,q2)(1);
  double xr = L0/2 + L1*cos(q1) + L3*cos(q3); // right arm forwardkin
  double xl = -L0/2 + L2*cos(q2) + L4*cos(q4); // left arm forward kin
  double yr = L1*sin(q1) + L3*sin(q3);
  double yl = L2*sin(q2) + L4*sin(q4);

  Eigen::Vector2d endEffPos;
  endEffPos << (xr+xl)/2, (yr+yl)/2; //returning average of forwardkin from left and right
  return endEffPos; //return if incorrect char input
}

/*
  Function Name:     invKin
  Purpose:           Determine the angles of the base joints given an end effector position
  Returns:           the angle of the specified motor 
  Inputs:            the end effect or position in cartesian coords, 'r' for the right side angle,
                     or 'l' for the left side angle
  Helper functinos:  magnitude 
*/
Eigen::Vector2d fiveBarKin::invKin(double x, double y) {
  Eigen::Vector2d qs;
  double q2 = atan2(y,x+(L0/2)) + acos( (sq(L2) + sq(magnitude(x,y)(1)) - sq(L4)) / (2*L2*magnitude(x,y)(1) ));
  double q1 = atan2(y,x-(L0/2)) - acos((sq(L1) + sq(magnitude(x,y)(1)) - sq(L3))/(2*L1*magnitude(x,y)(0)));
  qs << q1, q2;
  return qs;
}

/*
  Function Name:     distalAngle
  Purpose:           Determine the absolute angle of the distal joints of the robot
  Returns:           the absolute angle of the specified distanl joint 
  Inputs:            the two base joint angles, and side of the desired distal joint 
  Helper functinos:  exteriorAngle 
*/
Eigen::Vector2d fiveBarKin::distalAngle(double thetaR, double thetaL) {
  Eigen::Vector2d distalAngles;
  distalAngles << thetaR + exteriorAngle(thetaR, thetaL)(0), thetaL - exteriorAngle(thetaR, thetaL)(1);

  return distalAngles;
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
Eigen::Vector2d fiveBarKin::baseVelocity(double q1, double q2) {
    for(int i = 9; i > 0; i--) { //moving back all values one spot in array
      q1s[i] = q1s[i-1];
      q2s[i] = q2s[i-1];
    }
    q1s[0] = q1; // updating new value of thetaR
    q2s[0] = q2; // updating new value of thetaL
    Eigen::Vector2d qdot;
    qdot << (q1s[0] - q1s[9]) / (0.0022 * 10 ), (q2s[0] - q2s[9]) / (0.0022 * 10);
    return qdot;
  }


/*
  Function Name:     elbowVelocity
  Purpose:           determine the angular velocity of an elbow joint
  Returns:           the angular velocity of the elbow joint
  inputs:            all four absolute angles, the base joint angular velocities, 'r' for the 
                     right elbow joint ext angle, or 'l' r'. 
  Helper functinos:  N/A
*/
Eigen::Vector2d fiveBarKin::distalVelocity(double q1, double q2, double q1d, double q2d) {
  Eigen::Vector2d qDist, distVel;
  qDist = distalAngle(q1, q2);
  distVel << (-L1*q1d*sin(q1-qDist(1)) + L2*q2d*sin(q2-qDist(1))) / (L3*sin(qDist(0)-qDist(1))),
            (-L1*q1d*sin(q1-qDist(0)) + L2*q2d*sin(q2-qDist(0))) / (L4*sin(qDist(0)-qDist(1)));
    //return (-L1*q1d*sin(q1-q4) + L2*q2d*sin(q2-q4)) / (L3*sin(q3-q4));
    //return (-L1*q1d*sin(q1-q3) + L2*q2d*sin(q2-q3)) / (L4*sin(q3-q4));
  return distVel;
}


/*
  Function Name:     endEffVelocity
  Purpose:           determine the x and y velocities of the end effector 
  Returns:           a 2x1 matrix containing the x position (index 0) and y position (index 1)
  inputs:            all for absolute angles, and all 4 angle velocities.
  Helper functinos:  N/A
  Notes:             
*/
Eigen::Vector2d fiveBarKin::endEffVelocity(double q1, double q2, double q1d, double q2d, double q3d, double q4d) {
  Eigen::Matrix2d JR, JL;
  Eigen::Vector2d qDist, qdR, qdL, endR, endL;
  qDist = distalAngle(q1,q2);
  // the two jacobians, one for each arm
  JR << -L1*sin(q1), -L3*sin(qDist(0)), L1*cos(q1), L3*cos(qDist(0));
  JL << -L2*sin(q2), -L4*sin(qDist(1)), L2*cos(q2), L4*cos(qDist(1));

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
Eigen::Vector2d fiveBarKin::exteriorAngle(double q1, double q2) {
  //need to use 3 inscribed triangles within the robot to determine the interior angle

  double d1R = lawofCosLen(L0,L1,pi-q1); // dist. from left motor to right joint
  double alphaR = lawofCosAng(d1R,L1,L0); // angle formed by d1 at right joint
  double d2R = lawofCosLen(d1R,L2,q2-q1+alphaR); // dist. from left join to right joint
  double betaR = lawofCosAng(d1R,d2R,L2); // angle formed by d1 and d2 at right joint
  double cR = lawofCosAng(d2R, L3, L4); // angle formed by d2 and L3 at right joint
  
  double d1L = lawofCosLen(L0,L1,q2); // dist. from right motor to left joint
  double alphaL = lawofCosAng(d1L,L2,L0); // angle formed by d1 at right joint
  double d2L = lawofCosLen(d1L,L1,q2-q1+alphaL); // dist. from left join to right joint
  double betaL = lawofCosAng(d1L,d2L,L1); // angle formed by d1 and d2 at right joint
  double cL = lawofCosAng(d2L, L4, L3); // angle formed by d2 and L3 at right joint

  Eigen::Vector2d extAngles;
  extAngles << pi - (alphaR + betaR + cR), pi - (alphaL + betaL + cL);
  return extAngles;
}

/*
  Function Name:     magnitude
  Purpose:           
  Helper functinos:  N/A
*/
Eigen::Vector2d fiveBarKin::magnitude(double x, double y) {
  Eigen::Vector2d mags;
  double magL = sqrt(sq(x+(L0/2))+sq(y));
  double magR =  sqrt(sq(x-(L0/2))+sq(y));
  mags << magR, magL;

  return mags;
}

/*
  Function Name:     lawOfCosLen
  Purpose:           find the length opposite of angle theta where a, b, c forms a triangle
  Helper functinos:  N/A
*/
double fiveBarKin::lawofCosLen(double a, double b, double theta) {
  return sqrt(sq(a)+sq(b)-2*a*b*cos(theta));
}

/*
  Function Name:     lawOfCosAng
  Purpose:           find the angle opposite of length c where a, b, c forms a triangle
  Helper functinos:  N/A
*/
double fiveBarKin::lawofCosAng(double a, double b, double c) {
  return acos((sq(a) + sq(b) - sq(c)) / (2*a*b));
}

/*
  Function Name:     setPointError
  Purpose:           calculate error, errordot, and interror for PIDcontrol to a set point
  Returns:           N?A
  Inputs:            the current end effector position, current motor angles, and the motor to find the effort of
  Helper functinos:  invKin

void fiveBarKin::setPointError(double x, double y, double thetaRCurr, double thetaLCurr, char motor) {
  
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