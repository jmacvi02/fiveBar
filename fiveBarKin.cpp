/*
  Name:           Joel MacVicar
  Last Updated:   3/6/2025
  Purpose:        The code is this file implements a variety of functions needed to perform 
                  kinematics operations for a five bar robot. Several of these functions are
                  not written using the most efficient c++ methods to account for ME students
                  lack of experience with the language (e.g. pass by reference functions)

*/

#include "fiveBarKin.h"

/*
  Function Name:     forKin (GIT PUSH ORGIN MASTER)
  Purpose:           Determine the end effector position given angles of base joint angles
  Returns:           the end effector position for the given direction (either x or y)
  Inputs:            the base joint angles, 'x' for x coord of end effector, or 'y' for y coord of end effector
  Helper functinos:  exteriorAngle
  Other:             returns 10000 if an invalid direction is choosen
*/
float fiveBarKin::forKin(float theta1, float theta2, char direction) {
  float phi1 = exteriorAngle(theta1,theta2,'r');
  float phi2 = exteriorAngle(theta1,theta2,'l');
  if (tolower(direction) == 'x') {
    float xr = L0/2 + L1*cos(theta1) + L3*cos(theta1+phi1); // right arm forwardkin
    float xl = -L0/2 + L2*cos(theta2) + L4*cos(theta2-phi2); // left arm forward kin
    return (xr + xl)/2;      //returning average of forwardkin from left and right
  }
  else if (tolower(direction) == 'y') {
    float yr = L1*sin(theta1) + L3*sin(theta1+phi1);
    float yl = L2*sin(theta2) + L4*sin(theta2-phi2);
    return (yr + yl)/2;      //returning average of forwardkin from left and right
  }
  return 10000; //return if incorrect char input
}

/*
  Function Name:     invKin
  Purpose:           Determine the angles of the base joints given an end effector position
  Returns:           the angle of the specified motor 
  Inputs:            the end effect or position in cartesian coords, 'r' for the right joint angle,
                     or 'l' for the left joint angle
  Helper functinos:  magnitude 
*/
float fiveBarKin::invKin(float x, float y, char baseJoint) {
  if (tolower(baseJoint) == 'l')
    return atan2(y,x+(L0/2)) + acos( (sq(L2) + sq(magnitude(x,y,baseJoint)) - sq(L4)) / (2*L2*magnitude(x,y,baseJoint) ));
  else if (tolower(baseJoint) == 'r')
    return atan2(y,x-(L0/2)) - acos((sq(L1) + sq(magnitude(x,y,baseJoint)) - sq(L3))/(2*L1*magnitude(x,y,baseJoint)));
  else
    return 0;
}

/*
  Function Name:     exteriorAngle
  Purpose:           determine the exterior angle of the left or right joint for forward kinematics
  Returns:           the exterior angle of the desired joint
  inputs:            the base joints' angles, 'r' for the right elbow joint ext angle, or 'l' r' 
                     for the right elbow joint ext angle
  Helper functinos:  lawofCosLen nad lawofCosAng
*/
float fiveBarKin::exteriorAngle(float theta1, float theta2, char elbowJoint) {
  //need to use 3 inscribed triangles within the robot to determine the interior angle
  float intAngle = 0;
  float d1 = 0;
  float alpha = 0;
  float d2 = 0;
  float beta = 0;
  float c = 0;

  if (tolower(elbowJoint) == 'r') {
    d1 = lawofCosLen(L0,L1,pi-theta1); // dist. from left motor to right joint
    alpha = lawofCosAng(d1,L1,L0); // angle formed by d1 at right joint
    d2 = lawofCosLen(d1,L2,theta2-theta1+alpha); // dist. from left join to right joint
    beta = lawofCosAng(d1,d2,L2); // angle formed by d1 and d2 at right joint
    c = lawofCosAng(d2, L3, L4); // angle formed by d2 and L3 at right joint
  }
  else if (tolower(elbowJoint) == 'l') {
    d1 = lawofCosLen(L0,L1,theta2); // dist. from right motor to left joint
    alpha = lawofCosAng(d1,L2,L0); // angle formed by d1 at right joint
    d2 = lawofCosLen(d1,L1,theta2-theta1+alpha); // dist. from left join to right joint
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

