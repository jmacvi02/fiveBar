#ifndef fiveBarKin_h
#define fiveBarKin_h

#include "Arduino.h"

/*

five bar kinematics class

*/
class fiveBarKin{
  public:
    float L0 = 9.3; // in centimeters
    float L1 = 14.6; 
    float L2 = 14.6;
    float L3 = 17.6; 
    float L4 = 17.2;

    fiveBarKin() : L0(1), L1(1), L2(1), L3(1), L4(1) {}
    fiveBarKin(float L0, float L1, float L2,float L3, float L4) : L0(L0), L1(L1), L2(L2), L3(L3), L4(L4) {}

    float forKin(float theta1, float theta2, char direction);
    float invKin(float x, float y, char baseJoint);

  private:
    float pi = 3.14159;

    float exteriorAngle(float theta1, float theta2, char elbowJoint);
    float magnitude(float x, float y, char motor);
    float lawofCosLen(float a, float b, float theta);
    float lawofCosAng(float a, float b, float c);
};

#endif