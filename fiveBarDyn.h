#ifndef fiveBarDyn_h
#define fiveBarDyn_h

#include "Arduino.h"
#include "ArduinoEigenDense.h"

/*
five bar dynamics class
*/
class fiveBarDyn{
    public:
        float L0; // in meters
        float L1; 
        float L2;
        float L3; 
        float L4; 
        float Lc1 = L1/2; //approximation
        float Lc2 = L2/2;
        float Lc3 = L3/2; 
        float Lc4 = L4/2; 
        float m1; //estimation
        float m2;
        float m3;
        float m4;
        float I1 = m1*sq(Lc1); //approximation using point mass
        float I2 = m2*sq(Lc2);
        float I3 = m3*sq(Lc3);
        float I4 = m4*sq(Lc4);

        fiveBarDyn() : L0(1), L1(1), L2(1), L3(1), L4(1), m1(1), m2(1), m3(1), m4(1) {}
        fiveBarDyn(float L0, float L1, float L2,float L3, float L4, float m1, float m2, float m3, float m4): 
                            L0(L0), L1(L1), L2(L2), L3(L3), L4(L4), m1(m1), m2(m2), m3(m3), m4(m4) {}


        Eigen::Matrix2f massMatrix(float q1, float q2, float q3, float q4);

        Eigen::Matrix2f centCoriolMatrix(float q1, float q2, float q3, float q4, float q1d, float q2d, float q3d, float q4d);

};

#endif