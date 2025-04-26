#ifndef fiveBarDyn_h
#define fiveBarDyn_h

#include "Arduino.h"
#include "ArduinoEigenDense.h"

/*
five bar dynamics class
*/
class fiveBarDyn{
    public:
        double L0; // in meters
        double L1; 
        double L2;
        double L3; 
        double L4; 
        double Lc1 = L1/2; //approximation
        double Lc2 = L2/2;
        double Lc3 = L3/2; 
        double Lc4 = L4/2; 
        double m1; //estimation
        double m2;
        double m3;
        double m4;
        double I1 = m1*sq(Lc1); //approximation using point mass
        double I2 = m2*sq(Lc2);
        double I3 = m3*sq(Lc3);
        double I4 = m4*sq(Lc4);

        fiveBarDyn() : L0(1), L1(1), L2(1), L3(1), L4(1), m1(1), m2(1), m3(1), m4(1) {}
        fiveBarDyn(double L0, double L1, double L2,double L3, double L4, double m1, double m2, double m3, double m4): 
                            L0(L0), L1(L1), L2(L2), L3(L3), L4(L4), m1(m1), m2(m2), m3(m3), m4(m4) {}


        Eigen::Matrix2d massMatrix(double q1, double q2, double q3, double q4);

        Eigen::Matrix2d centCoriolMatrix(double q1, double q2, double q3, double q4, double q1d, double q2d, double q3d, double q4d);

};

#endif