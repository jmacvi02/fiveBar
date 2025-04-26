#ifndef fiveBarKin_h
#define fiveBarKin_h

#include "Arduino.h"
#include "ArduinoEigenDense.h"

/*

five bar kinematics class

*/
class fiveBarKin{
    public:
        double L0; // in meters
        double L1; 
        double L2;
        double L3; 
        double L4;

        fiveBarKin() : L0(1), L1(1), L2(1), L3(1), L4(1) {}
        fiveBarKin(double L0, double L1, double L2, double L3, double L4) : L0(L0), L1(L1), L2(L2), L3(L3), L4(L4) {}

        Eigen::Vector2d forKin(double q1, double q2);
        Eigen::Vector2d invKin(double x, double y);
        Eigen::Vector2d distalAngle(double q1, double q2);
        Eigen::Vector2d baseVelocity(double q1, double q2);
        Eigen::Vector2d distalVelocity(double q1, double q2, double q1d, double q2d);
        Eigen::Vector2d endEffVelocity(double q1, double q2, double q1d, double q2d, double q3d, double q4d);


    //float setPointPIDControl(float x, float y, float thetaRCurr, float thetaLCurr, char motor);
    private:
        double pi = 3.14159;
        double q1s[10] = {};
        double q2s[10] = {};

        /*
        float Kp = 15;
        float Ki = 0;
        float Kd = 1;
        float errorR = 0;
        float errorRdot = 0;
        float errorRPrev = 0;
        float errorL = 0;
        float errorLdot = 0;
        float errorLPrev = 0;
        */

        Eigen::Vector2d exteriorAngle(double q1, double q2);
        Eigen::Vector2d magnitude(double x, double y);
        double lawofCosLen(double a, double b, double theta);
        double lawofCosAng(double a, double b, double c); 
        void setPointError(float x, float y, float thetaRCurr, float thetaLCurr, char motor);
};

#endif