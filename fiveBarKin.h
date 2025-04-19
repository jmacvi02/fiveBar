#ifndef fiveBarKin_h
#define fiveBarKin_h

#include "Arduino.h"
#include "ArduinoEigenDense.h"

/*

five bar kinematics class

*/
class fiveBarKin{
    public:
        float L0; // in centimeters
        float L1; 
        float L2;
        float L3; 
        float L4;

        fiveBarKin() : L0(1), L1(1), L2(1), L3(1), L4(1) {}
        fiveBarKin(float L0, float L1, float L2,float L3, float L4) : L0(L0), L1(L1), L2(L2), L3(L3), L4(L4) {}

        float forKin(float thetaR, float thetaL, char axis);
        float invKin(float x, float y, char side);
        float distalAngle(float thetaR, float thetaL, char side);
        float baseVelocity(float thetaR, float thetaL, char side);
        float distalVelocity(float q1, float q2, float q3, float q4, float q1d, float q2d, char side);
        Eigen::Vector2f endEffVelocity(float q1, float q2, float q3, float q4, float q1d, float q2d, float q3d, float q4d);


    //float setPointPIDControl(float x, float y, float thetaRCurr, float thetaLCurr, char motor);
    private:
        float pi = 3.14159;
        float thetaRs[10] = {};
        float thetaLs[10] = {};

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

        float exteriorAngle(float thetaR, float thetaL, char elbowJoint);
        float magnitude(float x, float y, char motor);
        float lawofCosLen(float a, float b, float theta);
        float lawofCosAng(float a, float b, float c); 
        void setPointError(float x, float y, float thetaRCurr, float thetaLCurr, char motor);
};

#endif