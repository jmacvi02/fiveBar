


#include "fiveBarCon.h"


/*
  Function Name:     kinShaping
  Purpose:           Uses calculated torque methods to shape the path of the end effector depending on a time constant
  Inputs:            the base angles and the desired time contant for the acceleration decay            
  Returns:           calculated torque
  Helper functions:  
*/

Eigen::Vector2d fiveBarCon::kinShaping(double q1, double q2, double tau) {
    Eigen::Vector4d q;
    Eigen::Vector2d qd, qdB, qdD, endEff, endEffVel;
    Eigen:: Matrix2d C, M;
    qd = kin.distalAngle(q1, q2);    //getting distal angles from base angles
    q << q1, q2, qd(0), qd(1);
    qdB = kin.baseVelocity(q1, q2);   //getting velocities from the base angles
    qdD = kin.distalVelocity(q1, q2, qdB(0), qdB(1));
    C = dyn.centCoriolMatrix(q1, q2, qd(0), qd(1), qdB(0), qdB(1), qdD(0), qdD(1));
    M = dyn.massMatrix(q1, q2, qd(0), qd(1));
    endEff = kin.forKin(q1,q2);
    endEffVel = kin.endEffVelocity(q1, q2, qdB(0), qdB(1), qdD(0), qdD(1));
    //actual first order kinematic shaping calculations
    Eigen::Matrix2d JR, JL;
    Eigen::Vector2d endEffAcc, angVelR, angVelL, goldRuleR, goldRuleL, angAccR, angAccL, baseVel, baseAcc, torque;
    endEffAcc = -1/tau * endEffVel;     // target acceleeration given a target timeconstant
    JR << -kin.L1*sin(q(0)), -kin.L3*sin(q(2)), kin.L1*cos(q(0)), kin.L3*cos(q(2));     // linear to angular vel
    JL << -kin.L2*sin(q(1)), -kin.L4*sin(q(3)), kin.L2*cos(q(1)), kin.L4*cos(q(3));
    angVelR = JR.inverse() * endEffVel;
    angVelL = JR.inverse() * endEffVel;
    // linear to angular accel
    goldRuleR << endEffAcc.row(0) + kin.L1*cos(q(0))*sq(angVelR.row(0)) + kin.L2*cos(q(2))*sq(angVelR.row(1)), 
                endEffAcc.row(1) + kin.L1*sin(q(0))*sq(angVelR.row(0)) + kin.L2*sin(q(2))*sq(angVelR.row(1));
    goldRuleL << endEffAcc.row(0) + kin.L1*cos(q(1))*sq(angVelL.row(0)) + kin.L2*cos(q(3))*sq(angVelL.row(1)), 
                endEffAcc.row(1) + kin.L1*sin(q(1))*sq(angVelL.row(0)) + kin.L2*cos(q(3))*sq(angVelL.row(1));
    angAccR = JR.inverse() * goldRuleR;    // the two jacobians, one for each arm
    angAccL = JL.inverse() * goldRuleL;
    baseVel << angVelR(0), angVelL(0);
    baseAcc << angAccR(0), angAccL(0);
    torque = M*baseAcc - C*baseVel;
  
    return torque;
  }

/*
  Function Name:     NLCancel
  Purpose:           Calculate torque1 and torque2 to send to cancel the c matrix
  inputs:            the base angles
  Returns:           A vector containing the two torques to send to the motor
  Helper functions:  A lot from the dynamics and Kin classes
*/
Eigen::Vector2d fiveBarCon::NLCancel(double q1, double q2) {
    Eigen::Vector4d q;
    Eigen::Vector2d qdB, qdD;
    //getting distal angles from base angles
    double q3 = kin.distalAngle(q1, q2)(0);
    double q4 = kin.distalAngle(q1, q2)(1);
    //getting velocities from the base angles
    qdB = kin.baseVelocity(q1, q2);
    double q1d = qdB(0);
    double q2d = qdB(1);
    qdD = kin.distalVelocity(q1, q2, q1d, q2d);
    double q3d = qdD(0);
    double q4d = qdD(1);
    //calculating c matrix 
    Eigen:: Matrix2d C, M;
    C = dyn.centCoriolMatrix(q1, q2, q3, q4, q1d, q2d, q3d, q4d);
    Eigen::Vector2d qd, tauNL;
    qd << q1d, q2d;
    tauNL = C * qd;

    return tauNL;
}
