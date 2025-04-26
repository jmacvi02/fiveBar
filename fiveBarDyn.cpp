/*
  Name:           Joel MacVicar
  Last Updated:   4/17/2025
  Purpose:        The code is this file implements a variety of functions needed to perform 
                  kinematics operations for a five bar robot. Several of these functions are
                  not written using the most efficient c++ methods to account for ME students
                  lack of experience with the language (e.g. pass by reference functions)

*/

#include "fiveBarDyn.h"

/*
  Function Name:     centCoriolCalc
  Purpose:           to initialize any hardware software links, such as the encoders to each motor
  Helper functinos:  a lot of them 
*/
/*
  Function Name:     centCoriolMatrix
  Purpose:           calculate the C matrix dynamics for a five bar robot 
  Returns:           a 2x2 matrix (the c matrix)
  inputs:            all for absolute angles, and all 4 angle velocities.
  Helper functinos:  N/A
  Notes:             
*/
Eigen::Matrix2d fiveBarDyn::centCoriolMatrix(double q1, double q2, double q3, double q4, double q1d, double q2d, double q3d, double q4d) {
    Eigen::Matrix2d as, ac, as34, ac34, a12, a34, m12, m34, i12, i34, cs34, c34, a12d, asd, acd, as34d, ac34d, a34d, a3412, a3412d; 
    Eigen::Matrix2d D, CQ, C, B;
    Eigen::Vector2d sc1, sc2;
  
    as << Lc1*sin(q1), 0, 0, Lc2*sin(q2);
    ac << Lc1*cos(q1), 0, 0, Lc2*cos(q2);
    as34 << Lc3*sin(q3), 0, 0, Lc4*sin(q4);
    ac34 << Lc3*cos(q3), 0, 0, Lc4*cos(q4);
    a12 << L1*sin(q1), -L2*sin(q2), L1*cos(q1), -L2*cos(q2);
    a34 << -L3*sin(q3), L4*sin(q4), -L3*cos(q3), L4*cos(q4);
    m12 << m1, 0, 0, m2;
    m34 << m3, 0, 0, m4;
    i12 << I1, 0, 0, I2;
    i34 << I3, 0, 0, I4;
    sc1 << sin(q1), cos(q1);
    sc2 << sin(q2), cos(q2);
    cs34 << -2*m3*L1*Lc3*sin(q1 - q3)*q1d, 0, 0, -2*m4*L2*Lc4*sin(q2-q4)*q2d; 
    c34.row(0) = L1*(a34.inverse() * sc1).transpose() * -cs34;
    c34.row(1) = L1*(a34.inverse() * sc2).transpose() * -cs34;
    a12d << L1*cos(q1)*q1d, -L2*cos(q2)*q2d, -L1*sin(q1)*q2d, L2*sin(q2)*q2d; //ASK WHY only 1 Q1D
    asd << -Lc1*cos(q1)*q1d, 0, 0, -Lc2*cos(q2)*q2d; 
    acd << -Lc1*sin(q1)*q1d, 0, 0, -Lc2*sin(q2)*q2d; 
    as34d << Lc3*sin(q3)*q3d, 0, 0, Lc4*sin(q4)*q4d; 
    ac34d << Lc3*cos(q3)*q3d, 0, 0, Lc4*cos(q4)*q4d; 
    a34d << -L3*cos(q3)*q3d, L4*cos(q4)*q4d, L3*sin(q3)*q3d, -L4*sin(q4)*q4d;
    a3412 = a34.inverse()* a12;
    a3412d = a34.inverse()* a12d;
  
    //2*A34_A12'*I34*A34\(A12d-A34d*A34_A12)...
    D = 2*a3412.transpose()*i34*(a34.inverse()*(a12d-a34d*a3412));   
    //    +2*As'*M12*Asd          +    2*(As34*A34_A12+As)'*     M34*.   ((As34d-As34*A34_A12d)* A34_A12+As34*A34_A12d+Asd)..
    D = D+ 2*as.transpose()*m12*asd + 2*(as34*a3412+as).transpose() * m34 * ((as34d-as34*a3412d) * a3412+as34*a3412d+asd);  
    //     +2*Ac'*M12*Acd           +2*(Ac34*A34_A12+Asd)*M34*((Ac34d-Ac34*A34_A12d)*A34_A12+Ac34*A34_A12d+Acd);
    D = D+ 2*ac.transpose()*m12*acd + 2*(ac34*a3412+asd) * m34 * ((ac34d-ac34*a3412d) * a3412+ac34*a3412d+acd);
  
    CQ = (cs34+c34)*a3412;
  
    B << 0.5, 0, 0, 0.5;
  
    C = D - CQ;
  
    //Serial.println(as(1,1));
    return C;
  }



Eigen::Matrix2d fiveBarDyn::massMatrix(double q1, double q2, double q3, double q4) {
    Eigen::Matrix2d m12, m34, i12, i34, as, ac, as34, ac34, a12, a34, a3412;
    Eigen::Matrix2d M;
    as << Lc1*sin(q1), 0, 0, Lc2*sin(q2);
    ac << Lc1*cos(q1), 0, 0, Lc2*cos(q2);
    as34 << Lc3*sin(q3), 0, 0, Lc4*sin(q4);
    ac34 << Lc3*cos(q3), 0, 0, Lc4*cos(q4);
    a12 << L1*sin(q1), -L2*sin(q2), L1*cos(q1), -L2*cos(q2);
    a34 << -L3*sin(q3), L4*sin(q4), -L3*cos(q3), L4*cos(q4);
    m12 << m1, 0, 0, m2;
    m34 << m3, 0, 0, m4;
    i12 << I1, 0, 0, I2;
    i34 << I3, 0, 0, I4;
    a3412 = a34.inverse()* a12;

    M = i12 + a3412.transpose()*i34*a3412 + as.transpose()*m12*as;
    M = M + (as34*a3412+as).transpose()*m34*(as34*a3412+as);
    M = M + ac.transpose()*m12*ac + (ac34*a3412+ac).transpose()*m34*(ac34*a3412+ac);
    return M;
}
