




// /*
//   Function Name:     massEQ
//   Purpose:           mass dude
//   Returns:           calculated torque
//   Helper functinos:  
// */
// Eigen::Matrix2d massEQ(double q1, double q2) {
//     Eigen::Matrix2d iJ, J, M, Meq;
//     Eigen::Vector2d qDist;
  
//     qDist = kin.distalAngle(q1, q2);
//     M = dyn.massMatrix(q1, q2, qDist(0), qDist(1));
//     J << -L1*sin(q1), -L3*sin(qDist(0)), -L2*cos(q2), -L4*cos(qDist(1));
//     iJ << -cos(qDist(0))/(L1*sin(q1-qDist(0))),  -sin(qDist(0))/(L1*sin(q1-qDist(0))), -cos(qDist(1))/(L2*sin(q2-qDist(1))), -sin(qDist(1))/(L2*sin(q2-qDist(1)));
  
//     Meq = J.transpose().inverse() * M * iJ;
  
//     return Meq;
  
  
//   }
  
//   double q1Filt[3] = {};
//   double q2Filt[3] = {};
//   double q1FiltVelPrev = 0;
//   double q2FiltVelPrev = 0;
//   double wn = 11;
//   double zeta = 0.5;
  
//   /*
//     Function Name:     baseAcc
//     Purpose:           get a filtered ang acceleration of the base joints 
//     Returns:           vector of q1dd and q2dd
//     Helper functinos:  
  
//   Eigen::Vector2d baseAcc(double q1, double q2) {
//     for(int i=2; i>0; i--) {
//       q1Filt[i] = q1Filt[i-1];
//       q2Filt[i] = q2Filt[i-1];
//     }
    
//     q1Filt[0] = (sq(wn)*q1 + (1/sq(0.0022))*2*q1Filt[1] - (1/sq(0.0022))*q1Filt[2] + ((2*zeta*wn)/0.0022)*q1Filt[1]) / ((1/sq(0.0022)) + ((2*zeta*wn)/0.0022)+ sq(wn));
//     q2Filt[0] = (sq(wn)*q2 + (1/sq(0.0022))*2*q2Filt[1] - (1/sq(0.0022))*q2Filt[2] + ((2*zeta*wn)/0.0022)*q2Filt[1]) / ((1/sq(0.0022)) + ((2*zeta*wn)/0.0022)+ sq(wn));
  
//     double q1FiltVel = (q1Filt[0] - q1Filt[1]) / 0.0022;
//     double q2FiltVel = (q1Filt[0] - q1Filt[1]) / 0.0022;
  
//     Serial.print("q1VelFilt:");
//     Serial.print(q1FiltVel);
//     Serial.print(" q2VelFilt:");
//     Serial.println(q2FiltVel);
  
//     double q1FiltAcc = (q1FiltVel - q1FiltVelPrev) / 0.0022;
//     double q2FiltAcc = (q2FiltVel - q2FiltVelPrev) / 0.0022;
  
//     Eigen::Vector2d baseAcc;
//     baseAcc << q1FiltAcc, q2FiltAcc;
  
//     q1FiltVelPrev = q1FiltVel;
//     q2FiltVelPrev = q2FiltVel;
  
//     return baseAcc;
//   }
  
//   double prevXError = 0;
//   double prevYError = 0;
//   double intXError = 0;
//   double intYError = 0;
//   double Kp = 1;
//   double Kd = 1; //0.5; //0.1;
//   /*
//     Function Name:     
//     Purpose:           
//     Returns:           
//     Helper functinos:  
  
//   Eigen::Vector2d PIDEndEff(double xCurr, double yCurr, double xTar, double yTar, double time) {
//     Eigen::Vector2d qErrors;
  
//     qErrors =  kin.invKin(xTar, yTar) - kin.invKin(xCurr, yCurr);
  
//     qErrors = qErrors * Kp;
  
//     return qErrors;
//   }
//   */