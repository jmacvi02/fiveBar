
//
// JOElS LOOP UNIT TESTING CODE 
// each chunk can be run in the loop() in arduino ide for calibration, sanity checks, etc
//

/* testing inverse kinematics
  Serial.print(" IK of:");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print("thetaL = ");
  Serial.print(inverseKinematics(x,y,'L'));
  Serial.print(", ");
  Serial.print("thetaR = ");
  Serial.println(inverseKinematics(x,y,'R'));


 //testing forward kinematics
  float theta1 = countsToRadians(readEncoderR());
  float theta2 = countsToRadians(readEncoderL());
  Serial.print(" X:");
  Serial.print(forwardKinematics(theta1,theta2,'x'));
  Serial.print("  Y:");
  Serial.println(forwardKinematics(theta1,theta2,'y'));

 //testing inverse kinematics
   float x = 0;
  float y = 15;
  Serial.print(" IK of:");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print("thetaL = ");
  Serial.print(inverseKinematics(x,y,'L'));
  Serial.print(", ");
  Serial.print("thetaR = ");
  Serial.println(inverseKinematics(x,y,'R'));

  // testing IK FK together
  Serial.print("IK says @ point: ");
  Serial.print("(");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(") ");
  Serial.println("the angles should be");
  Serial.print("theta1 = ");
  Serial.print(inverseKinematics(x,y,'R'));
  Serial.print(", ");
  Serial.print("theta2 = ");
  Serial.println(inverseKinematics(x,y,'L'));
  Serial.print("FK says the end effector is @ point: ");
  Serial.print("(");
  Serial.print(forwardKinematics(inverseKinematics(x,y,'R'), inverseKinematics(x,y,'L'), 'x'));
  Serial.print(", ");
  Serial.print(forwardKinematics(inverseKinematics(x,y,'R'), inverseKinematics(x,y,'L'), 'y'));
  Serial.println(") ");
*/

 /*testing encoder calibration
  Serial.print("encL - encR:");
  Serial.println(countsToRadians(readEncoderL()) - countsToRadians(readEncoderR()));

  if (workSpace(countsToRadians(readEncoderR()), countsToRadians(readEncoderL()))) {
    Serial.println("IN the workspace");
  }
  else {
    Serial.println("NOT IN the workspace");
  }
  */


/* end point PID control
  motorL.loopFOC();
  motorR.loopFOC();
  fiveBarKin fiveBar(L0, L1, L2, L3, L4);
  float xTarget = 0.0;
  float yTarget = 20.0;
  Serial.print("errorL:");
  Serial.print(fiveBar.setPointPIDControl(xTarget, yTarget, countsToRadians(readEncoderR()), countsToRadians(readEncoderL()), 'L'));
  //Serial.println(countsToRadians(readEncoderL()));
  //Serial.println(countsToRadians(readEncoderR()));
  Serial.print(" errorR:");
  Serial.println(fiveBar.setPointPIDControl(xTarget, yTarget, countsToRadians(readEncoderR()), countsToRadians(readEncoderL()), 'R')); 

  if (workSpace(countsToRadians(readEncoderR()), countsToRadians(readEncoderL()))) {
    Serial.println("moving");
    motorL.move(fiveBar.setPointPIDControl(xTarget, yTarget, countsToRadians(readEncoderR()), countsToRadians(readEncoderL()), 'L'));
    motorR.move(fiveBar.setPointPIDControl(xTarget, yTarget, countsToRadians(readEncoderR()), countsToRadians(readEncoderL()), 'R'));
  }
  */
/*
  Additional damping in x direction using end effector velocity 

  float currTime = millis();
  float xdot = (fiveBar.forKin(thetaR, thetaL, 'x') - xPrev)/ (currTime-prevTime);
  float ydot = (fiveBar.forKin(thetaR, thetaL, 'y') - yPrev)/ (currTime-prevTime);

  Serial.print("xVel:");
  Serial.print(xdot);
  Serial.print(" yVel:");
  Serial.println(ydot);

  prevTime = currTime;
  xPrev = fiveBar.forKin(thetaR, thetaL, 'x');
  yPrev = fiveBar.forKin(thetaR, thetaL, 'y');

  motorR.move(xdot * -100);
  motorL.move(xdot * -100);

*/