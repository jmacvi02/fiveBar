#ifndef fiveBarCon_h
#define fiveBarCon_h

#include "Arduino.h"
#include "ArduinoEigenDense.h"
#include "fiveBarKin.h"
#include "fiveBarDyn.h"


/*

five bar Control class

*/
class fiveBarCon{
    public:
    fiveBarKin& kin;
    fiveBarDyn& dyn;
    fiveBarCon(fiveBarKin& kin, fiveBarDyn& dyn) : kin(kin), dyn(dyn) {} // for in the loop use, intialize both of these classes with global class

    Eigen::Vector2d kinShaping(double q1, double q2, double tau);
    Eigen::Vector2d NLCancel(double q1, double q2);

    private:
 
};

#endif