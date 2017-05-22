#ifndef QUAD_ON_WHEELS_PROPELLER_SIM_H
#define QUAD_ON_WHEELS_PROPELLER_SIM_H

class PropellerSim {
  public : 
    PropellerSim();
    void getThrustAndTorque(double *, double *, double);

  private : 
    struct {
      // Ct = m*rmp + c 
      double m = 0.00000009604;
      double c = 0.162;
      double Cq = 0.02027;
      double D = 0.1016;
      double p = 1.225;
    } propConst;

    double d4p;
    double d5p;
};

#endif
