#include <quad_on_wheels/propeller_sim.hpp>
#include <math.h>

PropellerSim::PropellerSim() {
  std::cout << "Propeller Costants: \n"
  << "Ct = m * (rpm) + c\n"
  << "m : " << this.propConst.m << "\n"
  << "c : " << this.propConst.c << "\n"
  << "Cq : " << this.propConst.Cq << "\n"
  << "D : " << this.propConst.D << "\n"
  << "p : " << this.propConst.p << std:endl;

  this.d4p = pow(this.propConst.D, 4) * this.propConst.p;
  this.d5p = pow(this.propConst.D, 5) * this.propConst.p;
} 

double PropellerSim::getThurstAndTorque(double *thrust, double *torque, double rps) {
  double rpm = rps * 60;
  
  // T = CT p n2  D4
  &thrust = (this.propConst.m * rpm + this.propConst.c) * pow(rps, 2) * this.d4p;

  // Q = CQ p n2 D5
  &torque = this.propConst.Cq * pow(this.propConst.n, 2) * this.d5p;
}
