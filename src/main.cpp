#include <vehicle_sim.h>
#include <iostream>

using namespace std;


int main(int argc, char ** argv){

  typedef float floatType;
  typedef Eigen::Matrix<floatType, 3, 1> Vec3;
  typedef Eigen::Quaternion<floatType> Quat;
  typedef Eigen::Matrix<floatType, 3, 3> Mat33;

  // Construct base vehicle (SI units: e.g m, kg)
  floatType mass = 50;
  Mat33 moment_of_inertia_tensor;
  moment_of_inertia_tensor << 0.4,   0,   0,
                                0, 0.4,   0,
                                0,   0, 0.4;    // Same as 1m diameter, 50 kg sphere

  vehicle<floatType> robot(mass, moment_of_inertia_tensor);
  return 0;  
} 
