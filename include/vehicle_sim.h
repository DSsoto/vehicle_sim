#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

template<typename floatType>
class thruster{
  typedef Eigen::Matrix<floatType, 3, 1> Vec3;
  typedef Eigen::Quaternion<floatType> Quat;
public:
  thruster(const Vec3 &pos, const Quat &orient, floatType THRUST_MAX_F, floatType THRUST_MAX_R){ 
    position = pos; orientation = orient; 
    THRUST_MAX_FORWARD = THRUST_MAX_F; 
    THRUST_MAX_REVERSE = THRUST_MAX_R;
  }
  ~thruster(){}
  void setPose(const Vec3 &pos, const Quat &orient){
    position = pos;
    orientation = orient;
  }
  void setThrustLevel(floatType th){ thrust_level = th; }
  floatType getThrustMag(){ return (thrust_level >= 0)? 
    (thrust_level * THRUST_MAX_FORWARD) : 
    (thrust_level * THRUST_MAX_REVERSE);
  }
  Vec3 getThrust();
private:
  Vec3 position;                // w.r.t body's center of mass
  Quat orientation;             // identity rotation == body x-axis
  floatType thrust_level;       // range: [-1,1]
  floatType THRUST_MAX_FORWARD; // in units of force
  floatType THRUST_MAX_REVERSE;
};


template<typename floatType>
class vehicle{
    typedef Eigen::Matrix<floatType, 3, 1> Vec3;
    typedef Eigen::Quaternion<floatType> Quat;
    typedef Eigen::Matrix<floatType, 3, 3> Mat33;
public:
  vehicle(floatType mass, const Mat33 &moment_of_inertia_matx);
  vehicle(const Vec3 &c_o_m, const Quat &orientation, 
          floatType mass, const Mat33 &moment_of_inertia_matx);
  ~vehicle(){}
  void addThruster(const thruster<floatType> &thruster_to_add);
  Vec3 getNetForce();
  Vec3 getNetTorque();
  void commandSub(std::vector<floatType> thruster_values);
private:
  Vec3 position;    // displacement of COM from intial position
  Quat orientation; // represents rotation of body frame from initial orientation
  std::vector<thruster<floatType> > thrusters;
  floatType mass;
  Mat33 moment_of_inertia_matrix;
  floatType time;   // current simulation time of vehicle
  floatType dt;     // size of simulation time step
  nextState();      // moves to next state given current state and thruster commands
};

#include <vehicle_sim.tpp>