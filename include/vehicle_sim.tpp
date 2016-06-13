// vehicle_sim.tpp

// typedef Eigen::Matrix<floatType, 3, 1> Vec3;
// typedef Eigen::Quaternion<floatType> Quat;
// typedef Eigen::Matrix<floatType, 3, 3> Mat33;

template<typename floatType> 
Eigen::Matrix<floatType, 3, 1> thruster<floatType>::getThrust(){
	typedef Eigen::Matrix<floatType, 3, 1> Vec3;
	// Thruster quaternion represents rotation from body's x-axis
	Vec3 x_axis(1, 0, 0);
	Vec3 thrust = getThrustMag() * x_axis;
	thrust = orientation._transformVector(thrust);
	return thrust;
}

template<typename floatType>
vehicle<floatType>::vehicle(floatType mass, floatType differential_time_step,
                            const Eigen::Matrix<floatType, 3, 3> &moment_of_inertia_matrix){
	center_of_mass = 0;
	this->orientation.setIdentity();
	this->mass = mass;
	this->dt = differential_time_step;
	this->moment_of_inertia_matrix = moment_of_inertia_matrix;
	time = 0;
}

template<typename floatType>
vehicle<floatType>::vehicle(const Eigen::Matrix<floatType, 3, 1> &c_o_m,
								  const Eigen::Quaternion<floatType> &orientation, 
	             				  floatType mass, floatType differential_time_step,
	             				  const Eigen::Matrix<floatType, 3, 3> &moment_of_inertia_matrix){
	center_of_mass = c_o_m;
	this->orientation = orientation;
	this->mass = mass;
	this->dt = differential_time_step;
	this->moment_of_inertia_matrix = moment_of_inertia_matrix;
	time = 0;
}

template<typename floatType>
void vehicle<floatType>::addThruster(const thruster<floatType> &thruster_to_add){
	thrusters.push_back(thruster_to_add);
}

template<typename floatType>
Eigen::Matrix<floatType, 3, 1> vehicle<floatType>::getNetForce(){
	// Thruster quaternion represents rotation from body's x-axis
	Eigen::Matrix<floatType, 3, 1> net_force(0, 0, 0);
	for (thruster<floatType> thrstr : thrusters) { 
		net_force = net_force + thrstr.getThrust(); 
	}
	return net_force;		
}

template<typename floatType>
Eigen::Matrix<floatType, 3, 1> vehicle<floatType>::getNetTorque(){
	Eigen::Matrix<floatType, 3, 1> net_torque(0, 0, 0);
	for (thruster<floatType> thrstr : thrusters) {
		net_torque = net_torque + thrstr.position.cross(thrstr.getThrust());
	}
	return net_torque;
}


// TODO: finish implementing commandSub()
// template<typename floatType>
// void vehicle<floatType>::commandSub(std::vector<floatType> thruster_commands){
// 	int num_thrusters = thrusters.size();
// 	if(thruster_commands.size() != num_thrusters){
// 		std::cout << "The size of the thruster command vector (" << thruster_commands.size() 
// 		          << ") was not equal to the number of thrusters (" << num_thrusters << "\n";
// 		          return;
// 	}

// 	for(int i = 0; i < num_thrusters; i++){
// 		thrusters[i].setThrust(thruster_commands[i]);
// 	}

// 	// Update position of center of mass
// 	typedef Eigen::Matrix<floatType, 3, 1> Vec3;
// 	Vec3 acceleration = getNetForce() / mass;
// }