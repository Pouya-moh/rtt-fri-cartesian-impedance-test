#include "CartesianImpedance.hpp"
#include <rtt/Component.hpp>
#include <iostream>

CartesianImpedance::CartesianImpedance(std::string const& name) : TaskContext(name){
}

bool CartesianImpedance::configureHook(){
	return true;
}

bool CartesianImpedance::startHook(){
	return true;
}

void CartesianImpedance::updateHook(){
}

void CartesianImpedance::stopHook() {
}

void CartesianImpedance::cleanupHook() {
}

void CartesianImpedance::initializePorts(){
	cur_cart_pose_in_flow = RTT::NoData;
    cur_cart_pose_in_port.setName("cur_cart_pose_in_port");
    cur_cart_pose_in_data = KDL::Frame();
    ports()->addPort(cur_cart_pose_in_port);

    cart_pose_out_data = KDL::Frame();
    cart_pose_out_port.setName("cart_pose_out_port");
    cart_pose_out_port.setDataSample(cart_pose_out_data);
    ports()->addPort(cart_pose_out_port);

    cart_pose_loop_out_data = KDL::Frame();
    cart_pose_loop_out_port.setName("cart_pose_loop_out_port");
    cart_pose_loop_out_port.setDataSample(cart_pose_loop_out_data);
    ports()->addPort(cart_pose_loop_out_port);

    red_res_out_data = rstrt::kinematics::JointAngles(DOF_SIZE);
    red_res_out_port.setName("red_res_out_port");
    red_res_out_port.setDataSample(red_res_out_data);
    ports()->addPort(red_res_out_port);

    ext_trq_out_data = rstrt::dynamics::JointTorques(DOF_SIZE);
    ext_trq_out_port.setName("ext_trq_out_port");
    ext_trq_out_port.setDataSample(ext_trq_out_data);
    ports()->addPort(ext_trq_out_port);
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(CartesianImpedance)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(CartesianImpedance)
