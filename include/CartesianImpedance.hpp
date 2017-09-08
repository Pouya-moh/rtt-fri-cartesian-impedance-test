#ifndef CARTESIANIMPEDANCE_HPP
#define CARTESIANIMPEDANCE_HPP

#include <rtt/RTT.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <kdl/frames.hpp>

class CartesianImpedance : public RTT::TaskContext{
public:
    CartesianImpedance(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    RTT::OutputPort<rstrt::kinematics::JointAngles> red_res_out_port;
    rstrt::kinematics::JointAngles                  red_res_out_data;

    RTT::OutputPort<rstrt::dynamics::JointTorques> ext_trq_out_port;
    rstrt::dynamics::JointTorques                  ext_trq_out_data;

    RTT::OutputPort<KDL::Frame> cart_pose_out_port, cart_pose_loop_out_port;
    KDL::Frame                  cart_pose_out_data, cart_pose_loop_out_data;

    RTT::InputPort<KDL::Frame> cur_cart_pose_in_port;
    RTT::FlowStatus            cur_cart_pose_in_flow;
    KDL::Frame                 cur_cart_pose_in_data;

    void initializePorts();
};
#endif
