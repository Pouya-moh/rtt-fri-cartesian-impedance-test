#ifndef CARTESIANIMPEDANCE_HPP
#define CARTESIANIMPEDANCE_HPP

#include <rtt/RTT.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <kdl/frames.hpp>
#include <QuinticPolynomial.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#define DOF_SIZE 7

class CartesianImpedance : public RTT::TaskContext{
public:
    CartesianImpedance(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    bool setCartPose(KDL::Vector position, KDL::Rotation orientation, double duration);
    void setRedRes(rstrt::kinematics::JointAngles joint_vals);

    void setCartStiffDamp(Eigen::VectorXf  cart_stiff, Eigen::VectorXf cart_damp);

private:
    RTT::OutputPort<rstrt::dynamics::JointImpedance> cart_stiffdamp_out_port;
    rstrt::dynamics::JointImpedance                  cart_stiffdamp_out_data;

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

    // tasks:
    QuinticPolynomial<float> task;
    Eigen::Vector3f init_pos, fin_pos, intermed_pos;
    Eigen::Quaternionf init_rot, fin_rot, intermed_rot;
    double start_time, end_time;
    double time;
    bool task_set;
};
#endif
