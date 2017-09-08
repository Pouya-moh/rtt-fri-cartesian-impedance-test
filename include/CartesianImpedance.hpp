#ifndef CARTESIANIMPEDANCE_HPP
#define CARTESIANIMPEDANCE_HPP

#include <rtt/RTT.hpp>

class CartesianImpedance : public RTT::TaskContext{
  public:
    CartesianImpedance(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
