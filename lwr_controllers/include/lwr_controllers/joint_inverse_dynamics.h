#ifndef LWR_CONTROLLERS__JOINT_INVERSE_DYN_CONTROLLER_H
#define LWR_CONTROLLERS__JOINT_INVERSE_DYN_CONTROLLER_H

#include "KinematicChainControllerBase.h"

//#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>

/*
  tau_cmd_ = B(q) * y
  y =  - Kp * q - Kd * q_d + r
  r = Kp * qd

*/

namespace lwr_controllers
{

  class JointInverseDynamics: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
  {
  public:

    JointInverseDynamics();
    ~JointInverseDynamics();
     		
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);
    void set_qd(const std_msgs::Float64MultiArray::ConstPtr &msg);

  private:

    ros::Subscriber sub_posture_;

    KDL::JntArray q_msr_, q_des_, dotq_msr_;
    KDL::JntArray tau_cmd_, M_tau_cmd_;
    KDL::JntArray K_, D_;
    KDL::JntSpaceInertiaMatrix M_;
    boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;
  };

} // namespace

#endif
