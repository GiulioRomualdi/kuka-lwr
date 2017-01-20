#ifndef LWR_CONTROLLERS__CARTESIAN_POSITION_CONTROLLER_H
#define LWR_CONTROLLERS__CARTESIAN_POSITION_CONTROLLER_H

#include "KinematicChainControllerBase.h"

//#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>

/*
  tau_cmd_ = B(q) * y
  y =  - Kp * q - Kd * dq + r
  r = Kp * q_d
  q_d = inverse_kinematics(x_d)
*/

namespace lwr_controllers
{

  class CartesianPositionController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
  {
  public:

    CartesianPositionController();
    ~CartesianPositionController();
     		
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);
    void set_qd(const std_msgs::Float64MultiArray::ConstPtr &msg);

  private:
    
    ros::Subscriber sub_posture_;

    // joint position controller
    KDL::JntArray q_msr_, q_des_, dotq_msr_;
    KDL::JntArray tau_cmd_, M_tau_cmd_;
    KDL::JntArray K_, D_;
   
    // inertial matrix evaluation
    KDL::JntSpaceInertiaMatrix M_;
    boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;

    // inverse kinematics
    boost::scoped_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    boost::scoped_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;
    KDL::Frame x_des_;
    KDL::Rotation des_attitude_;
    KDL::Vector des_pose_;

    
  };

} // namespace

#endif
