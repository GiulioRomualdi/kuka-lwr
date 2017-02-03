#ifndef LWR_CONTROLLERS__CARTESIAN_POSITION_CONTROLLER_H
#define LWR_CONTROLLERS__CARTESIAN_POSITION_CONTROLLER_H

#include "KinematicChainControllerBase.h"

// msg include
#include <geometry_msgs/WrenchStamped.h>

#include <boost/scoped_ptr.hpp>

/*
  tau_cmd_ = B(q) * y + C * dq + J' * F
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
    void force_torque_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  private:

    // joint position controller
    KDL::JntArray q_msr_, q_des_, dotq_msr_;
    KDL::JntArray tau_cmd_, M_tau_cmd_;
    KDL::JntArray K_, D_;
   
    // inertial matrix evaluation
    KDL::JntSpaceInertiaMatrix M_;
    KDL::JntArray C_;
    boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;

    // inverse kinematics
    boost::scoped_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    boost::scoped_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;
    KDL::Frame x_des_;
    KDL::Rotation des_attitude_;
    KDL::Vector des_pose_;

    // force compensation
    // WARNING: the end effector is compensated using force-torque sensor only
    //          instead of using B, C and G matrix
    ros::Subscriber sub_force_;
    KDL::Frame ee_fk_frame_;
    KDL::Wrench wrench_wrist_, base_wrench_wrist_;
    Eigen::Matrix<double, 6,1> base_F_wrist_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
    KDL::Jacobian J_;
  };

} // namespace

#endif
