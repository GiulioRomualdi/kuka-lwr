#ifndef LWR_CONTROLLERS__CARTESIAN_INVERSE_DYNAMICS_CONTROLLER_H
#define LWR_CONTROLLERS__CARTESIAN_INVERSE_DYNAMICS_CONTROLLER_H

#include "KinematicChainControllerBase.h"
#include <boost/scoped_ptr.hpp>

//KDL include
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdljacdot/chainjnttojacdotsolver.hpp>

// msg include
#include <geometry_msgs/WrenchStamped.h>

namespace lwr_controllers
{
  class CartesianInverseDynamicsController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
  {
  public:
    
    CartesianInverseDynamicsController();
    ~CartesianInverseDynamicsController();
    
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
    
    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);
    
    void ForceTorqueCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  
  private:

    // for jacobians x_J_y := Jacobian w.r.t reference point y expressed in basis x
    // for analytical jacobians x_JA_y := analytical Jacobian w.r.t reference point y expressed in basis x
    // for rotation matrices R_x_y := Rotation from basis y to basis x
    // tip := upper part of the 7-th link
    // ee := reference point of interest
    // T := euler kinematical matrix
    
    KDL::JntArray C_;
    KDL::JntArrayVel jnt_q_qdot_;
    KDL::JntSpaceInertiaMatrix B_;

    KDL::Jacobian base_J_tip_, base_J_ee_, ws_J_ee_, ws_J_ee_dot_;
    KDL::Rotation R_w_base_, R_w_ee_;
    KDL::Frame ee_fk_frame_;
    KDL::Wrench wrench_tip_;    
    KDL::Chain extended_chain_;

    boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> ee_jacobian_solver_, tip_jacobian_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> ee_fk_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> ee_jacobian_dot_solver_;

    Eigen::VectorXd ws_xdot_;
    Eigen::VectorXd ws_acc_cmd_, tau_fri_;
    Eigen::Matrix<double, 6,1> base_F_tip_;
    Eigen::Matrix3d ws_T_, ws_T_dot_;
    Eigen::MatrixXd ws_TA_, ws_TA_dot_;
    Eigen::MatrixXd ws_JA_ee_, ws_JA_ee_dot_;
    Eigen::MatrixXd B_inv_, BA_;
    
    double roll_, pitch_, yaw_;

    ros::Subscriber sub_;
  };

} // namespace

#endif
