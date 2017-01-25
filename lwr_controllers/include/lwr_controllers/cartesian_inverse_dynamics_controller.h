#ifndef LWR_CONTROLLERS__CARTESIAN_INVERSE_DYNAMICS_CONTROLLER_H
#define LWR_CONTROLLERS__CARTESIAN_INVERSE_DYNAMICS_CONTROLLER_H

#include "KinematicChainControllerBase.h"
#include <boost/scoped_ptr.hpp>

//KDL include
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdljacdot/chainjnttojacdotsolver.hpp>

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

  private:
    
    KDL::JntArray C_;
    KDL::JntArrayVel jnt_q_qdot_;
    KDL::JntSpaceInertiaMatrix B_;
    KDL::Jacobian J_base_, J_interm_, J_dot_;
    KDL::Rotation R_w_base_, R_w_ee_;
    KDL::Frame fk_frame_;
    
    boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jacobian_dot_solver_;

    Eigen::VectorXd xdot_;
    Eigen::VectorXd acc_cmd_, tau_fri_;
    Eigen::Matrix3d T_, T_dot_;
    Eigen::MatrixXd T_a_, T_a_dot_;;
    Eigen::MatrixXd wJ_a_, wJ_a_dot_;
    Eigen::MatrixXd B_inv_, B_a_;
    
    double roll_, pitch_, yaw_;
  };

} // namespace

#endif
