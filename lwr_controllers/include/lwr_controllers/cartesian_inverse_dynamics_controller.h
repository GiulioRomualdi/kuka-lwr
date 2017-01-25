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
    
    KDL::JntArray acc_cmd_, tau_fri_;
    KDL::JntSpaceInertiaMatrix B_;
    KDL::JntArray C_;

    // jacobians

    // geometric jacobian written w.r.t the base frame
    KDL::Jacobian J_base_;
    // geometric jacobian written w.r.t the wall frame
    KDL::Jacobian J_wall_;
    // derivative of the geometric jacobian
    KDL::Jacobian J_dot_;
    
    // rotation matrix from base to wall
    KDL::Rotation R_w_base_;
    
    // solvers
    boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jacobian_dot_solver_;

    //
    KDL::Vector p_wall_;

    // controller gains
    Eigen::MatrixXd Kp, Kd;


    double time_;

  };

} // namespace

#endif
