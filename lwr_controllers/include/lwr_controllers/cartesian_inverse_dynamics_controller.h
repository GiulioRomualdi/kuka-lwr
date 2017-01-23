#ifndef LWR_CONTROLLERS__CARTESIAN_POSITION_CONTROLLER_H
#define LWR_CONTROLLERS__CARTESIAN_POSITION_CONTROLLER_H

#include "KinematicChainControllerBase.h"
#include <boost/scoped_ptr.hpp>

//KDL include
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

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
    
    // rotation matrix from base to wall
    KDL::Rotation R_w_base_;
    
    // solvers
    boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  };

} // namespace

#endif
