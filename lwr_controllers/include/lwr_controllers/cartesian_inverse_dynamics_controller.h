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
    friend class HybridImpedanceController;
  public:
    
    CartesianInverseDynamicsController();
    ~CartesianInverseDynamicsController();
    
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
    
    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

    void set_command(Eigen::VectorXd& commanded_acceleration);
    
    void force_torque_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  
    void set_p_wrist_ee(double x, double y, double z);

    void set_p_base_ws(double x, double y, double z);

    void set_ws_base_angles(double alpha, double beta, double gamma);

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
    KDL::Jacobian base_J_wrist_, base_J_ee_, ws_J_ee_, ws_J_ee_dot_;
    KDL::Rotation R_ws_base_, R_ws_ee_;
    KDL::Frame ee_fk_frame_;
    KDL::Wrench wrench_wrist_, base_wrench_wrist_;    
    KDL::Chain extended_chain_;
    KDL::Vector p_wrist_ee_; // distance between the origin of the 7-th link and the point of interest (see urdf)
    KDL::Vector p_base_ws_;
    KDL::Vector p_ws_ee_;

    boost::scoped_ptr<KDL::ChainDynParam> dyn_param_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> ee_jacobian_solver_, wrist_jacobian_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> ee_fk_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> ee_jacobian_dot_solver_;

    Eigen::VectorXd ws_x_, ws_xdot_;
    Eigen::VectorXd tau_fri_;
    Eigen::Matrix<double, 6,1> base_F_wrist_;
    Eigen::Matrix3d ws_T_, ws_T_dot_;
    Eigen::MatrixXd ws_TA_, ws_TA_dot_;
    Eigen::MatrixXd ws_JA_ee_, ws_JA_ee_dot_;
    Eigen::MatrixXd B_inv_, BA_;
    
    double roll_, pitch_, yaw_;

    ros::Subscriber sub_;
  };

} // namespace

#endif
