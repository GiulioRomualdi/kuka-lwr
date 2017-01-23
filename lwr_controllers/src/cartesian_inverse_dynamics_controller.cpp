#include <pluginlib/class_list_macros.h>
#include <lwr_controllers/cartesian_inverse_dynamics_controller.h>
#include <math.h>
#include <utils/euler_kinematical_rpy.h>
#include <Eigen/LU>

namespace lwr_controllers {

  CartesianInverseDynamicsController::CartesianInverseDynamicsController() {}

  CartesianInverseDynamicsController::~CartesianInverseDynamicsController() {}

  bool CartesianInverseDynamicsController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

    dyn_param_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    
    tau_fri_.resize(kdl_chain_.getNrOfJoints());
    acc_cmd_.resize(kdl_chain_.getNrOfJoints());
    B_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());

    R_w_base_ = KDL::Rotation::RotY(-M_PI / 2);

    // jacobian
    J_base_.resize(kdl_chain_.getNrOfJoints());
    J_wall_.resize(kdl_chain_.getNrOfJoints());
    return true;
  }

  void CartesianInverseDynamicsController::starting(const ros::Time& time)
  {
    // get joint positions
    for(size_t i=0; i<joint_handles_.size(); i++) {
      joint_msr_states_.q(i) = joint_handles_[i].getPosition();
      joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
      acc_cmd_(i) = 0;
    }

    acc_cmd_(0) = 0.5;
  }

  void CartesianInverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
    KDL::Frame fk_frame;
    KDL::Rotation R_w_ee;
    Eigen::Matrix3d T;
    Eigen::MatrixXd T_a = MatrixXd::Zero(6,6);
    Eigen::MatrixXd wJ_a;
    double roll, pitch, yaw;

    // get joint states	
    for(size_t i=0; i<joint_handles_.size(); i++) {
      joint_msr_states_.q(i) = joint_handles_[i].getPosition();
      joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }

    // solvers
    // evaluate the current B(q)
    dyn_param_solver_->JntToMass(joint_msr_states_.q, B_);
    // evaluate thecurrent C(q) * q_dot
    dyn_param_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
    // evaluate the current geometric jacobian J(q)
    jacobian_solver_->JntToJac(joint_msr_states_.q, J_base_);
    // evaluate the current homogeneous transformation from the base to the last link fk_frame
    fk_solver_->JntToCart(joint_msr_states_.q, fk_frame);
    
    /////////////////////////////////////////////////////////////////////////
    //
    // evaluate the analytical jacobian written w.r.t. the wall frame wJ_a
    // wJ_a = T_a * wJ
    //
    /////////////////////////////////////////////////////////////////////////
    
    // get the current RPY attitude representation PHI from R_w_base * fk_frame.M
    R_w_ee = R_w_base_ * fk_frame.M;
    R_w_ee.GetRPY(roll, pitch, yaw);
    
    // evaluate the transformation matrix between the geometric and analytical jacobian T_a
    // T_a = [eye(3), zeros(3);
    //        zeros(3), inv(T(PHI))]
    // where T is the Euler Kinematical Matrix
    eul_kin_RPY(pitch, yaw, T);
    T_a.block<3,3>(0,0) = Matrix<double, 3, 3>::Identity();
    T_a.block<3,3>(3,3) = T.inverse();

    // evaluate wJ
    KDL::changeBase(J_base_, R_w_base_, J_wall_);

    wJ_a = T_a * J_wall_.data;
    //
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    //
    // evaluate the kinetic pseudo-energy B_a (Siciliano p. 297)
    //
    ////////////////////////////////////////////////////////////////////////

    // evaluate the inverse of the joint space inertia matrix B
    Eigen::MatrixXd B_inv = B_.data.inverse();

    // B_a = inv(wJ_A * B_inv * J_base_')
    Eigen::MatrixXd B_a = wJ_a * B_inv * J_base_.data.transpose();
    B_a = B_a.inverse();
    //
    ////////////////////////////////////////////////////////////////////////
    
    // evaluate tau_cmd
    tau_fri_.data = C_.data + J_base_.data.transpose()  * B_a * acc_cmd_.data;

    // set tau
    for(size_t i = 0; i < joint_handles_.size(); i++) 
      joint_handles_[i].setCommand(tau_fri_(i));
  }

}// namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::CartesianInverseDynamicsController , controller_interface::ControllerBase)
