#include <pluginlib/class_list_macros.h>
#include <lwr_controllers/cartesian_inverse_dynamics_controller.h>
#include <math.h>
#include <utils/euler_kinematical_rpy.h>
#include <Eigen/LU>
#include <angles/angles.h>

#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>

namespace lwr_controllers {

  CartesianInverseDynamicsController::CartesianInverseDynamicsController() {}

  CartesianInverseDynamicsController::~CartesianInverseDynamicsController() {}

  bool CartesianInverseDynamicsController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

    dyn_param_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    jacobian_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));
    jacobian_dot_solver_->setHybridRepresentation();
    
    B_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());
    J_base_.resize(kdl_chain_.getNrOfJoints());
    J_w_.resize(kdl_chain_.getNrOfJoints());
    J_dot_.resize(kdl_chain_.getNrOfJoints());

    wrench_wrist_ = KDL::Wrench();

    xdot_ = Eigen::VectorXd(6);
    acc_cmd_ = Eigen::VectorXd(6);
    tau_fri_ = Eigen::VectorXd(kdl_chain_.getNrOfJoints());

    sub_ = n.subscribe("/lwr/ft_sensor", 1, &CartesianInverseDynamicsController::ForceTorqueCallback, this);
    return true;
  }

  void CartesianInverseDynamicsController::starting(const ros::Time& time)
  {
    // get joint positions
    for(size_t i=0; i<joint_handles_.size(); i++) {
      joint_msr_states_.q(i) = joint_handles_[i].getPosition();
      joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }

    for(size_t i=0; i<6; i++)
      acc_cmd_(i) = 0;

    R_w_base_ = KDL::Rotation::RotY(-M_PI / 2);
    T_a_ = MatrixXd::Zero(6,6);
    T_a_dot_ = MatrixXd::Zero(6,6);
  }

  void CartesianInverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
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
    fk_solver_->JntToCart(joint_msr_states_.q, fk_frame_);
    
    /////////////////////////////////////////////////////////////////////////
    //
    // evaluate the analytical jacobian written w.r.t. the intermediate frame wJ_a
    // wJ_a = T_a * wJ
    //
    /////////////////////////////////////////////////////////////////////////
    
    // get the current RPY attitude representation PHI from R_w_base * fk_frame.M
    R_w_ee_ = R_w_base_ * fk_frame_.M;
    R_w_ee_.GetEulerZYX(yaw_, pitch_, roll_);
    
    // evaluate the transformation matrix between the geometric and analytical jacobian T_a
    // T_a = [eye(3), zeros(3);
    //        zeros(3), inv(T(PHI))]
    // where T is the Euler Kinematical Matrix
    eul_kin_RPY(pitch_, yaw_, T_);
    T_a_.block<3,3>(0,0) = Matrix<double, 3, 3>::Identity();
    T_a_.block<3,3>(3,3) = T_.inverse();

    // evaluate wJ
    KDL::changeBase(J_base_, R_w_base_, J_w_);

    wJ_a_ = T_a_ * J_w_.data;
    //
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    //
    // evaluate the kinetic pseudo-energy B_a (Siciliano p. 297)
    //
    ////////////////////////////////////////////////////////////////////////

    // evaluate the inverse of the joint space inertia matrix B
    B_inv_ = B_.data.inverse();

    // B_a = inv(wJ_a * B_inv * J_base_')
    B_a_ = wJ_a_ * B_inv_ * J_base_.data.transpose();
    B_a_ = B_a_.inverse();
    //
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    //
    // evaluate part of the Coriolis compensation in operational space
    // B_a * dot(wJ_a) * qdot
    //
    ////////////////////////////////////////////////////////////////////////
    //
    
    // evaluation of dot(wJ_a) = d/dt{T_a} * J_w_ + T_a * d/dt{J_w_}
    //
    // where d/dt{T_a} = [d/dt{eye(3)}, d/dt{zeros(3)}; 
    //                   d/dt{zeros(3)}, d/dt{inv(T(PHI))}]
    //                 = [zeros(3), zeros(3);
    //                    zeros(3), -inv(T) * d/dt{T} * int(T)]
    //
    // and d/dt{J_w_} = [R_w_base_, zeros(3);
    //                      zeros(3), R_w_base_] * d/dt{J_base_}
    
    // evaluate the derivative of the state using the analytical jacobian
    xdot_ = wJ_a_ * joint_msr_states_.qdot.data;

    eul_kin_RPY_dot(pitch_, yaw_, xdot_(4), xdot_(3), T_dot_);
    T_a_dot_.block<3,3>(3,3) = - T_.inverse() * T_dot_ * T_.inverse();

    // evaluate the derivative of the jacobian J_base_
    jnt_q_qdot_.q = joint_msr_states_.q;
    jnt_q_qdot_.qdot = joint_msr_states_.qdot;
    jacobian_dot_solver_->JntToJacDot(jnt_q_qdot_, J_dot_);
    
    // and project it in the intermediate frame
    J_dot_.changeBase(R_w_base_);

    wJ_a_dot_ = T_a_dot_ * J_w_.data + T_a_ * J_dot_.data;
    //
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    //
    // evaluate force and torque compensation
    // bF = 
    //     [R_base_wrist, zeros(3);
    //      zeros(3), R_base_wrist_] * wF
    //
    ////////////////////////////////////////////////////////////////////////
    //

    wrench_wrist_ = fk_frame_.M * wrench_wrist_;
    tf::wrenchKDLToEigen(wrench_wrist_, bF_);      
    
    //
    ////////////////////////////////////////////////////////////////////////

    // evaluate tau_cmd
    tau_fri_ = C_.data + J_base_.data.transpose()* B_a_ * (bF_ + acc_cmd_ - wJ_a_dot_ * joint_msr_states_.qdot.data);

    // set tau
    for(size_t i = 0; i < joint_handles_.size(); i++) 
      joint_handles_[i].setCommand(tau_fri_(i));
  }

  void CartesianInverseDynamicsController::ForceTorqueCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    tf::wrenchMsgToKDL(msg->wrench, wrench_wrist_);
  }
    
} // namespace

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianInverseDynamicsController , controller_interface::ControllerBase)
