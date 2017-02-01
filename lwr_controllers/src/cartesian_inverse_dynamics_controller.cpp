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

    // Extend the default chain with a fake segment in order to evaluate
    // Jacobians, derivatives of jacobians and forward kinematics with respect to given reference point
    KDL::Joint fake_joint = KDL::Joint();
    KDL::Frame frame(KDL::Rotation::Identity(), p_wrist_ee_);
    KDL::Segment fake_segment(fake_joint, frame);
    extended_chain_ = kdl_chain_;
    extended_chain_.addSegment(fake_segment);

    dyn_param_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    ee_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(extended_chain_));
    //ee_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    wrist_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    ee_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(extended_chain_));
    //ee_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    ee_jacobian_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(extended_chain_));
    //ee_jacobian_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));
    
    B_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());
    base_J_ee_.resize(kdl_chain_.getNrOfJoints());
    base_J_wrist_.resize(kdl_chain_.getNrOfJoints());
    ws_J_ee_.resize(kdl_chain_.getNrOfJoints());
    ws_J_ee_dot_.resize(kdl_chain_.getNrOfJoints());

    wrench_wrist_ = KDL::Wrench();
    base_wrench_wrist_ = KDL::Wrench();

    ws_x_ = Eigen::VectorXd(6);
    ws_xdot_ = Eigen::VectorXd(6);
    tau_fri_ = Eigen::VectorXd(kdl_chain_.getNrOfJoints());

    sub_ = n.subscribe("/lwr/ft_sensor", 1, &CartesianInverseDynamicsController::force_torque_callback, this);
    return true;
  }

  void CartesianInverseDynamicsController::starting(const ros::Time& time)
  {
    // get joint positions
    for(size_t i=0; i<joint_handles_.size(); i++) {
      joint_msr_states_.q(i) = joint_handles_[i].getPosition();
      joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }

    ws_TA_ = MatrixXd::Zero(6,6);
    ws_TA_dot_ = MatrixXd::Zero(6,6);
  }

  void CartesianInverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
    // get joint states	
    for(size_t i=0; i<joint_handles_.size(); i++) {
      joint_msr_states_.q(i) = joint_handles_[i].getPosition();
      joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }

    // solvers
    // evaluate the current B(q) (B is evaluated without taking into account anything past the wrist)
    dyn_param_solver_->JntToMass(joint_msr_states_.q, B_);
    // evaluate thecurrent C(q) * q_dot (C is evaluated without taking into account anything past the wrist)
    dyn_param_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
    // evaluate the current geometric jacobian base_J_ee(q)
    ee_jacobian_solver_->JntToJac(joint_msr_states_.q, base_J_ee_);
    wrist_jacobian_solver_->JntToJac(joint_msr_states_.q, base_J_wrist_);
    // evaluate the current homogeneous transformation from the base to the point of interest ee_fk_frame
    ee_fk_solver_->JntToCart(joint_msr_states_.q, ee_fk_frame_);

    /////////////////////////////////////////////////////////////////////////
    //
    // evaluate the analytical jacobian written w.r.t. the intermediate frame ws_JA_ee
    // ws_JA_ee = ws_TA * wJ
    //
    /////////////////////////////////////////////////////////////////////////
    
    // get the current RPY attitude representation PHI from R_ws_base * ee_fk_frame.M
    R_ws_ee_ = R_ws_base_ * ee_fk_frame_.M;
    R_ws_ee_.GetEulerZYX(yaw_, pitch_, roll_);
    
    // evaluate the transformation matrix between the geometric and analytical jacobian TA
    // ws_TA = [eye(3), zeros(3);
    //        zeros(3), inv(T(PHI))]
    // where T is the Euler Kinematical Matrix
    eul_kin_RPY(pitch_, yaw_, ws_T_);
    ws_TA_.block<3,3>(0,0) = Matrix<double, 3, 3>::Identity();
    ws_TA_.block<3,3>(3,3) = ws_T_.inverse();

    // evaluate wJ
    KDL::changeBase(base_J_ee_, R_ws_base_, ws_J_ee_);

    ws_JA_ee_ = ws_TA_ * ws_J_ee_.data;
    //
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    //
    // evaluate the kinetic pseudo-energy BA (Siciliano p. 297)
    //
    ////////////////////////////////////////////////////////////////////////

    // evaluate the inverse of the joint space inertia matrix B
    B_inv_ = B_.data.inverse();

    // BA = inv(ws_JA_ee * B_inv * base_J_wrist_')
    BA_ = ws_JA_ee_ * B_inv_ * base_J_wrist_.data.transpose();
    BA_ = BA_.inverse();
    //
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    //
    // evaluate part of the Coriolis compensation in operational space
    // BA * dot(ws_JA_ee) * qdot
    //
    ////////////////////////////////////////////////////////////////////////
    //
    
    // evaluation of dot(ws_JA_ee) = d/dt{ws_TA} * ws_J_ee_ + ws_TA * d/dt{ws_J_ee_}
    //
    // where d/dt{ws_TA} = [d/dt{eye(3)}, d/dt{zeros(3)}; 
    //                   d/dt{zeros(3)}, d/dt{inv(T(PHI))}]
    //                 = [zeros(3), zeros(3);
    //                    zeros(3), -inv(T) * d/dt{T} * int(T)]
    //
    // and d/dt{ws_J_ee_} = [R_ws_base_, zeros(3);
    //                      zeros(3), R_ws_base_] * d/dt{base_J_ee_}
    
    // evaluate the derivative of the state using the analytical jacobian
    ws_xdot_ = ws_JA_ee_ * joint_msr_states_.qdot.data;

    eul_kin_RPY_dot(pitch_, yaw_, ws_xdot_(4), ws_xdot_(3), ws_T_dot_);
    ws_TA_dot_.block<3,3>(3,3) = - ws_T_.inverse() * ws_T_dot_ * ws_T_.inverse();

    // evaluate the derivative of the jacobian base_J_ee_
    jnt_q_qdot_.q = joint_msr_states_.q;
    jnt_q_qdot_.qdot = joint_msr_states_.qdot;
    ee_jacobian_dot_solver_->JntToJacDot(jnt_q_qdot_, ws_J_ee_dot_);
    
    // and project it in the intermediate frame
    ws_J_ee_dot_.changeBase(R_ws_base_);

    ws_JA_ee_dot_ = ws_TA_dot_ * ws_J_ee_.data + ws_TA_ * ws_J_ee_dot_.data;
    //
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    //
    // evaluate force and torque compensation
    // base_F_tip = 
    //     [R_base_wrist_, zeros(3);
    //      zeros(3), R_base_wrist_] * wF
    //
    ////////////////////////////////////////////////////////////////////////
    //

    base_wrench_wrist_ = ee_fk_frame_.M * wrench_wrist_;
    tf::wrenchKDLToEigen(base_wrench_wrist_, base_F_wrist_);      
    
    //
    ////////////////////////////////////////////////////////////////////////

    // evaluate position vector between the origin of the workspace frame and the end-effector
    p_ws_ee_ = R_ws_base_ * (ee_fk_frame_.p - p_base_ws_);
    
    // evaluate the state for the inheriting controller
    ws_x_ << p_ws_ee_(0), p_ws_ee_(1), p_ws_ee_(2), yaw_, pitch_, roll_;
  }

  void CartesianInverseDynamicsController::set_command(Eigen::VectorXd& commanded_acceleration)
  {
    // evaluate tau_cmd
    tau_fri_ = C_.data +\
	base_J_wrist_.data.transpose() * BA_ *\
      (base_F_wrist_ + commanded_acceleration - ws_JA_ee_dot_ * joint_msr_states_.qdot.data);

    for(int i=0; i<6; i++)
      joint_handles_[i].setCommand(tau_fri_(i));

  }

  void CartesianInverseDynamicsController::force_torque_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    
    tf::wrenchMsgToKDL(msg->wrench, wrench_wrist_);
  }

  void CartesianInverseDynamicsController::set_p_wrist_ee(double x, double y, double z)
  {
    p_wrist_ee_ = KDL::Vector(x, y, z);
  }
  
  void CartesianInverseDynamicsController::set_p_base_ws(double x, double y, double z)
  {
    p_base_ws_ = KDL::Vector(x, y, z);
  }
  
  void CartesianInverseDynamicsController::set_ws_base_angles(double alpha, double beta, double gamma)
  {
    R_ws_base_ = KDL::Rotation::EulerZYX(alpha, beta, gamma);
  }

    
} // namespace

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianInverseDynamicsController , controller_interface::ControllerBase)
