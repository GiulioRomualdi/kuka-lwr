#include <pluginlib/class_list_macros.h>
#include <lwr_controllers/cartesian_inverse_dynamics_controller.h>
#include <math.h>
#include <utils/euler_kinematical_rpy.h>
#include <Eigen/LU>
#include <angles/angles.h>

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
    
    tau_fri_.resize(kdl_chain_.getNrOfJoints());
    acc_cmd_.resize(6);
    B_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());

    // jacobian
    J_base_.resize(kdl_chain_.getNrOfJoints());
    J_wall_.resize(kdl_chain_.getNrOfJoints());
    J_dot_.resize(kdl_chain_.getNrOfJoints());
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

    time_ = 0;

    // position of a point on the wall w.r.t the base frame
    p_wall_ = KDL::Vector(-0.77, 0, 0.25);
    // project in wall frame
    p_wall_ = R_w_base_ * p_wall_;

    // controller gains
    Kp = Eigen::Matrix<double, 6, 6>::Identity() * 20;
    Kd = Eigen::Matrix<double, 6, 6>::Identity() * 20;
  }

  void CartesianInverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
    KDL::Frame fk_frame;
    KDL::Rotation R_w_ee, R_des;
    KDL::Vector r;
    KDL::JntArrayVel jnt_q_qdot;
    Eigen::VectorXd x(6), xdot(6), x_des(6), xdot_des(6), xdotdot_des(6), err_x(6);
    Eigen::Matrix3d T, T_dot;
    Eigen::MatrixXd T_a = MatrixXd::Zero(6,6);
    Eigen::MatrixXd wJ_a, wJ_a_dot;
    Eigen::MatrixXd B_inv, B_a;
    Eigen::MatrixXd T_a_dot = MatrixXd::Zero(6,6);
    
    double roll, pitch, yaw, roll_des, pitch_des, yaw_des;

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
    R_w_ee.GetEulerZYX(yaw, pitch, roll);
    
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
    B_inv = B_.data.inverse();

    // B_a = inv(wJ_A * B_inv * J_base_')
    B_a = wJ_a * B_inv * J_base_.data.transpose();
    B_a = B_a.inverse();
    //
    ////////////////////////////////////////////////////////////////////////

    // evaluate the derivative of the state using the analytical jacobian
    xdot = wJ_a * joint_msr_states_.qdot.data;

    ////////////////////////////////////////////////////////////////////////
    //
    // evaluate part of the Coriolis compensation in operational space
    // B_a * dot(wJ_a) * qdot
    //
    ////////////////////////////////////////////////////////////////////////
    //
    
    // evaluation of dot(wJ_a) = d/dt{T_a} * J_wall_ + T_a * d/dt{J_wall_}
    //
    // where d/dt{T_a} = [d/dt{eye(3)}, d/dt{zeros(3)}; 
    //                   d/dt{zeros(3)}, d/dt{inv(T(PHI))}]
    //                 = [zeros(3), zeros(3);
    //                    zeros(3), -inv(T) * d/dt{T} * int(T)]
    //
    // and d/dt{J_wall_} = [R_w_base_, zeros(3);
    //                      zeros(3), R_w_base_] * d/dt{J_base_}
    eul_kin_RPY_dot(pitch, yaw, xdot(4), xdot(3), T_dot);
    T_a_dot.block<3,3>(3,3) = - T.inverse() * T_dot * T.inverse();

    // evaluate the derivative of the jacobian J_base_
    jnt_q_qdot.q = joint_msr_states_.q;
    jnt_q_qdot.qdot = joint_msr_states_.qdot;
    jacobian_dot_solver_->JntToJacDot(jnt_q_qdot, J_dot_);
    // and project it in the wall base
    J_dot_.changeBase(R_w_base_);

    wJ_a_dot = T_a_dot * J_wall_.data + T_a * J_dot_.data;
    //
    ////////////////////////////////////////////////////////////////////////


    // Evaluate the position vector from the wall
    r = R_w_base_ * fk_frame.p - p_wall_;
    // evaluate the state
    x << r(0), r(1), r(2), yaw, pitch, roll;

    time_ = time_ + period.toSec();
    double f = 2;
    double omega = 2 * M_PI * f;
    double rho = 0.1;
    double x_trj = rho * cos(omega * time_);
    double y_trj = rho * sin(omega * time_);
    double z_trj = 0.04;
    double dx_trj = -omega * rho * sin(omega * time_);
    double dy_trj = omega * rho * cos(omega * time_);
    double dz_trj = 0;
    double ddx_trj = -omega * omega * rho * cos(omega * time_);
    double ddy_trj = -omega * omega * rho * sin(omega * time_);
    double ddz_trj = 0;

    // define the desired state
    R_des = KDL::Rotation::RotY(0) * KDL::Rotation::RotY(M_PI);
    R_des.GetEulerZYX(yaw_des, pitch_des, roll_des);
    x_des << x_trj, y_trj, z_trj, yaw_des, pitch_des, roll_des;
    xdot_des << dx_trj, dy_trj, dz_trj, 0, 0, 0;
    xdotdot_des << ddx_trj, ddy_trj, ddz_trj, 0, 0, 0;
    
    err_x = x_des - x;
    err_x(3) = angles::normalize_angle(err_x(3));
    err_x(4) = angles::normalize_angle(err_x(4));
    err_x(5) = angles::normalize_angle(err_x(5));

    // if (time_ < 5) 
    // 	{
    // 	    time_++;
    // 	    std::cout << "current: " << yaw << " " << pitch << " " << roll << std::endl;
    // 	    std::cout << "desired: " << yaw_des << " " << pitch_des << " " << roll_des << std::endl;
    // 	    std::cout << "error: " << err_x << std::endl;
    // 	}
    
    acc_cmd_.data = Kp * err_x + Kd * (xdot_des - xdot) + xdotdot_des;

    // evaluate tau_cmd
    tau_fri_.data = C_.data + J_base_.data.transpose()* B_a * (acc_cmd_.data - wJ_a_dot * joint_msr_states_.qdot.data);

    // set tau
    for(size_t i = 0; i < joint_handles_.size(); i++) 
      joint_handles_[i].setCommand(tau_fri_(i));

    std::cout << "compensation: " << J_base_.data.transpose() * B_a * wJ_a_dot * joint_msr_states_.qdot.data << std::endl;
}
    

}// namespace

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianInverseDynamicsController , controller_interface::ControllerBase)
