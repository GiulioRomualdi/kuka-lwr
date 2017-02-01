#include <pluginlib/class_list_macros.h>
#include <lwr_controllers/hybrid_impedance_controller.h>
#include <math.h>
#include <angles/angles.h>

namespace lwr_controllers {
  
  HybridImpedanceController::HybridImpedanceController()
  {
    set_p_wrist_ee(0, 0, 0.03);
    set_p_base_ws(-0.77, 0, 0.25);
    set_ws_base_angles(0, -M_PI/2, 0);
  }

  HybridImpedanceController::~HybridImpedanceController() {}

  bool HybridImpedanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
      CartesianInverseDynamicsController::init(robot, n);

      acc_cmd_ = Eigen::VectorXd(6);
      x_des_ = Eigen::VectorXd(6);
      xdot_des_ = Eigen::VectorXd(6);
      xdotdot_des_ = Eigen::VectorXd(6);
      err_x_ = Eigen::VectorXd(6);
    
      // service
      set_cmd_service_ = n.advertiseService("set_cmd", &HybridImpedanceController::set_cmd, this); 

      return true;
  }

  void HybridImpedanceController::starting(const ros::Time& time)
  {
    CartesianInverseDynamicsController::starting(time);
    //time_ = 0;
    
    // controller gains
    Kp_ = Eigen::Matrix<double, 6, 6>::Identity() * 20;
    Kd_ = Eigen::Matrix<double, 6, 6>::Identity() * 20; 

    for(int i = 0; i < 6; i++)
      acc_cmd_(i) = 0;

    fz_des_ = 0;
  }

  void HybridImpedanceController::update(const ros::Time& time, const ros::Duration& period)
  {

    // evaluate inverse dynamics
    CartesianInverseDynamicsController::update(time, period);
    
    // the base class evaluates the following quantities every update
    // ws_x_: distance and attitude between the workspace and the point of interest of the end-effector (workspace basis)
    // ws_xdot_: derivative of ws_x_ (workspace basis)
    // wrench_wrist_: forces and torques with the reference point on the wrist (world_base basis)

    // evaluate forces with the reference point on the wrist (workspace basis)
    KDL::Frame force_transformation(R_ws_base_, R_ws_ee_ * (-p_wrist_ee_)); 
    ws_F_ee_ = force_transformation * base_wrench_wrist_;

    // time_ = time_ + period.toSec();
    // double f = 0.1;
    // double omega = 2 * M_PI * f;
    // double rho = 0.1;
    // double x_trj = rho * cos(omega * time_);
    // double y_trj = rho * sin(omega * time_);
    // double z_trj = 0.02;
    // double dx_trj = -omega * rho * sin(omega * time_);
    // double dy_trj = omega * rho * cos(omega * time_);
    // double dz_trj = 0;
    // double ddx_trj = -omega * omega * rho * cos(omega * time_);
    // double ddy_trj = -omega * omega * rho * sin(omega * time_);
    // double ddz_trj = 0;
    // double yaw_des, pitch_des, roll_des;

    // define the desired state
    // R_des_ = KDL::Rotation::RotY(0) * KDL::Rotation::RotY(M_PI);
    // R_des_.GetEulerZYX(yaw_des, pitch_des, roll_des);
    // x_des_ << x_trj, y_trj, z_trj, yaw_des, pitch_des, roll_des;
    // xdot_des_ << dx_trj, dy_trj, dz_trj, 0, 0, 0;
    // xdotdot_des_ << ddx_trj, ddy_trj, ddz_trj, 0, 0, 0;
    
    err_x_ = x_des_ - ws_x_;
    err_x_(3) = angles::normalize_angle(err_x_(3));
    err_x_(4) = angles::normalize_angle(err_x_(4));
    err_x_(5) = angles::normalize_angle(err_x_(5));

    acc_cmd_ = Kp_ * err_x_ + Kd_ * (xdot_des_ - ws_xdot_) + xdotdot_des_;
    acc_cmd_(0) = acc_cmd_(0) - ws_F_ee_.force.x();
    acc_cmd_(1) = acc_cmd_(1) - ws_F_ee_.force.y();
    // acc_cmd_(3) = acc_cmd_(3) - ws_F_ee_.torque.x();
    // acc_cmd_(4) = acc_cmd_(4) - ws_F_ee_.torque.y();
    // acc_cmd_(5) = acc_cmd_(5) - ws_F_ee_.torque.z();
    acc_cmd_(2) = 0.1 * (-10 * ws_xdot_(2) + (fz_des_ - ws_F_ee_.force.z()));

    set_command(acc_cmd_);

    //std::cout << ((-1) - ws_F_ee_.force.z()) << std::endl;
    // std::cout << ws_F_ee_.force.z() << "\t"  <<  wrench_wrist_.force.z() <<  "\t" << base_wrench_wrist_.force.x() << std::endl;
  }

  bool HybridImpedanceController::set_cmd(lwr_controllers::HybridSetCmd::Request &req,\
					  lwr_controllers::HybridSetCmd::Response &res)
  {
    double yaw_des, pitch_des, roll_des;
    R_des_ = KDL::Rotation::RotY(0) * KDL::Rotation::RotY(M_PI);
    R_des_.GetEulerZYX(yaw_des, pitch_des, roll_des);
    x_des_ << req.command[0], req.command[1], 0, yaw_des, pitch_des, roll_des;
    xdot_des_ << 0, 0, 0, 0, 0, 0;
    xdotdot_des_ << 0, 0, 0, 0, 0, 0;
    fz_des_ = req.command[2];

    res.setted = true;
    return true;
  }

} // namespace

PLUGINLIB_EXPORT_CLASS(lwr_controllers::HybridImpedanceController, controller_interface::ControllerBase)
