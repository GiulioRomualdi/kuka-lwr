#include <pluginlib/class_list_macros.h>
#include <lwr_controllers/hybrid_impedance_controller.h>

namespace lwr_controllers {

  HybridImpedanceController::HybridImpedanceController() {}

  HybridImpedanceController::~HybridImpedanceController() {}

  bool HybridImpedanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
      CartesianInverseDynamicsController::init(robot, n);

      acc_cmd_.resize(6);

      return true;
  }

  // void HybridImpedanceController::starting(const ros::Time& time)
  // {
  //     CartesianInverseDynamicsController::starting(time);
  // }

  void HybridImpedanceController::update(const ros::Time& time, const ros::Duration& period)
  {
    for(int i=0; i<6; i++)
      acc_cmd_(i) = 0;
    eval_inverse_dynamics(acc_cmd_);

    

    for(int i=0; i<6; i++)
      joint_handles_[i].setCommand(tau_fri_(i));

  }
    
} // namespace

PLUGINLIB_EXPORT_CLASS(lwr_controllers::HybridImpedanceController, controller_interface::ControllerBase)
