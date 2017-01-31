#ifndef LWR_CONTROLLERS__HYBRID_IMPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__HYBRID_IMPEDANCE_CONTROLLER_H

#include "cartesian_inverse_dynamics_controller.h"

namespace lwr_controllers
{
  class HybridImpedanceController: public CartesianInverseDynamicsController
  {
  public:
    
    HybridImpedanceController();
    ~HybridImpedanceController();
    
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
    
    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

  private:
    
    KDL::Wrench ws_F_ee_;
    KDL::Rotation R_des_;

    Eigen::VectorXd acc_cmd_;
    Eigen::VectorXd x_des_, xdot_des_, xdotdot_des_;
    Eigen::VectorXd err_x_;
    Eigen::MatrixXd Kp_, Kd_;    

    double time_;

  };
} // namespace

#endif
