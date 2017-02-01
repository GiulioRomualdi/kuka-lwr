#ifndef LWR_CONTROLLERS__HYBRID_IMPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__HYBRID_IMPEDANCE_CONTROLLER_H

#include "cartesian_inverse_dynamics_controller.h"

// service
#include "lwr_controllers/HybridSetCmd.h"

namespace lwr_controllers
{
  class HybridImpedanceController: public CartesianInverseDynamicsController
  {
  public:
    
    HybridImpedanceController();
    ~HybridImpedanceController();
    
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

    bool set_cmd(lwr_controllers::HybridSetCmd::Request &req,\
		 lwr_controllers::HybridSetCmd::Response &res);
    
    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

  private:

    // service
    ros::ServiceServer set_cmd_service_;

    KDL::Wrench ws_F_ee_;
    KDL::Rotation R_des_;

    Eigen::VectorXd acc_cmd_;
    Eigen::VectorXd x_des_, xdot_des_, xdotdot_des_;
    Eigen::VectorXd err_x_;
    Eigen::MatrixXd Kp_, Kd_;    

    //double time_;
    double fz_des_;
  };
} // namespace

#endif
