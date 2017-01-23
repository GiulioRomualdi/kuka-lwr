//#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
//#include <algorithm>
//#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
//#include <kdl_parser/kdl_parser.hpp>
//#include <urdf/model.h>
#include <kdl/chainiksolverpos_lma.hpp>
#include <angles/angles.h>
#include <lwr_controllers/cartesian_position_controller.h>

namespace lwr_controllers {

  CartesianPositionController::CartesianPositionController() {}

  CartesianPositionController::~CartesianPositionController() {}

  bool CartesianPositionController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

    dyn_param_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    // fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    // ik_vel_solver_.reset(new KDL::ChainIkSolverVel_wdls(kdl_chain_));
    //    ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, joint_limits_.min, joint_limits_.max, *fk_solver_, *ik_vel_solver_));
    ik_solver_.reset(new KDL::ChainIkSolverPos_LMA(kdl_chain_));

    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());

    ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );
  
    dotq_msr_.resize(kdl_chain_.getNrOfJoints());
    q_msr_.resize(kdl_chain_.getNrOfJoints());
    q_des_.resize(kdl_chain_.getNrOfJoints());
    tau_cmd_.resize(kdl_chain_.getNrOfJoints());
    M_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());
    M_tau_cmd_.resize(kdl_chain_.getNrOfJoints());    

    sub_posture_ = nh_.subscribe("set_qd", 1, &CartesianPositionController::set_qd, this);

    des_pose_ = KDL::Vector::Zero();
    des_attitude_ = KDL::Rotation::Identity();
    
    return true;
  }

  void CartesianPositionController::starting(const ros::Time& time)
  {
    // get joint positions
    for(size_t i=0; i<joint_handles_.size(); i++) {
      K_(i) = 30.0;
      D_(i) = 30.0;
      q_msr_(i) = joint_handles_[i].getPosition();
      dotq_msr_(i) = joint_handles_[i].getVelocity();
    }
    
    des_pose_.z(0.2);
    des_pose_.y(0.2);
    des_attitude_.DoRotY(-1.57);
    des_pose_.x(-0.7);
    x_des_ = KDL::Frame(des_attitude_, des_pose_);

    ik_solver_->CartToJnt(q_msr_, x_des_, q_des_);

    for(int i=0; i<joint_handles_.size(); i++) {
      q_des_(i) =  angles::normalize_angle(q_des_(i));
    }
  }

  void CartesianPositionController::update(const ros::Time& time, const ros::Duration& period)
  {
    // get joint positions	
    for(size_t i=0; i<joint_handles_.size(); i++) {
      q_msr_(i) = joint_handles_[i].getPosition();
      dotq_msr_(i) = joint_handles_[i].getVelocity();
    }
  
    // compute control law
    for(size_t i=0; i<joint_handles_.size(); i++) {
      tau_cmd_(i) = K_(i) * (q_des_(i) - q_msr_(i)) - D_(i)*dotq_msr_(i);
    }

    dyn_param_solver_->JntToMass(q_msr_, M_);
    dyn_param_solver_->JntToCoriolis(q_msr_, dotq_msr_, C_);

    // evaluate M * tau_cmd
    M_tau_cmd_.data = M_.data * tau_cmd_.data + C_.data;

    // set tau
    for(size_t i=0; i<joint_handles_.size(); i++) 
      joint_handles_[i].setCommand(M_tau_cmd_(i));
  }


  void CartesianPositionController::set_qd(const std_msgs::Float64MultiArray::ConstPtr &msg){
    if (msg->data.size() == 0) {
      ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
    }
    else if ((int)msg->data.size() != joint_handles_.size()) {
      ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
      return;
    }
    else
      {
	for (unsigned int j = 0; j < joint_handles_.size(); ++j)
	  q_des_(j) = msg->data[j];
      }
  }

}// namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::CartesianPositionController, controller_interface::ControllerBase)


