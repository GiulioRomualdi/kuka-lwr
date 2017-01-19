//#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
//#include <algorithm>
//#include <kdl/tree.hpp>
//#include <kdl/chainfksolvervel_recursive.hpp>
//#include <kdl_parser/kdl_parser.hpp>
//#include <urdf/model.h>

#include <lwr_controllers/joint_inverse_dynamics.h>

namespace lwr_controllers {

  JointInverseDynamics::JointInverseDynamics() {}

  JointInverseDynamics::~JointInverseDynamics() {}

  bool JointInverseDynamics::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

    dyn_param_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());

    ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );
  
    dotq_msr_.resize(kdl_chain_.getNrOfJoints());
    q_msr_.resize(kdl_chain_.getNrOfJoints());
    q_des_.resize(kdl_chain_.getNrOfJoints());
    tau_cmd_.resize(kdl_chain_.getNrOfJoints());
    M_.resize(kdl_chain_.getNrOfJoints());
    M_tau_cmd_.resize(kdl_chain_.getNrOfJoints());    

    sub_posture_ = nh_.subscribe("set_qd", 1, &JointInverseDynamics::set_qd, this);
    return true;
  }

  void JointInverseDynamics::starting(const ros::Time& time)
  {
    // get joint positions
    for(size_t i=0; i<joint_handles_.size(); i++) {
      K_(i) = 10.0;
      D_(i) = 30.0;
      q_msr_(i) = joint_handles_[i].getPosition();
      dotq_msr_(i) = joint_handles_[i].getVelocity();
      q_des_(i) = 0;
    }
    q_des_(5) = 1.57; 
    q_des_(3) = 1.57;
    q_des_(2) = 1.57;
    q_des_(6) = 3.14;
  }

  void JointInverseDynamics::update(const ros::Time& time, const ros::Duration& period)
  {
    // get joint positions	
    for(size_t i=0; i<joint_handles_.size(); i++) {
      q_msr_(i) = joint_handles_[i].getPosition();
      dotq_msr_(i) = joint_handles_[i].getVelocity();
    }

    //Compute control law and set M_tau_cmd to 0
    for(size_t i=0; i<joint_handles_.size(); i++) {
      tau_cmd_(i) = K_(i) * (q_des_(i) - q_msr_(i)) - D_(i)*dotq_msr_(i);
      //M_tau_cmd_(i) = 0;
    }

    dyn_param_solver_->JntToMass(q_msr_, M_);

    // Evaluate M * tau_cmd
    M_tau_cmd_.data = M_.data * tau_cmd_.data;

    // Set () Tau ()
    for(size_t i=0; i<joint_handles_.size(); i++) 
      joint_handles_[i].setCommand(tau_cmd_(i));
  }


  void JointInverseDynamics::set_qd(const std_msgs::Float64MultiArray::ConstPtr &msg){
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

PLUGINLIB_EXPORT_CLASS( lwr_controllers::JointInverseDynamics, controller_interface::ControllerBase)


