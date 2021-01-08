#include <braccio_manipulator/hardware_interface.hpp>

namespace HardwareInterface {

////////////////////////////////////////////////////////////////////////////////
// Constructor
RobotHardwareInterface::RobotHardwareInterface(ros::NodeHandle& nh):
    nh_(nh)
{
    ROS_INFO("node_init");
    
    // Call init function
    init();
    
    // Create new controller manager in this ws
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

    // Publish loop frequency in ROS parameter server
    nh_.param("/ROBOT/hardware_interface/loop_hz", loop_hz_, 4.0);
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    
    // Create a timer to call the update function periodically
    non_realtime_loop_ = nh_.createTimer(update_freq, &RobotHardwareInterface::update, this);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RobotHardwareInterface::~RobotHardwareInterface()
{
}

////////////////////////////////////////////////////////////////////////////////
// Initialization function
void RobotHardwareInterface::init(){

    num_joints_=6;

    // Define the name of the joints
    // It has to be the same as the ones in the URDF
	joint_names_[0]="joint1";	
	joint_names_[1]="joint2";
	joint_names_[3]="joint3";
    joint_names_[4]="joint3";
    joint_names_[5]="joint3";

    // Init each joint
    for (int i = 0; i < num_joints_; ++i) {

        // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        position_joint_interface_.registerHandle(jointPositionHandle);
    
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

}

////////////////////////////////////////////////////////////////////////////////
// Initialization function
void RobotHardwareInterface::update(const ros::TimerEvent& e){

    ROS_INFO("update");
}


} /* namespace */