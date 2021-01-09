#include <braccio_manipulator/hardware_interface.hpp>

namespace HardwareInterface {

////////////////////////////////////////////////////////////////////////////////
// Callback function
void RobotHardwareInterface::Joint_state_CB(const sensor_msgs::JointState joint_state){

    jt_states[0] = joint_state.position[0];
    jt_states[1] = joint_state.position[1];
    jt_states[2] = joint_state.position[2];
    jt_states[3] = joint_state.position[3];
    jt_states[4] = joint_state.position[4];
    jt_states[5] = joint_state.position[5];

}

////////////////////////////////////////////////////////////////////////////////
// Constructor
RobotHardwareInterface::RobotHardwareInterface(ros::NodeHandle& nh):
    nh_(nh)
{
    ROS_INFO("node_init");

    // Init variables
    M_5_PI_12 = 5*M_PI/12;
    high_limit_joint_5 = angles::from_degrees(35);
    low_limit_joint_6 = angles::from_degrees(70);
    
    // Call init function
    init();
    
    // Create new controller manager in this ws
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

    // Create publisher and subscriber
    cmd_pub = nh_.advertise<std_msgs::UInt16MultiArray>("/servo_cmd",10);
    jt_st_sub = nh_.subscribe("/braccio_states", 10, &RobotHardwareInterface::Joint_state_CB, this);

    // Publish loop frequency in ROS parameter server
    nh_.param("/braccio_manipulator/hardware_interface/loop_hz", loop_hz_, 15.0);
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

    // Load URDF model
    urdf::Model urdf_model;
    if (!urdf_model.initParam("robot_description")) return;

    // Declare number of joints
    num_joints_=6;

    // Define the name of the joints
    // It has to be the same as the ones in the URDF
	joint_names_[0]="base_joint";	
	joint_names_[1]="shoulder_joint";
	joint_names_[2]="elbow_joint";
    joint_names_[3]="wrist_pitch_joint";
    joint_names_[4]="wrist_roll_joint";
    joint_names_[5]="gripper_joint";

    jt_states [6] = { };

    // Init each joint
    for (int i = 0; i < num_joints_; ++i) {

        // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);        

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);

        //--- Create limits
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::SoftJointLimits soft_limits;

        //--- Populate (soft) joint limits from URDF
        //--- Limits specified in URDF overwrite existing values in 'limits' and 'soft_limits'
        //--- Limits not specified in URDF preserve their existing values
        urdf::JointConstSharedPtr urdf_joint = urdf_model.getJoint(joint_names_[i]);
        bool urdf_limits_ok = getJointLimits(urdf_joint, limits);
        bool urdf_soft_limits_ok = getSoftJointLimits(urdf_joint, soft_limits);

        //--- Populate (soft) joint limits from the ros parameter server
        //--- Limits specified in the parameter server overwrite existing values in 'limits' and 'soft_limits'
        //--- Limits not specified in the parameter server preserve their existing values
        //const bool rosparam_limits_ok = getJointLimits(joint_names_[i], nh_, limits);

        //--- Create handle in joint limits interface
        joint_limits_interface::PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, soft_limits);  // Soft limits spec
        
        //--- Register handles
        joint_state_interface_.registerHandle(jointStateHandle);
        positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
        position_joint_interface_.registerHandle(jointPositionHandle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&positionJointSoftLimitsInterface);
}

////////////////////////////////////////////////////////////////////////////////
// Update function
void RobotHardwareInterface::update(const ros::TimerEvent& e){
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

////////////////////////////////////////////////////////////////////////////////
// Read function
void RobotHardwareInterface::read(){

    joint_position_[0] = (jt_states[0]-0.0)*(M_PI_2-(-M_PI_2))/(180.0-0.0) + (-M_PI_2);
    joint_position_[1] = (jt_states[1]-15.0)*(M_5_PI_12-(-M_5_PI_12))/(165.0-15.0) + (-M_5_PI_12);
    joint_position_[2] = (jt_states[2]-0.0)*(M_PI_2-(-M_PI_2))/(180.0-0.0) + (-M_PI_2);
    joint_position_[3] = (jt_states[3]-0.0)*(M_PI_2-(-M_PI_2))/(180.0-0.0) + (-M_PI_2);
    joint_position_[4] = (jt_states[4]-0.0)*(high_limit_joint_5-(-M_PI_2))/(125.0-0.0) + (-M_PI_2);
    joint_position_[5] = (jt_states[5]-0.0)*(M_PI_2-(low_limit_joint_6))/(90.0-0.0) + (low_limit_joint_6);

}

////////////////////////////////////////////////////////////////////////////////
// Write function
void RobotHardwareInterface::write(ros::Duration elapsed_time){

    //positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
    
    jt_cmds.data.clear();
    jt_cmds.data.push_back((joint_position_command_[0]-(-M_PI_2))*(180.0-0.0)/(M_PI_2-(-M_PI_2)) + (0.0));
    jt_cmds.data.push_back((joint_position_command_[1]-(-M_5_PI_12))*(165.0-15.0)/(M_5_PI_12-(-M_5_PI_12)) + (15.0));
    jt_cmds.data.push_back((joint_position_command_[2]-(-M_PI_2))*(180.0-0.0)/(M_PI_2-(-M_PI_2)) + (0.0));
    jt_cmds.data.push_back((joint_position_command_[3]-(-M_PI_2))*(180.0-0.0)/(M_PI_2-(-M_PI_2)) + (0.0));
    jt_cmds.data.push_back((joint_position_command_[4]-(-M_PI_2))*(125.0-0.0)/(high_limit_joint_5-(-M_PI_2)) + (0.0));
    jt_cmds.data.push_back((joint_position_command_[5]-(low_limit_joint_6))*(90.0-0.0)/(M_PI_2-(low_limit_joint_6)) + (0.0));

    cmd_pub.publish(jt_cmds);

}

} /* namespace */