#pragma once

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <controller_manager/controller_manager.h>

#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <angles/angles.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf/model.h>

#include <std_msgs/UInt16MultiArray.h>
#include <sensor_msgs/JointState.h>


namespace HardwareInterface
{

/*!
 * Class containing the Hardware interface class
*/
class RobotHardwareInterface: public hardware_interface::RobotHW
{
public:
    /*!
	 * Constructor.
	*/
    RobotHardwareInterface(ros::NodeHandle& nh);
    /*!
	 * Destructor.
	 */
    ~RobotHardwareInterface();

    ros::Publisher cmd_pub;
    ros::Subscriber jt_st_sub;

    float jt_states[6];
    std_msgs::UInt16MultiArray jt_cmds;

    float M_5_PI_12;
    float high_limit_joint_5;
    float low_limit_joint_6; 
    
    /*!
     * Functions of the class
     * 
    */
    void init();
    void update(const ros::TimerEvent& e); // The update function has to be a timed event
    void read();
    void write(ros::Duration elapsed_time); // The write function needs to have the elapsed time

    /*!
     * Callback functions 
    */
    void Joint_state_CB(const sensor_msgs::JointState joint_state);

protected:

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
    joint_limits_interface::PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
    
    int num_joints_;
    std::string joint_names_[6];  
    double joint_position_[6];
    double joint_velocity_[6];
    double joint_effort_[6];
    double joint_position_command_[6];
    
    ros::NodeHandle nh_;
    ros::Timer non_realtime_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

}
