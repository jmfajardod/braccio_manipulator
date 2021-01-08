#include <ros/ros.h>
#include <ros/spinner.h>
#include <braccio_manipulator/hardware_interface.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Braccio_hardware_interface");
    ros::NodeHandle nh;

    HardwareInterface::RobotHardwareInterface HW_interface(nh);
    
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}