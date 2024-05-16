#include "ros/ros.h"
#include "std_msgs/String.h"
#include "arm_controller/ArmController.h"
#include <sstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller_node");
    ros::NodeHandle nh;

    ArmController armController(nh);

    ros::spin();

    return 0;
}