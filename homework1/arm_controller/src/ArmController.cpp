//
// Created by stefanocovone on 01/11/23.
//

#include "arm_controller/ArmController.h"

ArmController::ArmController(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
    jointStateSub = nodeHandle_.subscribe("/arm/joint_states", 10, &ArmController::jointStateCallback, this);

    joint0CommandPub = nodeHandle_.advertise<std_msgs::Float64>("arm/PositionJointInterface_J0_controller/command", 10);
    joint1CommandPub = nodeHandle_.advertise<std_msgs::Float64>("arm/PositionJointInterface_J1_controller/command", 10);
    joint2CommandPub = nodeHandle_.advertise<std_msgs::Float64>("arm/PositionJointInterface_J2_controller/command", 10);
    joint3CommandPub = nodeHandle_.advertise<std_msgs::Float64>("arm/PositionJointInterface_J3_controller/command", 10);

    ROS_INFO("Successfully launched node.");
}

ArmController::~ArmController() = default;

void ArmController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Print the current joint positions
    ROS_INFO("\n--Joint States--");
    for (size_t i = 0; i < msg->name.size(); i++) {
        ROS_INFO("Joint %s: Position %.4f", msg->name[i].c_str(), msg->position[i]);
    }
    float increments[4] = {0.2, 0.01, -0.02, 0.2};
    std_msgs::Float64 commandMsg[4];
    for (size_t i = 0; i < msg->name.size(); i++) {
        commandMsg[i].data = msg->position[i]+increments[i]; // Set the command value as needed
    }
    joint0CommandPub.publish(commandMsg[0]); // Publish the command
    joint1CommandPub.publish(commandMsg[1]); // Publish the command
    joint2CommandPub.publish(commandMsg[2]); // Publish the command
    joint3CommandPub.publish(commandMsg[3]); // Publish the command
}