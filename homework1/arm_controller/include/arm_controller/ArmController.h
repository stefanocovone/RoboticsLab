//
// Created by stefanocovone on 01/11/23.
//

#ifndef ARM_DESCRIPTION_ARMCONTROLLER_H
#define ARM_DESCRIPTION_ARMCONTROLLER_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"


class ArmController {
public:
    /*!
    * Constructor.
    * @param nodeHandle the ROS node handle.
    */
    ArmController(ros::NodeHandle& nodeHandle);
    /*!
    * Destructor.
    */
    virtual ~ArmController();
private:

/*!
* ROS topic callback method.
* @param message the received message.
*/
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;
    //! ROS topic subscriber.
    ros::Subscriber jointStateSub;
    //! Ros topic publishers.
    ros::Publisher joint0CommandPub;
    ros::Publisher joint1CommandPub;
    ros::Publisher joint2CommandPub;
    ros::Publisher joint3CommandPub;
};


#endif //ARM_DESCRIPTION_ARMCONTROLLER_H
