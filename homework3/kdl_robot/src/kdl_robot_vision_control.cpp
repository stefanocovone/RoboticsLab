#include "kdl_ros_control/kdl_robot.h"
#include "kdl_ros_control/kdl_control.h"
#include "kdl_ros_control/kdl_planner.h"

#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"
#include <std_srvs/Empty.h>
#include "eigen_conversions/eigen_kdl.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/SetModelConfiguration.h"

#include "geometry_msgs/Vector3.h"

#define USING_CONTROLLER_2B 0

// Global variables
std::vector<double> jnt_pos(7, 0.0), init_jnt_pos(7, 0.0), jnt_vel(7, 0.0), aruco_pose(7, 0.0);
bool robot_state_available = false, aruco_pose_available = false;
double lambda = 10 * 0.2;
double KP = 15;

// Functions
KDLRobot createRobot(std::string robot_string) {
    KDL::Tree robot_tree;
    urdf::Model my_model;
    if (!my_model.initFile(robot_string)) {
        printf("Failed to parse urdf robot model \n");
    }
    if (!kdl_parser::treeFromUrdfModel(my_model, robot_tree)) {
        printf("Failed to construct kdl tree \n");
    }

    KDLRobot robot(robot_tree);
    return robot;
}

void jointStateCallback(const sensor_msgs::JointState &msg) {
    robot_state_available = true;
    // Update joints
    jnt_pos.clear();
    jnt_vel.clear();
    for (int i = 0; i < msg.position.size(); i++) {
        jnt_pos.push_back(msg.position[i]);
        jnt_vel.push_back(msg.velocity[i]);
    }
}

void arucoPoseCallback(const geometry_msgs::PoseStamped &msg) {
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);
}


// Main
int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Please, provide a path to a URDF file...\n");
        return 0;
    }

    // Init node
    ros::init(argc, argv, "kdl_ros_control_node");
    ros::NodeHandle n;

    // Rate
    ros::Rate loop_rate(500);

    // Subscribers
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);
    ros::Subscriber joint_state_sub = n.subscribe("/iiwa/joint_states", 1, jointStateCallback);

    // Publishers
    ros::Publisher joint1_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J1_controller/command",
                                                                  1);
    ros::Publisher joint2_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J2_controller/command",
                                                                  1);
    ros::Publisher joint3_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J3_controller/command",
                                                                  1);
    ros::Publisher joint4_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J4_controller/command",
                                                                  1);
    ros::Publisher joint5_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J5_controller/command",
                                                                  1);
    ros::Publisher joint6_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J6_controller/command",
                                                                  1);
    ros::Publisher joint7_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J7_controller/command",
                                                                  1);
    // publisher to plot s
    ros::Publisher s_pub = n.advertise<geometry_msgs::Vector3>("/iiwa/s", 1);
    // publisher to plot the error
    ros::Publisher err_pub = n.advertise<std_msgs::Float64>("/iiwa/error", 1);

    // Services
    ros::ServiceClient robot_set_state_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>(
            "/gazebo/set_model_configuration");
    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

    // Initial desired robot state
    init_jnt_pos[0] = 0.0;
    init_jnt_pos[1] = 1.57;
    init_jnt_pos[2] = -1.57;
    init_jnt_pos[3] = -1.57;
    init_jnt_pos[4] = 1.57;
    init_jnt_pos[5] = -1.57;
    init_jnt_pos[6] = +1.57;
    Eigen::VectorXd qdi = toEigen(init_jnt_pos);

    // Create robot
    KDLRobot robot = createRobot(argv[1]);

    // Messages
    std_msgs::Float64 dq1_msg, dq2_msg, dq3_msg, dq4_msg, dq5_msg, dq6_msg, dq7_msg, err_msg;
    std_srvs::Empty pauseSrv;

    // Joints
    KDL::JntArray qd(robot.getNrJnts()), dqd(robot.getNrJnts()), ddqd(robot.getNrJnts());
    qd.data.setZero();
    dqd.data.setZero();
    ddqd.data.setZero();

    // Wait for robot and object state
    while (!(robot_state_available)) {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Bring robot in a desired initial configuration with velocity control
    ROS_INFO("Robot going into initial configuration....");
    double jnt_position_error_norm = computeJointErrorNorm(toEigen(init_jnt_pos), toEigen(jnt_pos));
    while (jnt_position_error_norm > 0.01) {
        dqd.data = KP * (toEigen(init_jnt_pos) - toEigen(jnt_pos));

        // Set joints
        dq1_msg.data = dqd.data[0];
        dq2_msg.data = dqd.data[1];
        dq3_msg.data = dqd.data[2];
        dq4_msg.data = dqd.data[3];
        dq5_msg.data = dqd.data[4];
        dq6_msg.data = dqd.data[5];
        dq7_msg.data = dqd.data[6];

        // Publish
        joint1_dq_pub.publish(dq1_msg);
        joint2_dq_pub.publish(dq2_msg);
        joint3_dq_pub.publish(dq3_msg);
        joint4_dq_pub.publish(dq4_msg);
        joint5_dq_pub.publish(dq5_msg);
        joint6_dq_pub.publish(dq6_msg);
        joint7_dq_pub.publish(dq7_msg);

        jnt_position_error_norm = computeJointErrorNorm(toEigen(init_jnt_pos), toEigen(jnt_pos));
        std::cout << "jnt_position_error_norm: " << jnt_position_error_norm << "\n" << std::endl;
        ros::spinOnce();
        loop_rate.sleep();

    }

    // Specify an end-effector: camera in flange transform
    KDL::Frame ee_T_cam;
    ee_T_cam.M = KDL::Rotation::RotY(1.57) * KDL::Rotation::RotZ(-1.57);
    ee_T_cam.p = KDL::Vector(0, 0, 0.025);
    robot.addEE(ee_T_cam);

    // Update robot
    robot.update(jnt_pos, jnt_vel);

    // Retrieve initial simulation time
    ros::Time begin = ros::Time::now();
    ROS_INFO_STREAM_ONCE("Starting control loop ...");

    // Retrieve initial ee pose
    KDL::Frame Fi = robot.getEEFrame();
    Eigen::Vector3d pdi = toEigen(Fi.p);

    geometry_msgs::Vector3 s_msg;

    while (ros::ok()) {
        if (robot_state_available && aruco_pose_available) {
            // Update robot
            robot.update(jnt_pos, jnt_vel);

            // Update time
            double t = (ros::Time::now() - begin).toSec();
            std::cout << "time: " << t << std::endl;

            // compute current jacobians
            KDL::Jacobian J_cam = robot.getEEJacobian();
            KDL::Frame cam_T_object(
                    KDL::Rotation::Quaternion(aruco_pose[3], aruco_pose[4], aruco_pose[5], aruco_pose[6]),
                    KDL::Vector(aruco_pose[0], aruco_pose[1], aruco_pose[2]));
            KDL::Frame base_T_object = robot.getEEFrame() * cam_T_object;

            // compute offset transformation
            // KDL::Rotation R_off(1,0,0,0,-1,0,0,0,-1);
            KDL::Rotation R_off = KDL::Rotation::RotX(3.14);
            KDL::Vector P_off(0, 0, 0.3);
            KDL::Frame T_offset(R_off, P_off);
            KDL::Frame T_desired = base_T_object * T_offset;


            // look at point: compute rotation error from angle/axis
            Eigen::Matrix<double, 3, 1> aruco_pos_n = toEigen(
                    cam_T_object.p); //(aruco_pose[0],aruco_pose[1],aruco_pose[2]);
            aruco_pos_n.normalize();    // this is the unit vector describing position of marker wrt camera (s in the pdf)
            // Eigen::Vector3d r_o = skew(Eigen::Vector3d(0,0,1))*aruco_pos_n;
            // double aruco_angle = std::acos(Eigen::Vector3d(0,0,1).dot(aruco_pos_n));
            // KDL::Rotation Re = KDL::Rotation::Rot(KDL::Vector(r_o[0], r_o[1], r_o[2]), aruco_angle);

            if (!USING_CONTROLLER_2B) {
                // using the controller required by point 2.a
                //   Eigen::Matrix<double, 3, 1> e_o = computeOrientationError(toEigen(robot.getEEFrame().M * Re),
                //                                                             toEigen(robot.getEEFrame().M));
                //   Eigen::Matrix<double, 3, 1> e_o_w = computeOrientationError(toEigen(Fi.M), toEigen(robot.getEEFrame().M));
                Eigen::Matrix<double, 3, 1> e_o_n = computeOrientationError(toEigen(T_desired.M),
                                                                            toEigen(robot.getEEFrame().M));
                //   Eigen::Matrix<double,3,1> e_p = computeLinearError(pdi,toEigen(robot.getEEFrame().p));
                Eigen::Matrix<double, 3, 1> e_p = computeLinearError(toEigen(T_desired.p),
                                                                     toEigen(robot.getEEFrame().p));
                //   Eigen::Matrix<double, 6, 1> x_tilde;
                //   x_tilde << e_p, e_o_w[0], e_o[1], e_o[2];
                Eigen::Matrix<double, 6, 1> x_tilde;
                x_tilde << e_p, e_o_n;
                err_msg.data = x_tilde.norm();

                // resolved velocity control low
                Eigen::MatrixXd J_pinv = J_cam.data.completeOrthogonalDecomposition().pseudoInverse();
                dqd.data = 1.5 * J_pinv * x_tilde +
                           10 * (Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J_cam.data) *
                           (qdi - toEigen(jnt_pos));
            } else {
                // Using the controller required by point 2.b
                Eigen::Matrix<double, 3, 3> R_c = toEigen(robot.getEEFrame().M);
                Eigen::Matrix<double, 3, 6> L;
                // L being a 3x6 matrix, it is defined by two 3x3 blocks
                L.block(0, 0, 3, 3) = (-1 / cam_T_object.p.Norm()) * (Eigen::Matrix<double, 3, 3>::Identity() -
                                                                      (aruco_pos_n * aruco_pos_n.transpose())) *
                                      R_c.transpose();
                L.block(0, 3, 3, 3) = skew(aruco_pos_n) * R_c.transpose();
                // note: matrix R_c is transposed and then post-multiplied. R_c represents the rotation to go from base frame to camera frame.
                //        Therefore, we need the transposed matrix to go from the camera to the base frame

                Eigen::MatrixXd LJ_pinv = (L * J_cam.data).completeOrthogonalDecomposition().pseudoInverse();
                // Eigen::MatrixXd LJ_pinv = weightedPseudoInverse(L,J_cam.data);

                Eigen::MatrixXd Null_projector = Eigen::Matrix<double, 7, 7>::Identity() - (LJ_pinv * (L * J_cam.data));
                Eigen::Vector3d sd;
                sd << 0, 0, 1;
                // Eigen::Matrix<double,7,1> q0_dot; q0_dot << 0,0,0,0.1,0.1,0.1,0.1;
                dqd.data = 2 * LJ_pinv * sd + Null_projector * (qdi - toEigen(jnt_pos));
                // std::cout << "dqd: \n" << dqd.data << std::endl;


                /* during code refactor a bug emerged - correct implementation below
                // PAPER IMPLEMENTATION OF NULL SPACE PROJECTOR
                Eigen::Matrix<double, 6, 1> n1, n2, n3, n4;
                Eigen::Vector3d ex, ey, s;
                s = aruco_pos_n;
                ex << 1, 0, 0;
                ey << 0, 1, 0;
                double d = cam_T_object.p.Norm();
                Eigen::Matrix<double, 3, 3> Ps =
                        Eigen::Matrix<double, 3, 3>::Identity() - (s * s.transpose());

                Eigen::Matrix<double, 6, 4> N;
                N.col(0).topRows(3) = s;
                N.col(0).bottomRows(3) = Eigen::Vector3d::Zero();
                N.col(1).topRows(3) = Eigen::Vector3d::Zero();
                N.col(1).bottomRows(3) = s;
                N.col(2).topRows(3) = -skew(s) * ey;
                N.col(2).bottomRows(3) = -Ps * ey;
                N.col(3).topRows(3) = skew(s) * ex;
                N.col(3).bottomRows(3) = Ps * ex;

                Eigen::Vector4d lambda;
                lambda[1] = 0 * 0.1 * std::sin(t);
                lambda[2] = 0 * 0.1 * std::sin(t);
                lambda[3] = 0 * 0.1 * std::sin(t);
                lambda[4] = 1 * 0.1 * std::sin(t);

                Eigen::MatrixXd J_dagger =
                        J_cam.data.completeOrthogonalDecomposition().pseudoInverse();
                dqd.data = 2 * LJ_pinv * sd + J_dagger * N * lambda +
                           0.5 * Null_projector * (qdi - toEigen(jnt_pos));

                s_msg.x = aruco_pos_n(0, 0);
                s_msg.y = aruco_pos_n(1, 0);
                s_msg.z = aruco_pos_n(2, 0);
                 */

                // IMPLEMENTATION FROM A PREVIOUS COMMIT
                Eigen::Matrix<double,6,1> n1,n2,n3,n4;
                Eigen::Vector3d ex, ey, s;
                s = aruco_pos_n;
                double d = cam_T_object.p.Norm();
                Eigen::Matrix<double,3,3> Ps = Eigen::Matrix<double,3,3>::Identity() -
                                               (s * s.transpose());
                // unit vectors as defined in the paper
                ex << 1,0,0;
                ey << 0,1,0;

                // null matrix definition
                n1.topRows(3) = s;
                n1.bottomRows(3) = Eigen::Vector3d::Zero();

                n2.topRows(3) = Eigen::Vector3d::Zero();
                n2.bottomRows(3) = s;

                n3.topRows(3) = -skew(s)*ey;
                n3.bottomRows(3) = -Ps*ey;

                n4.topRows(3) = skew(s)*ex;
                n4.bottomRows(3) = Ps*ex;


                double lambda1, lambda2, lambda3, lambda4, roll, pitch, yaw;
                robot.getEEFrame().M.GetRPY(roll,pitch,yaw);
                std::cout << "\nroll: " << roll << "\n\n";
                // pseudo velocities
                lambda1 = 0*0.1*std::sin(t);
                lambda2 = 0*0.1*std::sin(t);
                lambda3 = 0*0.1*std::sin(t);
                lambda4 = 1*0.1*std::sin(t);
                Eigen::Vector4d lambda;
                lambda << lambda1, lambda2, lambda3, lambda4;
                Eigen::Matrix<double,6,4> N;
                N.col(0) = n1;
                N.col(1) = n2;
                N.col(2) = n3;
                N.col(3) = n4;


                Eigen::MatrixXd J_dagger = J_cam.data.completeOrthogonalDecomposition().pseudoInverse();
                dqd.data = 2 * LJ_pinv * sd + J_dagger*N*lambda + 0.5*Null_projector * (qdi - toEigen(jnt_pos));


                s_msg.x = aruco_pos_n(0, 0);
                s_msg.y = aruco_pos_n(1, 0);
                s_msg.z = aruco_pos_n(2, 0);

            }
            // debug
            // std::cout << "x_tilde: " << std::endl << x_tilde << std::endl;
            // std::cout << "R: " << std::endl << toEigen(cam_T_object.M) << std::endl;
            // std::cout << "P: " << std::endl << toEigen(cam_T_object.p) << std::endl;
            // std::cout << "aruco_pos_n: " << std::endl << aruco_pos_n << std::endl;
            // std::cout << "aruco_pos_n.norm(): " << std::endl << aruco_pos_n.norm() << std::endl;
            // std::cout << "Re: " << std::endl << Re << std::endl;
            // std::cout << "jacobian: " << std::endl << robot.getEEJacobian().data << std::endl;
            // std::cout << "jsim: " << std::endl << robot.getJsim() << std::endl;
            // std::cout << "c: " << std::endl << robot.getCoriolis().transpose() << std::endl;
            // std::cout << "g: " << std::endl << robot.getGravity().transpose() << std::endl;
            // std::cout << "qd: " << std::endl << qd.data.transpose() << std::endl;
            // std::cout << "q: " << std::endl << robot.getJntValues().transpose() << std::endl;
            // std::cout << "tau: " << std::endl << tau.transpose() << std::endl;
            // std::cout << "desired_pose: " << std::endl << des_pose << std::endl;
            // std::cout << "current_pose: " << std::endl << robot.getEEFrame() << std::endl;
            // double alpha,beta,gamma;
            // cam_T_object.M.GetEulerZYX(alpha,beta,gamma);
            // Eigen::Vector3d euler; euler << alpha,beta,gamma;
            // std::cout << "euler angles:" <<std::endl << euler <<std::endl <<std::endl;
            // std::cout << "desired rotaion:" <<std::endl << R_off <<std::endl <<std::endl;
        } else {
            // if either the robot state or the aruco state are not available, go back to the initial pos
            dqd.data = KP * (toEigen(init_jnt_pos) - toEigen(jnt_pos));
        }
        // Set joints
        dq1_msg.data = dqd.data[0];
        dq2_msg.data = dqd.data[1];
        dq3_msg.data = dqd.data[2];
        dq4_msg.data = dqd.data[3];
        dq5_msg.data = dqd.data[4];
        dq6_msg.data = dqd.data[5];
        dq7_msg.data = dqd.data[6];

        // Publish
        joint1_dq_pub.publish(dq1_msg);
        joint2_dq_pub.publish(dq2_msg);
        joint3_dq_pub.publish(dq3_msg);
        joint4_dq_pub.publish(dq4_msg);
        joint5_dq_pub.publish(dq5_msg);
        joint6_dq_pub.publish(dq6_msg);
        joint7_dq_pub.publish(dq7_msg);

        s_pub.publish(s_msg);
        err_pub.publish(err_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}