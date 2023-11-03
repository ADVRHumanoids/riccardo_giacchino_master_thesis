#ifndef BASESTATEGETTER_H
#define BASESTATEGETTER_H


#include <thread>   // Standar
#include <iostream> // Standard
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <xbot2/xbot2.h>
#include <xbot2/hal/dev_ft.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>


/**
 * @brief The BaseStateGetter class is used to read from topic the state value about the base, like Pose and Twist of the base w.r.t. the world frame
 */
class BaseStateGetter
{

    public:
        /**
         * @brief Constructor BaseStateGetter: initiate an instance of BaseStateGetter.
         * @param nh is the node handler
         * @param linkname: is the frame for which you want to obtain the Pose and Twist. By default is set to be the "Pelvis"
         */
        BaseStateGetter(ros::NodeHandle nh,
                        std::string linkname);

        /**
         * @brief getPose
         * @return the pose of the frame as a roto-translational matrix w.r.t. the world frame
         */
        Eigen::Affine3d getPose();  // Getter of the pose, since the variable that store this information is private

        /**
         * @brief getTwist
         * @return the twist of the frame w.r.t. the world, compose by linear and angular velocity
         */
        Eigen::Vector6d getTwist(); // Getter of the twist

    private:

        ros::NodeHandle _nh;    // node handler

        std::string _linkname = "pelvis";   // by default is the Pelvis, which is the reference frame of the floating base

        std::string _pose_topic;    // name of the topic for the pose
        std::string _twist_topic;   // name of the topic for the twist

        ros::Subscriber _link_pose_sub; // subscriber to the topic where the pose is published
        ros::Subscriber _link_twist_sub;    // subscriber to the topic where the twist is published

        Eigen::Affine3d _M_world_from_link;
        Eigen::Vector6d _twist;

        /**
         * @brief on_pelvis_link_pose_received: is the callback of the subscriber to the pose topic.
         *
         * It convert the data about the pose, stored in a geometry_msg::PoseStamped type, into an Affine3d object
         *
         * @param msg is the message read on topic by the subscriber
         */
        void on_pelvis_link_pose_received(const geometry_msgs::PoseStamped& msg);   // Callback for topic pose

        /**
         * @brief on_pelvis_link_twist_received: is the callback of the subscriber to the twist topic.
         *
         * It convert the data about the twist, stored in a geometry_msg::TwistStamped type, into an Vector6d object,
         * where the first 3 elements represent the linear velocity, while the last 3 are the angular velocty.
         *
         * @param msg is the message read on topic by the subscriber
         */
        void on_pelvis_link_twist_received(const geometry_msgs::TwistStamped& msg); // Callback for topic twist


};

#endif // BASESTATEGETTER_H
