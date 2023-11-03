#include <basestategetter.h>

BaseStateGetter::BaseStateGetter(ros::NodeHandle nh,
                                 std::string linkname = "pelvis"):
    _nh(nh),
    _linkname(linkname)
{

    _pose_topic = std::string("/xbotcore/link_state/") + _linkname + std::string("/pose");
    _twist_topic = std::string("/xbotcore/link_state/") + _linkname + std::string("/twist");

    /* Subscribers */

    _link_pose_sub = _nh.subscribe(_pose_topic, // topic
                                   1,   //queue
                                   &BaseStateGetter::on_pelvis_link_pose_received,  //callback function
                                   this);   // this class

    _link_twist_sub = _nh.subscribe(_twist_topic,
                                    1,
                                    &BaseStateGetter::on_pelvis_link_twist_received,
                                    this);

}

void BaseStateGetter::on_pelvis_link_pose_received(const geometry_msgs::PoseStamped& msg)
{
    tf::poseMsgToEigen(msg.pose, _M_world_from_link);

    // Print test
    /*
    Eigen::Vector3d translation = _M_world_from_link.translation();
    Eigen::Matrix3d rotation = _M_world_from_link.rotation();

    ROS_INFO("Received msg from %s\n"
             "Position\n"
             "x = %f\n"
             "y = %f\n"
             "z = %f\n",
             _twist_topic.c_str(),
             _M_world_from_link.translation().x(),
             _M_world_from_link.translation().y(),
             _M_world_from_link.translation().z());

    cout << "Orientation\n" << rotation << "\n--------\n";
    */
}

void BaseStateGetter::on_pelvis_link_twist_received(const geometry_msgs::TwistStamped& msg)
{
    tf::twistMsgToEigen(msg.twist, _twist);

    // Print test
    /*
    Eigen::Vector3d linear_vel = _twist.head(3);
    Eigen::Vector3d angular_vel = _twist.tail(3);

    ROS_INFO("Received msg from %s\n"
             "Linear       | Angular\n"
             "x = %f   | x = %f\n"
             "y = %f   | y = %f\n"
             "z = %f   | z = %f\n"
             "-----------------\n",
             _pose_topic.c_str(),
             linear_vel.x(),
             angular_vel.x(),
             linear_vel.y(),
             angular_vel.y(),
             linear_vel.z(),
             angular_vel.z());
    */
}

Eigen::Affine3d BaseStateGetter::getPose()
{
    return _M_world_from_link;
}

Eigen::Vector6d BaseStateGetter::getTwist()
{
    return _twist;
}
