#ifndef CONTROLLERMANAGER_H
#define CONTROLLERMANAGER_H

#include <cartesianimpedancecontroller.h>
#include <xbot_msgs/JointCommand.h>
#include <xbot2/ros/ros_support.h>

using namespace XBot;

class ControllerManager : public ControlPlugin
{
public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;

    void on_start() override;

    void run() override;

    void on_stop() override;

private:

    std::map<std::string, ControlMode> _ctrl_map;

    ModelInterface::Ptr _model;

    // ROS related
    std::unique_ptr<RosSupport> _ros;
    PublisherPtr<xbot_msgs::JointCommand> _pub;

    //std::vector<CartesianImpedanceController> _legs_controller;

    std::vector<std::unique_ptr<CartesianImpedanceController>> _legs_controller;

    vector<string> _end_effector_links;

    vector<Eigen::Vector6d> _stiffness;

    JointNameMap _stiff_initial_state, _damp_initial_state;
    JointNameMap _stiff_tmp_state, _damp_tmp_state;

    vector<string> joint_names;

    int _n_joints = 6;  // TODO: find a way to compute it automatically

};

#endif // CONTROLLERMANAGER_H
