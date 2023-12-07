#ifndef CARTESIO_ROS_WRAPPER_H
#define CARTESIO_ROS_WRAPPER_H

#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <xbot2/system/thread.h>

class CartesioRosWrapper
{

public:

    CartesioRosWrapper(XBot::Cartesian::CartesianInterfaceImpl::Ptr rt_ci,
                       std::string tf_prefix,
                       std::string ros_ns):
        _rt_ci(rt_ci),
        _rt_model(rt_ci->getModel())
    {
        auto nrt_model = XBot::ModelInterface::getModel(rt_ci->getModel()->getConfigOptions());

        auto nrt_ctx = std::make_shared<XBot::Cartesian::Context>(
            std::make_shared<XBot::Cartesian::Parameters>(*rt_ci->getContext()->params()),
            nrt_model);

        _nrt_ci = std::make_shared<XBot::Cartesian::LockfreeBufferImpl>(rt_ci.get(), nrt_ctx);
        _nrt_ci->pushState(rt_ci.get(), rt_ci->getModel().get());
        _nrt_ci->updateState();
        auto nrt_ci = _nrt_ci;

        /*  Create ros api server */
        XBot::Cartesian::RosServerClass::Options opt;
        opt.tf_prefix = tf_prefix;
        opt.ros_namespace = ros_ns;

        /* Initialization */
        _rt_active = false;
        auto rt_active_ptr = &_rt_active;

        _nrt_exit = false;
        auto nrt_exit_ptr = &_nrt_exit;

        _ros_active = false;
        auto _ros_active_ptr = &_ros_active;

        /* Spawn thread */
        _nrt_th = std::make_unique<XBot::thread>(
            [rt_active_ptr, nrt_exit_ptr, _ros_active_ptr, nrt_ci, opt]()
            {
                XBot::this_thread::set_name("cartesio_nrt");
                std::shared_ptr<XBot::Cartesian::RosServerClass> ros_srv;

                while(!*nrt_exit_ptr)
                {
                    if ((*rt_active_ptr) && (!*_ros_active_ptr))
                    {
                        // plugin start detected
                        ros_srv = std::make_shared<XBot::Cartesian::RosServerClass>(nrt_ci, opt);
                        *_ros_active_ptr = true;
                    }

                    XBot::this_thread::sleep_for(std::chrono::milliseconds(10));

                    if (*_ros_active_ptr)
                    {
                        if (*rt_active_ptr)
                        {
                            // normal operation
                            nrt_ci->updateState();
                            ros_srv->run();
                        }

                        else
                        {
                            // plugin stop detected
                            ros_srv.reset();
                            *_ros_active_ptr = false;
                        }
                    }
                }

            });

    }

    void receive()
    {
        /* Receive commands from nrt */
        _nrt_ci->callAvailable(_rt_ci.get());
    }

    void send()
    {
        /* Send state to nrt */
        _nrt_ci->pushState(_rt_ci.get(), _rt_model.get());
    }

    void activate(bool flag)
    {
        _rt_active = flag;
    }

    void close()
    {
        _rt_active = false;
        _nrt_exit = true;
        if(_nrt_th) _nrt_th->join();
    }

private:


    XBot::Cartesian::CartesianInterfaceImpl::Ptr _rt_ci;
    XBot::ModelInterface::Ptr _rt_model;

    XBot::Cartesian::LockfreeBufferImpl::Ptr _nrt_ci;
    std::atomic_bool _rt_active, _ros_active;
    std::atomic_bool _nrt_exit;

    std::unique_ptr<XBot::thread> _nrt_th;

};


#endif // CARTESIO_ROS_WRAPPER_H
