#include "lowpassfilter.h"

LowPassFilter::LowPassFilter(float dt):
    _dt(dt),
    _filtered_output(Eigen::Vector3d::Zero())
{
}

Eigen::Vector3d LowPassFilter::update(Eigen::Vector3d input)
{
    /*
        Low pass filter in discrete time:


    */
    float c1 = (1 - _dt/2)/(1 + _dt/2);
    float c2 = (_dt)/(1 + _dt/2);

    _filtered_output = (c1 * _filtered_output + c2 * input);

    return _filtered_output;
}


