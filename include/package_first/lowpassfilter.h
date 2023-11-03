#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>

class LowPassFilter
{
    public:

        // Constructor
        LowPassFilter(float dt);

        // Functions
        Eigen::Vector3d update (Eigen::Vector3d input);


    private:

        float _dt;
        Eigen::Vector3d _filtered_output;
};

#endif // LOWPASSFILTER_H
