#ifndef EMAFILTER_H
#define EMAFILTER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief The EMAFilter class create a Exponential Moving Average filter (low pass filter)
 */
class EMAFilter
{
    public:

        EMAFilter(double alpha);

        Eigen::Vector3d update (Eigen::Vector3d new_value); //TODO: add value

    private:

        double _alpha;
        Eigen::Vector3d _filtered_value;
};

#endif // EMAFILTER_H
