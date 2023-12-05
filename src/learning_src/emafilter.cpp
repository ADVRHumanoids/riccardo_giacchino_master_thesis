#include "emafilter.h"

EMAFilter::EMAFilter(double alpha):
    _alpha(alpha)
{
    _filtered_value = Eigen::Vector3d::Zero();  // initialize to zero
}

Eigen::Vector3d EMAFilter::update(Eigen::Vector3d new_value)
{
/*
 * The exponential moving average (EMA) filter is a discrete, low-pass, infinite-impulse response (IIR) filter.
 * It places more weight on recent data by discounting old data in an exponential fashion, and behaves similarly
 * to the discrete first-order low-pass RC filter.
 *
 * The difference equation for an exponential moving average filter is:
 *
 *                      y[i]=α⋅x[i]+(1−α)⋅y[i−1]
 *
 * where y[i] is the new filtered value, y[i-1] is the value filtered at the previous iteration, x[i] is the new
 * value to be filtered, while α is a constant which sets the cutoff frequency (a value between 0 and 1)
 *
 * The constant α determines how aggressive the filter is. It can vary between 0 and 1 (inclusive). As α->0, the
 * filter gets more and more aggressive, until at α = 0, where the input has no effect on the output (if the filter
 * started like this, then the output would stay at 0). As α->1, the filter lets more of the raw input through at
 * less filtered data, until at α=1, where the filter is not “filtering” at all (pass-through from input to output).
 *
 * The cut-off frequency fc (-3dB point) of an EMA filter is given by2:
 *
 *                      fc=fs/(2π) * arccos[1−(α^2)/(2*(1−α))]
 *
 * where: fs is the sampling frequency in Hz
 *
*/
    _filtered_value = _alpha * new_value + (1 - _alpha) * _filtered_value;

    return _filtered_value;

}
