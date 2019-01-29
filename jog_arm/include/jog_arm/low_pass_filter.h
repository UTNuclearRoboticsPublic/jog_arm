//
// Created by alexander on 1/29/19.
//

#ifndef JOG_ARM_LOW_PASS_FILTER_H
#define JOG_ARM_LOW_PASS_FILTER_H

namespace jog_arm {

/**
 * Class LowPassFilter - Filter the joint velocities to avoid jerky motion.
 */
  class LowPassFilter {
  public:
    explicit LowPassFilter(double low_pass_filter_coeff);

    double filter(double new_msrmt);

    void reset(double data);

    double filter_coeff_ = 10.;

  private:
    double prev_msrmts_[3] = {0., 0., 0.};
    double prev_filtered_msrmts_[2] = {0., 0.};
  };

} // namespace jog_arm

#endif //JOG_ARM_LOW_PASS_FILTER_H
