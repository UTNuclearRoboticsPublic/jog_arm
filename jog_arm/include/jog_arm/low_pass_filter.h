//
// Created by alexander on 1/29/19.
//

#ifndef JOG_ARM_LOW_PASS_FILTER_H
#define JOG_ARM_LOW_PASS_FILTER_H

#include <vector>

/**
 * Class LowPassFilter - Filter the joint velocities to avoid jerky motion.
 */
class LowPassFilter {
public:
  explicit LowPassFilter(double low_pass_filter_coeff);

  double filter(double new_msrmt);

  void reset(double data);

private:
  std::vector<double> prev_msrmts_ = { 0., 0., 0. };
  std::vector<double> prev_filtered_msrmts_ = { 0., 0. };
  double filter_coeff_ = 10.;
};

#endif //JOG_ARM_LOW_PASS_FILTER_H
