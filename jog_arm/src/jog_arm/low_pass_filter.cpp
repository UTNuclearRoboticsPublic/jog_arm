//
// Created by alexander on 1/29/19.
//
#include <jog_arm/low_pass_filter.h>

namespace jog_arm {

  LowPassFilter::LowPassFilter(const double low_pass_filter_coeff) {
    filter_coeff_ = low_pass_filter_coeff;
  }

  void LowPassFilter::reset(const double data) {
    prev_msrmts_[0] = data;
    prev_msrmts_[1] = data;
    prev_msrmts_[2] = data;

    prev_filtered_msrmts_[0] = data;
    prev_filtered_msrmts_[1] = data;
  }

  double LowPassFilter::filter(const double new_msrmt) {
    // Push in the new measurement
    prev_msrmts_[2] = prev_msrmts_[1];
    prev_msrmts_[1] = prev_msrmts_[0];
    prev_msrmts_[0] = new_msrmt;

    double new_filtered_msrmt = (1 / (1 + filter_coeff_ * filter_coeff_ + 1.414 * filter_coeff_)) *
                                (prev_msrmts_[2] + 2 * prev_msrmts_[1] + prev_msrmts_[0] -
                                 (filter_coeff_ * filter_coeff_ - 1.414 * filter_coeff_ + 1) *
                                 prev_filtered_msrmts_[1] -
                                 (-2 * filter_coeff_ * filter_coeff_ + 2) * prev_filtered_msrmts_[0]);

    // Store the new filtered measurement
    prev_filtered_msrmts_[1] = prev_filtered_msrmts_[0];
    prev_filtered_msrmts_[0] = new_filtered_msrmt;

    return new_filtered_msrmt;
  }

} // namespace jog_arm
