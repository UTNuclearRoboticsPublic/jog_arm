
#include "compliant_control/compliant_control.h"
//#include "prettyprint/prettyprint.hpp"

namespace compliant_control {

compliantControl::compliantControl(std::vector<double> stiffness,
                                   std::vector<double> endConditionWrench, double filterCutoff,
                                   geometry_msgs::WrenchStamped bias)
    : stiffness_(stiffness), endConditionWrench_(endConditionWrench) {
  bias_.resize(compliantEnum::NUM_DIMS);
  ft_.resize(compliantEnum::NUM_DIMS);

  for (int i = 0; i < compliantEnum::NUM_DIMS; i++) {
    vectorOfFilters_.push_back(lpf(filterCutoff));
  }
  safeWrenchLimit_ = DBL_MAX; // No condition on accidental control
  bias_[0] = bias.wrench.force.x;
  bias_[1] = bias.wrench.force.y;
  bias_[2] = bias.wrench.force.z;
  bias_[3] = bias.wrench.torque.x;
  bias_[4] = bias.wrench.torque.y;
  bias_[5] = bias.wrench.torque.z;
  ft_ = bias_;
}

// Tare or bias the sensor -- i.e. reset its ground truth
void compliantControl::biasSensor(geometry_msgs::WrenchStamped bias) {
  bias_[0] = bias.wrench.force.x;
  bias_[1] = bias.wrench.force.y;
  bias_[2] = bias.wrench.force.z;
  bias_[3] = bias.wrench.torque.x;
  bias_[4] = bias.wrench.torque.y;
  bias_[5] = bias.wrench.torque.z;

  for (int i = 0; i < compliantEnum::NUM_DIMS; i++) {
    vectorOfFilters_[i].reset(0.);
  }
}

compliantControl::~compliantControl() {}

void compliantControl::setStiffness(std::vector<double> b) {
  if (b.size() != compliantEnum::NUM_DIMS) {
    ROS_ERROR_STREAM("Invalid stiffness vector: ");
  } else {
    for (int i = 0; i < compliantEnum::NUM_DIMS; i++) {
      if (fabs(b[i]) <= 1e-3) {
        ROS_ERROR_STREAM("Stiffness must be non-zero.Ignoring "
                         "Compliance in direction: "
                         << i);
        stiffness_[i] = DBL_MAX;
      } else {
        stiffness_[i] = b[i];
      }
    }
  }
}

void compliantControl::setSafetyLimit(double safeWrenchLimit) { safeWrenchLimit_ = safeWrenchLimit; }

void compliantControl::setExitCondition(std::vector<double> endConditionWrench) {
  if (endConditionWrench.size() != compliantEnum::NUM_DIMS) {
    ROS_ERROR_STREAM("Invalid vector endConditionWrench: ");
  } else {
    for (int i = 0; i < compliantEnum::NUM_DIMS; i++) {
      endConditionWrench_[i] = endConditionWrench[i];
      //ROS_INFO_STREAM("setting condition for compliant control in direction " << i);
    }
  }
}

void compliantControl::getFT(geometry_msgs::WrenchStamped ftData) {
  ft_[0] = vectorOfFilters_[0].filter(ftData.wrench.force.x - bias_[0]);
  ft_[1] = vectorOfFilters_[1].filter(ftData.wrench.force.y - bias_[1]);
  ft_[2] = vectorOfFilters_[2].filter(ftData.wrench.force.z - bias_[2]);
  ft_[3] = vectorOfFilters_[3].filter(ftData.wrench.torque.x - bias_[3]);
  ft_[4] = vectorOfFilters_[4].filter(ftData.wrench.torque.y - bias_[4]);
  ft_[5] = vectorOfFilters_[5].filter(ftData.wrench.torque.z - bias_[5]);
}

compliantEnum::exitCondition
compliantControl::getVelocity(geometry_msgs::TwistStamped vIn,
                              geometry_msgs::WrenchStamped ftData,
                              geometry_msgs::TwistStamped &vOut) {
  compliantEnum::exitCondition exitCondition = compliantEnum::NOT_CONTROLLED;
  std::vector<double> v1(compliantEnum::NUM_DIMS, 0.0);
  v1[0] = vIn.twist.linear.x;
  v1[1] = vIn.twist.linear.y;
  v1[2] = vIn.twist.linear.z;
  v1[3] = vIn.twist.angular.x;
  v1[4] = vIn.twist.angular.y;
  v1[5] = vIn.twist.angular.z;

  std::vector<double> v2 = v1;

  exitCondition = getVelocity(v1, ftData, v2);

  vOut.twist.linear.x = v2[0];
  vOut.twist.linear.y = v2[1];
  vOut.twist.linear.z = v2[2];
  vOut.twist.angular.x = v2[3];
  vOut.twist.angular.y = v2[4];
  vOut.twist.angular.z = v2[5];
  return exitCondition;
}

compliantEnum::exitCondition
compliantControl::getVelocity(std::vector<double> vIn,
                              geometry_msgs::WrenchStamped ftData,
                              std::vector<double> &vOut) {
  compliantEnum::exitCondition exitCondition = compliantEnum::NOT_CONTROLLED;
  getFT(ftData);
  //ROS_ERROR_STREAM("Internal Fitered F: " << ft_[0] << "  " << ft_[1] << "  " <<ft_[2]);

  if ((fabs(ft_[0]) + fabs(ft_[1]) + fabs(ft_[2])) >= safeWrenchLimit_) {
    ROS_ERROR_STREAM(
        "Total force is exceeding the safety values. Stopping motion.");
    vOut = std::vector<double>(6, 0.0);
    return compliantEnum::FT_VIOLATION;
  }

  for (int i = 0; i < compliantEnum::NUM_DIMS; i++) {
    if ( endConditionWrench_[i]>0 )
    {
      if (ft_[i] > endConditionWrench_[i]) {
        ROS_INFO_STREAM("Exit condition met in direction: " << i);
        vOut[i] = 0.0;
        exitCondition = compliantEnum::CONDITION_MET;
      } else {
        vOut[i] = vIn[i] + ft_[i] / stiffness_[i];
        if (exitCondition != compliantEnum::CONDITION_MET) {
          // ROS_INFO_STREAM("Exit condition not met in this iteration");
          exitCondition = compliantEnum::CONDITION_NOT_MET;
        }
      }
    }
    else // endConditionWrench_[i]<=0
    {
      if (ft_[i] < endConditionWrench_[i]) {
        ROS_INFO_STREAM("Exit condition met in direction: " << i);
        vOut[i] = 0.0;
        exitCondition = compliantEnum::CONDITION_MET;
      } else {
        vOut[i] = vIn[i] + ft_[i] / stiffness_[i];
        if (exitCondition != compliantEnum::CONDITION_MET) {
          // ROS_INFO_STREAM("Exit condition not met in this iteration");
          exitCondition = compliantEnum::CONDITION_NOT_MET;
        }
      }
    }
  }
  return exitCondition;
}

lpf::lpf(double filterCutoff) : filterCutoff_(filterCutoff) {}

double lpf::filter(const double &new_msrmt) {
  // Push in the new measurement
  prev_msrmts_[2] = prev_msrmts_[1];
  prev_msrmts_[1] = prev_msrmts_[0];
  prev_msrmts_[0] = new_msrmt;

  double new_filtered_msrmt =
      (1 / (1 + filterCutoff_ * filterCutoff_ + 1.414 * filterCutoff_)) *
      (prev_msrmts_[2] + 2 * prev_msrmts_[1] + prev_msrmts_[0] -
       (filterCutoff_ * filterCutoff_ - 1.414 * filterCutoff_ + 1) * prev_filtered_msrmts_[1] -
       (-2 * filterCutoff_ * filterCutoff_ + 2) * prev_filtered_msrmts_[0]);
  ;

  // Store the new filtered measurement
  prev_filtered_msrmts_[1] = prev_filtered_msrmts_[0];
  prev_filtered_msrmts_[0] = new_filtered_msrmt;

  return new_filtered_msrmt;
}

void lpf::reset(double data) {
  prev_msrmts_ = {data, data, data};
  prev_filtered_msrmts_ = {data, data};
}


}