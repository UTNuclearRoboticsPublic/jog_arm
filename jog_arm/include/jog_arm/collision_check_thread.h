//
// Created by alexander on 1/29/19.
//

#ifndef JOG_ARM_COLLISION_CHECK_THREAD_H
#define JOG_ARM_COLLISION_CHECK_THREAD_H

#include <moveit/robot_model_loader/robot_model_loader.h>
#include "jog_arm_server.h"

namespace jog_arm {

  class CollisionCheckThread {
  public:
    CollisionCheckThread(const JogArmParameters &parameters, JogArmShared &shared_variables,
                         std::shared_ptr<robot_model_loader::RobotModelLoader> model_loader_ptr);
  };

} // namespace jog_arm

#endif //JOG_ARM_COLLISION_CHECK_THREAD_H
