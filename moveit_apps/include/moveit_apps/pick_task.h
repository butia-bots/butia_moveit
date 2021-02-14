#include <moveit_apps/pick_place_base.h>

#pragma once

namespace butia_moveit_tasks {

    class PickTask : public PickPlaceBase
    {
    public:
        PickTask(const std::string &task_name, const ros::NodeHandle &nh);
        ~PickTask() = default;
        void init() override;
    };
}