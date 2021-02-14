#include <moveit_apps/pick_place_base.h>

#pragma once

namespace butia_moveit_tasks
{

    class PickPlaceTask : public PickPlaceBase
    {
    public:
        PickPlaceTask(const std::string &task_name, const ros::NodeHandle &nh);
        ~PickPlaceTask() = default;
        void init() override;
    };
} // namespace butia_moveit_tasks
