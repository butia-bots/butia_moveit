#include <moveit_apps/pick_place_base.h>

#pragma once

namespace butia_moveit_tasks {

    class PlaceTask : public PickPlaceBase
    {
    public:
        PlaceTask(const std::string &task_name, const ros::NodeHandle &nh);
        ~PlaceTask() = default;
        void init() override;
        bool serviceCallback(moveit_apps::PickPlaceServiceRequest &req, moveit_apps::PickPlaceServiceResponse &res) override;
    };
}