#include <moveit_apps/pick_place_task.h>
#include <moveit_apps/pick_task.h>
#include <moveit_apps/place_task.h>
#include <moveit_apps/PickPlaceService.h>
#include <moveit_apps/PickPlaceServiceRequest.h>
#include <moveit_apps/PickPlaceServiceResponse.h>

using namespace butia_moveit_tasks;


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "task_server");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    PickPlaceTask pick_place_task("pick_and_place", nh);
    PickTask pick_task("pick", nh);
    PlaceTask place_task("place", nh);
    auto pick_place_service = nh.advertiseService("/butia_moveit/tasks/pick_place", &PickPlaceBase::serviceCallback, dynamic_cast<PickPlaceBase*>(&pick_place_task));
    auto pick_service = nh.advertiseService("/butia_moveit/tasks/pick", &PickPlaceBase::serviceCallback, dynamic_cast<PickPlaceBase*>(&pick_task));
    auto place_service = nh.advertiseService("/butia_moveit/tasks/place", &PickPlaceBase::serviceCallback, dynamic_cast<PickPlaceBase*>(&place_task));
    ros::waitForShutdown();
    return 0;
}
