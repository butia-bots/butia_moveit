#include <moveit_apps/pick_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace butia_moveit_tasks {
    PickTask::PickTask(const std::string &task_name, const ros::NodeHandle &nh) : PickPlaceBase::PickPlaceBase(task_name, nh) {}

    void PickTask::init()
    {
        std::string object = object_name_;
        task_.reset();
        task_.reset(new moveit::task_constructor::Task());
        Task &t = *task_;
        t.stages()->setName("Pick");
        ROS_INFO("Set task name");
        t.loadRobotModel();
        ROS_INFO("Loaded robot model");
        auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
        sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
        ROS_INFO("Created sampling planner");
        auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScaling(1.0);
        cartesian_planner->setMaxAccelerationScaling(1.0);
        cartesian_planner->setStepSize(.01);
        ROS_INFO("Created cartesian planner");
        t.setProperty("group", arm_group_name_);
        t.setProperty("eef", eef_name_);
        t.setProperty("hand", hand_group_name_);
        t.setProperty("hand_grasping_frame", hand_frame_);
        t.setProperty("ik_frame", hand_frame_);
        ROS_INFO("Set task properties");

        /****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
        Stage *current_state_ptr = nullptr;
        {
            auto current_state = std::make_unique<stages::CurrentState>("current state");
            auto applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
            applicability_filter->setPredicate([object](const SolutionBase &s, std::string &comment) {
                if (s.start()->scene()->getCurrentState().hasAttachedBody(object))
                {
                    comment = "object with id '" + object + "' is already attached and cannot be picked";
                    return false;
                }
                return true;
            });
            current_state_ptr = applicability_filter.get();
            t.add(std::move(applicability_filter));
        }

        /****************************************************
	 *                                                  *
	 *               Open Hand                          *
	 *                                                  *
	 ***************************************************/
        {
            auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
            stage->setGroup(hand_group_name_);
            stage->setGoal(hand_open_pose_);
            t.add(std::move(stage));
        }

        /****************************************************
	 *                                                  *
	 *               Move to Pick                       *
	 *                                                  *
	 ***************************************************/
        {
            auto stage = std::make_unique<stages::Connect>("move to pick", stages::Connect::GroupPlannerVector{{arm_group_name_, sampling_planner}});
            stage->setTimeout(5.0);
            stage->properties().configureInitFrom(Stage::PARENT);
            t.add(std::move(stage));
        }

        /****************************************************
	 *                                                  *
	 *               Pick Object                        *
	 *                                                  *
	 ***************************************************/
        Stage *attach_object_stage = nullptr;
        {
            auto grasp = std::make_unique<SerialContainer>("pick object");
            t.properties().exposeTo(grasp->properties(), {"eef", "hand", "group", "ik_frame"});
            grasp->properties().configureInitFrom(Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

            /****************************************************
  ---- *               Approach Object                    *
		 ***************************************************/
            {
                auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
                stage->properties().set("marker_ns", "aprroach_object");
                stage->properties().set("link", hand_frame_);
                stage->properties().configureInitFrom(Stage::PARENT, {"group"});
                stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);
                geometry_msgs::Vector3Stamped vec;
                vec.header.frame_id = hand_frame_;
                vec.vector.z = -1.0;
                stage->setDirection(vec);
                grasp->insert(std::move(stage));
            }

            /****************************************************
  ---- *               Generate Grasp Pose                *
		 ***************************************************/
            {
                auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
                stage->properties().configureInitFrom(Stage::PARENT);
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPreGraspPose(hand_open_pose_);
                stage->setObject(object);
                stage->setAngleDelta(M_PI / 12);
                stage->setMonitoredStage(current_state_ptr);
                auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(8);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
                wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
                grasp->insert(std::move(wrapper));
            }

            /****************************************************
  ---- *               Allow Collision (hand object)   *
		 ***************************************************/
            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand, object");
                stage->allowCollisions(
                    object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(), true);
                grasp->insert(std::move(stage));
            }

            /****************************************************
  ---- *               Close Hand                      *
		 ***************************************************/
            {
                auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
                stage->setGroup(hand_group_name_);
                stage->setGoal(hand_close_pose_);
                grasp->insert(std::move(stage));
            }

            /****************************************************
  .... *               Attach Object                      *
		 ***************************************************/
            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
                stage->attachObject(object, hand_frame_);
                attach_object_stage = stage.get();
                grasp->insert(std::move(stage));
            }

            /****************************************************
  .... *               Allow collision (object support)   *
		 ***************************************************/
            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object, support)");
                stage->allowCollisions({object}, support_surfaces_, true);
                grasp->insert(std::move(stage));
            }

            /****************************************************
  .... *               Lift object                        *
		 ***************************************************/
            {
                auto stage = std::make_unique<stages::MoveRelative>("lift_object", cartesian_planner);
                stage->properties().configureInitFrom(Stage::PARENT, {"group"});
                stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
                stage->setIKFrame(hand_frame_);
                stage->properties().set("marker_ns", "lift_object");
                geometry_msgs::Vector3Stamped vec;
                vec.header.frame_id = world_frame_;
                vec.vector.z = 1.0;
                stage->setDirection(vec);
                grasp->insert(std::move(stage));
            }

            /****************************************************
  .... *               Forbid collision (object support)  *
		 ***************************************************/
            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object, surface)");
                stage->allowCollisions({object}, support_surfaces_, false);
                grasp->insert(std::move(stage));
            }

            t.add(std::move(grasp));
        }

        /*****************************************
         * 
         *          Return home                 *
         *
         ******************************************/
        {
            auto stage = std::make_unique<stages::MoveTo>("return home", sampling_planner);
            stage->setGroup(arm_group_name_);
            stage->setGoal(arm_home_pose_);
            stage->restrictDirection(stages::MoveTo::FORWARD);
            t.add(std::move(stage));
        }
    }
}