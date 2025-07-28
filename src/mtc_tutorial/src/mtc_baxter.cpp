#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_baxter_demo");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
    MTCTaskNode(const rclcpp::NodeOptions &options)
        : node_{std::make_shared<rclcpp::Node>("mtc_baxter_node", options)} {}

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }

    void setupPlanningScene()
    {
        moveit_msgs::msg::CollisionObject object;
        object.id = "object";
        object.header.frame_id = "world";
        object.primitives.resize(1);
        object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        object.primitives[0].dimensions = {0.05, 0.05, 0.15};
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.7;
        pose.position.y = 0.2;
        pose.position.z = 0.075;
        pose.orientation.w = 1.0;
        object.pose = pose;

        moveit::planning_interface::PlanningSceneInterface psi;
        psi.applyCollisionObjects({object});
    }

    void doTask()
    {
        task_ = createTask();
        try
        {
            task_.init();
        }
        catch (mtc::InitStageException &e)
        {
            RCLCPP_ERROR_STREAM(LOGGER, e);
            return;
        }
        if (!task_.plan(5))
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
            return;
        }
        task_.introspection().publishSolution(*task_.solutions().front());
        auto result = task_.execute(*task_.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
            return;
        }
    }

private:
    mtc::Task createTask()
    {
        mtc::Task task;
        task.stages()->setName("baxter pick and place");
        task.loadRobotModel(node_);

        const auto &arm_group_name = "left_arm";
        const auto &hand_group_name = "left_gripper";
        const auto &hand_frame = "left_gripper"; // ili "left_gripper_tip" prema SRDF-u

        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

        mtc::Stage *current_state_ptr = nullptr;
        mtc::Stage *attach_object_stage = nullptr;

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        current_state_ptr = stage_state_current.get();
        task.add(std::move(stage_state_current));

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(.01);

        // Open gripper
        auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
        stage_open_hand->setGroup(hand_group_name);
        stage_open_hand->setGoal("left_hand_open");
        task.add(std::move(stage_open_hand));

        // Move to pick
        auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
            "move to pick",
            mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
        stage_move_to_pick->setTimeout(5.0);
        stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_pick));

        // Pick sequence
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
        grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        // Approach
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.1, 0.15);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }
        // Generate grasp pose
        {
            mtc::Stage *attach_object_stage = nullptr;
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("left_hand_open");
            stage->setObject("object");
            stage->setAngleDelta(M_PI / 12);
            stage->setMonitoredStage(current_state_ptr);
            Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
            grasp_frame_transform.translation().z() = 0.08;
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(grasp_frame_transform, hand_frame);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            grasp->insert(std::move(wrapper));
        }
        // Allow collision
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
            stage->allowCollisions("object",
                                   task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                                   true);
            grasp->insert(std::move(stage));
        }
        // Close gripper
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("close");
            grasp->insert(std::move(stage));
        }
        // Attach object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("object", hand_frame);
            attach_object_stage = stage.get();
            grasp->insert(std::move(stage));
        }
        // Lift
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.1, 0.3);
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "lift_object");
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }
        task.add(std::move(grasp));

        // Move to place
        {
            auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
                "move to place",
                mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner},
                                                         {hand_group_name, interpolation_planner}});
            stage_move_to_place->setTimeout(5.0);
            stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
            task.add(std::move(stage_move_to_place));
        }
        // Place sequence
        {
            auto place = std::make_unique<mtc::SerialContainer>("place object");
            task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
            place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
            // Generate place pose
            {
                auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "place_pose");
                stage->setObject("object");
                geometry_msgs::msg::PoseStamped target_pose_msg;
                target_pose_msg.header.frame_id = "object";
                target_pose_msg.pose.position.y = 0.5;
                target_pose_msg.pose.orientation.w = 1.0;
                stage->setPose(target_pose_msg);
                stage->setMonitoredStage(attach_object_stage);
                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(2);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame("object");
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
                place->insert(std::move(wrapper));
            }
            // Open gripper
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
                stage->setGroup(hand_group_name);
                stage->setGoal("left_hand_open");
                place->insert(std::move(stage));
            }
            // Forbid collision
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (gripper,object)");
                stage->allowCollisions("object",
                                       task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                                       false);
                place->insert(std::move(stage));
            }
            // Detach object
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
                stage->detachObject("object", hand_frame);
                place->insert(std::move(stage));
            }
            // Retreat
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setMinMaxDistance(0.1, 0.3);
                stage->setIKFrame(hand_frame);
                stage->properties().set("marker_ns", "retreat");
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = "world";
                vec.vector.x = -0.5;
                stage->setDirection(vec);
                place->insert(std::move(stage));
            }
            task.add(std::move(place));
        }
        // Return home
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setGoal("ready");
            task.add(std::move(stage));
        }
        return task;
    }

    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
                                                     {
      executor.add_node(mtc_task_node->getNodeBaseInterface());
      executor.spin();
      executor.remove_node(mtc_task_node->getNodeBaseInterface()); });

    mtc_task_node->setupPlanningScene();
    mtc_task_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}