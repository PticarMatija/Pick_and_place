#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mtc = moveit::task_constructor;

class BaxterMTCNode
{
public:
  BaxterMTCNode(const rclcpp::NodeOptions &options)
      : node_{std::make_shared<rclcpp::Node>("baxter_mtc_node", options)}
  {
  }

  void setupPlanningScene()
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";

    object.header.frame_id = "base";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = {0.025, 0.025, 0.10};
    // širina, dužina, visina objekta
    object.pose.position.x = 0.6;  // ili čak 0.55
    object.pose.position.y = 0.3;  // više ulijevo
    object.pose.position.z = 0.23; // increase from 0.15 to 0.20
    object.pose.orientation.w = 1.0;

    /*  moveit_msgs::msg::CollisionObject table;
     table.id = "table";
     table.header.frame_id = "base";
     table.primitives.resize(1);
     table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
     table.primitives[0].dimensions = {0.6, 1.2, 0.03}; // širina, dužina, visina stola
     table.pose.position.x = 0.7;
     table.pose.position.y = 0.0;
     table.pose.position.z = 0.015; // niži stol
     table.pose.orientation.w = 1.0; */

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObjects({object});
  }

  void doTask()
  {
    mtc::Task task;
    task.stages()->setName("baxter pick and place");
    task.loadRobotModel(node_);

    const std::string arm_group = "left_arm";
    const std::string hand_group = "left_hand";
    const std::string hand_frame = "left_gripper";
    const std::string eef_name = "left_hand_eef";

    task.setProperty("group", arm_group);
    task.setProperty("eef", eef_name);
    task.setProperty("ik_frame", hand_frame);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage *attach_object_stage = nullptr;
#pragma GCC diagnostic pop
    // Current state
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));

    // Open hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("left_hand_open");
      task.add(std::move(stage));
    }

    // Move to pick
    {
      auto stage = std::make_unique<mtc::stages::Connect>(
          "move to pick",
          mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage));
    }

    // Pick container
    auto pick = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(pick->properties(), {"eef", "group", "ik_frame"});
    pick->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // Approach
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.15, 0.20);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base"; // change from hand_frame to base
      vec.vector.x = 0.0;
      vec.vector.y = 1.0;
      vec.vector.z = 0.0;
      stage->setDirection(vec);
      pick->insert(std::move(stage));
    }

    // Generate grasp pose
    //
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("left_hand_open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 72); // još više uzoraka
      stage->setMonitoredStage(task.stages()->findChild("current"));

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      // Try a slight tilt to avoid finger collision
      grasp_frame_transform.linear() =
          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
          Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitX()).toRotationMatrix();
      grasp_frame_transform.translation().y() = -0.04; // closer to object
      grasp_frame_transform.translation().z() = 0.015; // slightly above center
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(32);
      wrapper->setMinSolutionDistance(0.5);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      pick->insert(std::move(wrapper));
    }

    // Allow collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object", {hand_frame}, true);
      pick->insert(std::move(stage));
    }

    // Close hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("close");
      pick->insert(std::move(stage));
    }

    // Attach object
    {

      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      pick->insert(std::move(stage));
    }

    // Lift object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.2);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      pick->insert(std::move(stage));
    }
    task.add(std::move(pick));

    // Move to place
    {
      auto stage = std::make_unique<mtc::stages::Connect>(
          "move to place",
          mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage));
    }

    // Place container
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
      target_pose_msg.header.frame_id = "base";
      target_pose_msg.pose.position.x = 0.7;
      target_pose_msg.pose.position.y = 0.3;
      target_pose_msg.pose.position.z = 0.15;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(task.stages()->findChild("current"));

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    // Open hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group);
      stage->setGoal("left_hand_open");
      place->insert(std::move(stage));
    }

    // Forbid collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object", {hand_frame}, false);
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
      stage->setMinMaxDistance(0.1, 0.2);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base";
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));

    // Return home
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setGoal("left_ready");
      task.add(std::move(stage));
    }

    // Plan & execute
    try
    {
      task.init();
    }
    catch (mtc::InitStageException &e)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Task init failed: " << e);
      return;
    }

    if (!task.plan(5))
    {
      RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
      return;
    }
    task.introspection().publishSolution(*task.solutions().front());

    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Task execution failed");
      return;
    }
  }

  rclcpp::Node::SharedPtr node_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<BaxterMTCNode>(options);
  node->setupPlanningScene();
  node->doTask();

  rclcpp::shutdown();
  return 0;
}