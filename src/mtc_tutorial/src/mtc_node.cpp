#include <rclcpp/rclcpp.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include "rviz_gazebo_bridge/rviz_gazebo_bridge.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions &options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

public:
  std::shared_ptr<RVizGazeboBridge> rviz_gazebo_bridge;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}, rviz_gazebo_bridge{std::make_shared<RVizGazeboBridge>("panda")}
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;

  object.primitives[0].dimensions = {0.08, 0.02}; // smanji visinu cilindra

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.34; // Pomjeri objekt još malo dalje od baze robota
  pose.position.y = 0.18; // Pomjeri još više udesno za više prostora
  pose.position.z = 0.09; // povisi objekt za više prostora
  pose.orientation.w = 1.0;
  object.pose = pose;

  // Novi objekt: prepreka (kutija)
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.id = "obstacle_box";
  obstacle.header.frame_id = "world";
  obstacle.primitives.resize(1);
  obstacle.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  // dimenzije: x, y, z (npr. 0.2m x 0.2m x 0.2m)
  obstacle.primitives[0].dimensions = {0.2, 0.2, 0.2};

  geometry_msgs::msg::Pose obstacle_pose;
  obstacle_pose.position.x = 0.6;
  obstacle_pose.position.y = 0;   // između ruke i objekta
  obstacle_pose.position.z = 0.1; // na podu - pola visine kutije
  obstacle_pose.orientation.w = 1.0;
  obstacle.pose = obstacle_pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObjects({object, obstacle});
  // rviz_gazebo_bridge->load_from_rviz_to_gazebo();
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
    return;
  }

  RCLCPP_INFO(LOGGER, "Starting task planning...");
  if (!task_.plan(10)) // Povećaj timeout na 10 sekundi
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  RCLCPP_INFO_STREAM(LOGGER, "Found " << task_.solutions().size() << " solutions");
  task_.introspection().publishSolution(*task_.solutions().front());

  RCLCPP_INFO(LOGGER, "Starting task execution...");
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with error code: " << result.val);
    return;
  }

  RCLCPP_INFO(LOGGER, "Task executed successfully!");
  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("pick_and_place_task");
  task.loadRobotModel(node_);

  const auto &arm_group_name = "panda_arm"; // panda_arm
  const auto &hand_group_name = "hand";     // umjesto "hand"
  const auto &hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
  mtc::Stage *attach_object_stage = nullptr;
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);   // Povećaj step size za bolje performanse
  cartesian_planner->setMinFraction(0.8); // Smanji zahtjeve za min_fraction

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));
  // Forward attach_object_stage to place pose generator

  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
  grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                        {"eef", "group", "ik_frame"});

  {
    // Prvo pokušaj s CartesianPath
    cartesian_planner->setMinFraction(0.0);
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    stage->properties().set("marker_ns", "approach_object");
    stage->properties().set("link", hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.01, 0.08); // Smanji min_distance i povećaj max_distance

    // Set hand forward direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.x = 1.0; // ili -1.0
    vec.vector.y = 0.0;
    vec.vector.z = 0.0;
    stage->setDirection(vec);

    // Dodaj fallback na sampling planner
    auto fallback_stage = std::make_unique<mtc::stages::MoveRelative>("approach object fallback", sampling_planner);
    fallback_stage->properties().set("marker_ns", "approach_object_fallback");
    fallback_stage->properties().set("link", hand_frame);
    fallback_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    fallback_stage->setMinMaxDistance(0.01, 0.2);
    fallback_stage->setDirection(vec);

    auto alternatives = std::make_unique<mtc::Alternatives>("approach alternatives");
    // Eksplicitno eksponiraj properties iz parent container-a
    grasp->properties().exposeTo(alternatives->properties(), {"group", "ik_frame"});
    alternatives->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
    alternatives->add(std::move(stage));
    alternatives->add(std::move(fallback_stage));

    grasp->insert(std::move(alternatives));
  }
  {
    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_pose");
    stage->setPreGraspPose("open");
    stage->setObject("object");
    stage->setAngleDelta(M_PI / 36); // precizniji graspovi
    stage->setMonitoredStage(current_state_ptr);

    // Postavi grasp frame tako da Z grippera gleda u +X (bok, prema objektu) i prsti paralelni s podom
    Eigen::Isometry3d grasp_frame_transform;
    grasp_frame_transform.linear() =
        (Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()))
            .toRotationMatrix();
    grasp_frame_transform.translation().x() = -0.09; // još veći offset
    grasp_frame_transform.translation().y() = 0.0;
    grasp_frame_transform.translation().z() = 0.0;

    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(grasp_frame_transform, hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    grasp->insert(std::move(wrapper));
  }
  {
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage->allowCollisions("object",
                           task.getRobotModel()
                               ->getJointModelGroup(hand_group_name)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close");
    grasp->insert(std::move(stage));
  }

  {

    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject("object", hand_frame);
    attach_object_stage = stage.get();
    grasp->insert(std::move(stage));
  }

  {
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.05, 0.15); // Smanji min i max distance
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "lift_object");

    // Set upward direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
  }
  task.add(std::move(grasp));
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner},
                                                 {hand_group_name, interpolation_planner}});
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          {"eef", "group", "ik_frame"});
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";        // Koristimo world frame umjesto obstacle_box
      target_pose_msg.pose.position.x = 0.6;            // isto kao kutija
      target_pose_msg.pose.position.y = 0.0;            // isto kao kutija
      target_pose_msg.pose.position.z = 0.2 + 0.08 / 2; // vrh kutije
      target_pose_msg.pose.orientation.x = 0.0;
      target_pose_msg.pose.orientation.y = 1.0;
      target_pose_msg.pose.orientation.z = 0.0;
      target_pose_msg.pose.orientation.w = 0.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }
    {
      // Allow collision before opening hand
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }
    {
      // Retreat: prvo pokušaj s CartesianPath u +X smjeru (od kutije prema robotu), fallback na sampling planner
      auto cartesian_retreat = std::make_unique<mtc::stages::MoveRelative>("retreat cartesian", cartesian_planner);
      cartesian_retreat->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      cartesian_retreat->setMinMaxDistance(0.01, 0.2);
      cartesian_retreat->setIKFrame(hand_frame);
      cartesian_retreat->properties().set("marker_ns", "retreat");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -1.0; // povlači se od kutije prema robotu
      vec.vector.y = 0.0;
      vec.vector.z = 0.0;
      cartesian_retreat->setDirection(vec);

      auto sampling_retreat = std::make_unique<mtc::stages::MoveRelative>("retreat fallback", sampling_planner);
      sampling_retreat->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      sampling_retreat->setMinMaxDistance(0.01, 0.2);
      sampling_retreat->setIKFrame(hand_frame);
      sampling_retreat->properties().set("marker_ns", "retreat_fallback");
      sampling_retreat->setDirection(vec);

      auto retreat_alternatives = std::make_unique<mtc::Alternatives>("retreat alternatives");
      place->properties().exposeTo(retreat_alternatives->properties(), {"group", "ik_frame"});
      retreat_alternatives->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
      retreat_alternatives->add(std::move(cartesian_retreat));
      retreat_alternatives->add(std::move(sampling_retreat));
      place->insert(std::move(retreat_alternatives));
    }
    task.add(std::move(place));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  return task;
}

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
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  moveit::planning_interface::PlanningSceneInterface psi;
  const auto &collision_objects = psi.getObjects();
  for (const auto &object : collision_objects)
  {
    RCLCPP_INFO_STREAM(LOGGER, "Collision object: " << object.first);
  }
  // mtc_task_node->rviz_gazebo_bridge->load_from_rviz_to_gazebo();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}