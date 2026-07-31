/*********************************************************************
 * AEB-RRT* PlannerManager plugin implementation.
 *
 * Wraps the standard OMPL interface and adds AEB-RRT* as an
 * additional planner choice via PlanningContextManager::registerPlannerAllocator.
 *********************************************************************/

#include "aeb_rrtstar_ompl/aeb_rrtstar_planner_manager.h"
#include "aeb_rrtstar_ompl/AEBRRTstar.h"

#include <pluginlib/class_list_macros.hpp>
#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <ompl/base/SpaceInformation.h>

namespace aeb_rrtstar_ompl
{

// ======================================================================
//  Constructor / Destructor
// ======================================================================

AEBRRTstarPlannerManager::AEBRRTstarPlannerManager()
  : planning_interface::PlannerManager()
{
}

AEBRRTstarPlannerManager::~AEBRRTstarPlannerManager() = default;

// ======================================================================
//  Initialize
// ======================================================================

bool AEBRRTstarPlannerManager::initialize(
    const moveit::core::RobotModelConstPtr &model,
    const rclcpp::Node::SharedPtr &node,
    const std::string &parameter_namespace)
{
    robot_model_ = model;
    node_ = node;

    // Create the underlying OMPL interface (this registers all default planners)
    ompl_interface_ = std::make_unique<ompl_interface::OMPLInterface>(
        model, node, parameter_namespace);

    // Register AEB-RRT* as an additional planner
    registerAEBRRTstar();

    RCLCPP_INFO(node->get_logger(),
                "AEBRRTstarPlannerManager: initialized with AEB-RRT* planner");

    return true;
}

// ======================================================================
//  Register AEB-RRT* planner
// ======================================================================

void AEBRRTstarPlannerManager::registerAEBRRTstar()
{
    // Create the allocator function
    ompl_interface::ConfiguredPlannerAllocator allocator =
        [](const ompl::base::SpaceInformationPtr &si,
           const std::string & /*name*/,
           const ompl_interface::ModelBasedPlanningContextSpecification & /*spec*/)
        -> ompl::base::PlannerPtr
        {
            auto planner = std::make_shared<ompl::geometric::AEBRRTstar>(si);
            return planner;
        };

    // Register with the OMPL interface's context manager
    ompl_interface_->getPlanningContextManager()
        .registerPlannerAllocator("geometric::AEBRRTstar", allocator);

    // Also register an alias with the standard MoveIt2 naming convention
    ompl_interface_->getPlanningContextManager()
        .registerPlannerAllocator("geometric::AEBRRTstarFaithful", allocator);

    RCLCPP_INFO(node_->get_logger(),
                "AEBRRTstarPlannerManager: registered 'geometric::AEBRRTstar' "
                "and 'geometric::AEBRRTstarFaithful' planner allocators");
}

// ======================================================================
//  PlannerManager interface (delegating to OMPL interface)
// ======================================================================

std::string AEBRRTstarPlannerManager::getDescription() const
{
    return "AEB-RRT* OMPL Planner Manager (extends standard OMPL interface "
           "with Adaptive Extension Bidirectional RRT*)";
}

void AEBRRTstarPlannerManager::getPlanningAlgorithms(
    std::vector<std::string> &algs) const
{
    const auto &configs = ompl_interface_->getPlannerConfigurations();
    algs.clear();
    for (const auto &[name, config] : configs)
        algs.push_back(config.group + "[" + name + "]");
}

planning_interface::PlanningContextPtr
AEBRRTstarPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr &planning_scene,
    const planning_interface::MotionPlanRequest &req,
    moveit_msgs::msg::MoveItErrorCodes &error_code) const
{
    // Delegate to the underlying OMPL interface
    return ompl_interface_->getPlanningContext(planning_scene, req, error_code);
}

bool AEBRRTstarPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest &req) const
{
    // Check if this is a joint-space planning request (OMPL handles these)
    if (req.goal_constraints.empty())
        return false;

    // OMPL can handle most joint-space requests
    // Check for position/orientation constraints
    for (const auto &gc : req.goal_constraints)
    {
        if (!gc.joint_constraints.empty() ||
            !gc.position_constraints.empty() ||
            !gc.orientation_constraints.empty())
        {
            return true;
        }
    }

    return false;
}

void AEBRRTstarPlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap &pcs)
{
    ompl_interface_->setPlannerConfigurations(pcs);
}

}  // namespace aeb_rrtstar_ompl

// ======================================================================
//  Pluginlib export
// ======================================================================

PLUGINLIB_EXPORT_CLASS(aeb_rrtstar_ompl::AEBRRTstarPlannerManager,
                       planning_interface::PlannerManager)
