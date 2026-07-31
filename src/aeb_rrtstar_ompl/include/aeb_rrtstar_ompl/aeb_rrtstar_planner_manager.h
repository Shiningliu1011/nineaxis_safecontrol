/*********************************************************************
 * AEB-RRT* PlannerManager plugin for MoveIt2.
 *
 * This plugin wraps the existing OMPL planner infrastructure and
 * additionally registers the AEB-RRT* planner.  It is loaded by
 * MoveIt2's PlanningPipeline via pluginlib when configured in
 * ompl_planning.yaml:
 *
 *   planning_plugin: aeb_rrtstar_ompl/AEBRRTstarPlannerManager
 *********************************************************************/

#ifndef AEB_RRTSTAR_OMPL_PLANNER_MANAGER_H_
#define AEB_RRTSTAR_OMPL_PLANNER_MANAGER_H_

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <rclcpp/node.hpp>
#include <memory>
#include <string>

namespace aeb_rrtstar_ompl
{

/**
 * @brief PlannerManager that extends the OMPL interface with AEB-RRT*.
 *
 * Internally delegates to a standard ompl_interface::OMPLInterface
 * instance and additionally registers the AEB-RRT* planner allocator.
 */
class AEBRRTstarPlannerManager : public planning_interface::PlannerManager
{
public:
    AEBRRTstarPlannerManager();
    ~AEBRRTstarPlannerManager() override;

    // --- planning_interface::PlannerManager interface ---

    bool initialize(const moveit::core::RobotModelConstPtr &model,
                    const rclcpp::Node::SharedPtr &node,
                    const std::string &parameter_namespace) override;

    std::string getDescription() const override;

    void getPlanningAlgorithms(
        std::vector<std::string> &algs) const override;

    planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr &planning_scene,
        const planning_interface::MotionPlanRequest &req,
        moveit_msgs::msg::MoveItErrorCodes &error_code) const override;

    bool canServiceRequest(
        const planning_interface::MotionPlanRequest &req) const override;

    void setPlannerConfigurations(
        const planning_interface::PlannerConfigurationMap &pcs) override;

protected:
    /** \brief Underlying OMPL interface that handles all standard planners. */
    std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface_;

    /** \brief Register the AEB-RRT* planner allocator with the OMPL interface. */
    void registerAEBRRTstar();

    moveit::core::RobotModelConstPtr robot_model_;
    rclcpp::Node::SharedPtr node_;
};

}  // namespace aeb_rrtstar_ompl

#endif  // AEB_RRTSTAR_OMPL_PLANNER_MANAGER_H_
