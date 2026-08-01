/*********************************************************************
 * Verify AEB-RRT* plugin can be loaded by pluginlib.
 *********************************************************************/
#include <pluginlib/class_loader.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <iostream>

int main()
{
    // The ClassLoader loads classes implementing planning_interface::PlannerManager
    pluginlib::ClassLoader<planning_interface::PlannerManager> loader(
        "moveit_core", "planning_interface::PlannerManager");

    std::cout << "Available planner plugins:" << std::endl;
    for (const auto &name : loader.getDeclaredClasses())
    {
        std::cout << "  " << name << std::endl;
    }

    std::cout << "\nTrying to load aeb_rrtstar_ompl/AEBRRTstarPlannerManager..."
              << std::endl;

    try
    {
        auto instance = loader.createUniqueInstance(
            "aeb_rrtstar_ompl/AEBRRTstarPlannerManager");
        std::cout << "SUCCESS: Plugin loaded!" << std::endl;
        std::cout << "  Description: " << instance->getDescription() << std::endl;

        // We can't fully initialize without a RobotModel + ROS node,
        // but the fact that it loaded proves pluginlib registration works.
    }
    catch (const pluginlib::PluginlibException &e)
    {
        std::cerr << "FAILED: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "PLUGIN LOAD TEST PASSED" << std::endl;
    return 0;
}
