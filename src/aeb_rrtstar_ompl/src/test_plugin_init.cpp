/*********************************************************************
 * Integration test: initialize AEB-RRT* PlannerManager with the
 * ninezzhou RobotModel and verify planning algorithms.
 *********************************************************************/
#include <pluginlib/class_loader.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_aeb_init");

    // Find the URDF and SRDF from the installed ninezzhou packages
    std::string urdf_path, srdf_path;
    try {
        std::string share = ament_index_cpp::get_package_share_directory("ninezzhou");
        urdf_path = share + "/urdf/ninezzhou.urdf";
    } catch (...) {
        // Fallback: use absolute path from workspace
        urdf_path = "/home/lsn/robot_safecontrol/models/ninezzhou/urdf/ninezzhou.urdf";
    }
    try {
        std::string share = ament_index_cpp::get_package_share_directory(
            "ninezzhou_moveit_config");
        srdf_path = share + "/config/ninezzhou.srdf";
    } catch (...) {
        srdf_path = "/home/lsn/robot_safecontrol/models/ninezzhou_moveit_config/config/ninezzhou.srdf";
    }

    std::cout << "URDF: " << urdf_path << std::endl;
    std::cout << "SRDF: " << srdf_path << std::endl;

    // Load robot model via RDFLoader (needs ROS params)
    node->declare_parameter("robot_description", "");
    node->declare_parameter("robot_description_semantic", "");

    std::ifstream urdf_file(urdf_path);
    std::string urdf_xml((std::istreambuf_iterator<char>(urdf_file)),
                          std::istreambuf_iterator<char>());
    node->set_parameter(rclcpp::Parameter("robot_description", urdf_xml));

    std::ifstream srdf_file(srdf_path);
    std::string srdf_xml((std::istreambuf_iterator<char>(srdf_file)),
                          std::istreambuf_iterator<char>());
    node->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf_xml));

    rdf_loader::RDFLoader rdf_loader(node);
    auto srdf = std::make_shared<srdf::Model>();
    srdf->initString(*rdf_loader.getURDF(), srdf_xml);

    auto robot_model = std::make_shared<moveit::core::RobotModel>(
        rdf_loader.getURDF(), srdf);
    std::cout << "Robot model: " << robot_model->getName() << std::endl;

    // Load our plugin
    pluginlib::ClassLoader<planning_interface::PlannerManager> class_loader(
        "moveit_core", "planning_interface::PlannerManager");

    std::cout << "\nLoading aeb_rrtstar_ompl/AEBRRTstarPlannerManager..." << std::endl;
    auto planner_manager = class_loader.createUniqueInstance(
        "aeb_rrtstar_ompl/AEBRRTstarPlannerManager");

    // Initialize with robot model
    std::cout << "Initializing..." << std::endl;
    bool ok = planner_manager->initialize(robot_model, node, "test_aeb");
    std::cout << "Initialize: " << (ok ? "SUCCESS" : "FAILED") << std::endl;

    if (ok)
    {
        // Get available planning algorithms
        std::vector<std::string> algs;
        planner_manager->getPlanningAlgorithms(algs);
        std::cout << "\nPlanning algorithms (" << algs.size() << "):" << std::endl;
        for (const auto &a : algs)
            std::cout << "  " << a << std::endl;

        // Check if AEB-RRT* is available
        bool found_aeb = false;
        for (const auto &a : algs)
            if (a.find("AEBRRTstar") != std::string::npos)
                found_aeb = true;

        std::cout << "\nAEB-RRT* available: " << (found_aeb ? "YES" : "NO") << std::endl;
        rclcpp::shutdown();
        return found_aeb ? 0 : 1;
    }

    rclcpp::shutdown();
    return 1;
}
