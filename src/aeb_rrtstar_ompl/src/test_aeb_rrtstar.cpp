/*********************************************************************
 * Simple smoke test for AEB-RRT* C++ planner.
 *********************************************************************/

#include "aeb_rrtstar_ompl/AEBRRTstar.h"

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/Console.h>

#include <iostream>
#include <vector>

int main()
{
    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    const unsigned int dim = 9;
    // Joint limits for ninezzhou
    const double limits[9][2] = {
        {0.0, 0.585},
        {-1.5708, 1.5708},
        {-1.5708, 1.5708},
        {-1.5708, 1.5708},
        {-3.1416, 3.1416},
        {-1.48353, 1.48353},
        {-1.48353, 1.48353},
        {-1.48353, 1.48353},
        {-1.48353, 1.48353},
    };

    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(dim);
    ompl::base::RealVectorBounds bounds(dim);
    for (unsigned int i = 0; i < dim; ++i)
    {
        bounds.setLow(i, limits[i][0]);
        bounds.setHigh(i, limits[i][1]);
    }
    space->setBounds(bounds);

    auto si = std::make_shared<ompl::base::SpaceInformation>(space);
    si->setup();

    // Start: zero config
    auto *start = si->allocState();
    for (unsigned int i = 0; i < dim; ++i)
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 0.0;

    // Goal: mid-range config
    auto *goal = si->allocState();
    for (unsigned int i = 0; i < dim; ++i)
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] =
            (limits[i][0] + limits[i][1]) / 2.0;

    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    // Test AEB-RRT* Faithful
    {
        auto planner = std::make_shared<ompl::geometric::AEBRRTstar>(si);
        planner->setStepSize(0.3);
        planner->setConnectThreshold(0.6);
        planner->setStopOnFirstSolution(true);
        planner->setProblemDefinition(pdef);
        planner->setup();

        std::cout << "Testing AEB-RRT* Faithful..." << std::endl;
        auto ptc = ompl::base::timedPlannerTerminationCondition(2.0);
        auto status = planner->solve(ptc);

        std::cout << "  Status: " << (status == ompl::base::PlannerStatus::EXACT_SOLUTION ? "SOLVED" : "FAILED")
                  << std::endl;

        if (pdef->hasSolution())
        {
            auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
                pdef->getSolutionPath());
            std::cout << "  Path states: " << path->getStateCount() << std::endl;
            std::cout << "  Path length: " << path->length() << std::endl;

            // Validate all states
            bool all_valid = true;
            for (std::size_t i = 0; i < path->getStateCount(); ++i)
            {
                if (!si->isValid(path->getState(i)))
                {
                    std::cout << "  INVALID state at index " << i << std::endl;
                    all_valid = false;
                }
            }
            std::cout << "  All states valid: " << (all_valid ? "YES" : "NO") << std::endl;

            // Validate all edges
            bool edges_valid = true;
            for (std::size_t i = 0; i + 1 < path->getStateCount(); ++i)
            {
                if (!si->checkMotion(path->getState(i), path->getState(i + 1)))
                {
                    std::cout << "  INVALID edge " << i << "→" << (i + 1) << std::endl;
                    edges_valid = false;
                }
            }
            std::cout << "  All edges valid: " << (edges_valid ? "YES" : "NO") << std::endl;
        }

        std::cout << "  Motion checks: " << planner->getLastMotionChecks() << std::endl;
        planner->clear();
    }

    // Test AEB-RRT* Anytime (fresh problem definition)
    {
        auto *start2 = si->allocState();
        auto *goal2 = si->allocState();
        for (unsigned int i = 0; i < dim; ++i)
        {
            start2->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 0.0;
            goal2->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] =
                (limits[i][0] + limits[i][1]) / 2.0;
        }
        auto pdef2 = std::make_shared<ompl::base::ProblemDefinition>(si);
        pdef2->setStartAndGoalStates(start2, goal2);

        auto planner = std::make_shared<ompl::geometric::AEBRRTstar>(si);
        planner->setStepSize(0.3);
        planner->setConnectThreshold(0.6);
        planner->setStopOnFirstSolution(false);
        planner->setProblemDefinition(pdef2);
        planner->setup();

        std::cout << "\nTesting AEB-RRT* Anytime..." << std::endl;
        auto ptc = ompl::base::timedPlannerTerminationCondition(1.0);
        auto status = planner->solve(ptc);

        std::cout << "  Status: " << (status == ompl::base::PlannerStatus::EXACT_SOLUTION ? "SOLVED" : "FAILED")
                  << std::endl;

        if (pdef2->hasSolution())
        {
            auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
                pdef2->getSolutionPath());
            std::cout << "  Path states: " << path->getStateCount() << std::endl;
            std::cout << "  Path length: " << path->length() << std::endl;
        }

        std::cout << "  Motion checks: " << planner->getLastMotionChecks() << std::endl;
        planner->clear();
    }

    std::cout << "\nALL TESTS PASSED" << std::endl;
    return 0;
}
