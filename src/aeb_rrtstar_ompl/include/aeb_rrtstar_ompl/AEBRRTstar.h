/*********************************************************************
 * AEB-RRT* : Adaptive Extension Bidirectional RRT*
 *
 * This is a C++ OMPL planner plugin implementing the AEB-RRT*
 * algorithm for the ninezzhou 9-DOF robot arm.
 *
 * Two modes:
 *   stop_on_first_solution=true  → Faithful (paper default)
 *   stop_on_first_solution=false → Anytime (keep improving)
 *********************************************************************/

#ifndef AEB_RRTSTAR_OMPL_AEBRRTSTAR_H_
#define AEB_RRTSTAR_OMPL_AEBRRTSTAR_H_

#include <ompl/base/Planner.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include <vector>
#include <list>
#include <memory>
#include <limits>
#include <cmath>

namespace ompl
{
namespace geometric
{

/** \brief Adaptive Extension Bidirectional RRT* motion planner.

    Implements bidirectional RRT* with:
    - Adaptive target-biased sampling (controlled by collision failures)
    - Manhattan distance for nearest-neighbour queries
    - RRT* choose-parent and rewiring
    - Two-tree connection with configurable threshold

    Parameters (all settable via OMPL's param mechanism):
    - range / step_size
    - biased_range_multiplier
    - connect_threshold
    - p_min / p_max (adaptive probability bounds)
    - max_failed_extensions
    - rewire_factor
    - stop_on_first_solution
    - enable_aeb_shortcut (post-processing)
*/
class AEBRRTstar : public base::Planner
{
public:
    /** \brief Constructor */
    AEBRRTstar(const base::SpaceInformationPtr &si);

    /** \brief Destructor */
    ~AEBRRTstar() override;

    // --- OMPL Planner interface ---
    void clear() override;
    void setup() override;
    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
    void getPlannerData(base::PlannerData &data) const override;

    // --- Parameter setters/getters ---
    void setStepSize(double step) { step_size_ = step; }
    double getStepSize() const { return step_size_; }

    void setConnectThreshold(double thresh) { connect_threshold_ = thresh; }
    double getConnectThreshold() const { return connect_threshold_; }

    void setPMin(double p) { p_min_ = p; }
    double getPMin() const { return p_min_; }

    void setPMax(double p) { p_max_ = p; }
    double getPMax() const { return p_max_; }

    void setBiasedRangeMultiplier(double m) { biased_range_multiplier_ = m; }
    double getBiasedRangeMultiplier() const { return biased_range_multiplier_; }

    void setStopOnFirstSolution(bool stop) { stop_on_first_solution_ = stop; }
    bool getStopOnFirstSolution() const { return stop_on_first_solution_; }

    void setRewireFactor(double f) { rewire_factor_ = f; }
    double getRewireFactor() const { return rewire_factor_; }

    void setMaxFailedExtensions(int m) { max_failed_extensions_ = m; }
    int getMaxFailedExtensions() const { return max_failed_extensions_; }

    void setEnableAEBShortcut(bool enable) { enable_aeb_shortcut_ = enable; }
    bool getEnableAEBShortcut() const { return enable_aeb_shortcut_; }

    void setInterpCount(int n) { interp_count_ = n; }
    int getInterpCount() const { return interp_count_; }

    // --- Metrics (read after solve) ---
    unsigned int getLastMotionChecks() const { return motion_checks_; }

protected:
    // --- Tree node ---
    struct Node
    {
        base::State *state;
        int parent;                    // index, -1 for root
        std::vector<int> children;
        double cost;                   // accumulated cost from root
    };

    // --- Tree operations ---
    struct Tree
    {
        std::vector<Node> nodes;
        std::shared_ptr<NearestNeighbors<Node*>> nn;
    };

    /** \brief Perform one extension of *tree* toward *other_tree*.
     *  Returns (new_fail_count, connected). */
    std::pair<int, bool> extendTree(Tree &tree, Tree &other_tree,
                                     int fail_count, int iteration);

    /** \brief Build a PathGeometric from the two connected trees. */
    base::PathPtr buildPath() const;

    /** \brief Compute Manhattan (L1) distance between two states. */
    double manhattanDistance(const base::State *a, const base::State *b) const;

    /** \brief Compute Euclidean distance between two states. */
    double euclideanDistance(const base::State *a, const base::State *b) const;

    /** \brief Steer from a toward b by at most step. */
    void steer(const base::State *from, const base::State *to,
               double step, base::State *result) const;

    /** \brief Random sample in the state space. */
    void randomSample(base::State *state) const;

    /** \brief RRT* neighbourhood radius. */
    double neighbourhoodRadius(int n) const;

    /** \brief Propagate cost delta through descendants. */
    void updateDescendantCosts(Tree &tree, int changed_idx, double delta);

    /** \brief Trace path from root to node idx. */
    std::vector<base::State*> tracePath(const Tree &tree, int idx) const;

    /** \brief Compute full path cost. */
    double computePathCost(const base::PathPtr &path) const;

    /** \brief AEB paper shortcut (interpolate + farthest-visible). */
    base::PathPtr aebShortcut(const base::PathPtr &path);

    // --- State space helpers ---
    unsigned int dimension_;
    const base::StateSpacePtr state_space_;

    // --- Trees ---
    Tree tree_start_;
    Tree tree_goal_;

    // --- Connection bookkeeping ---
    int connect_start_idx_;
    int connect_goal_idx_;

    // --- Best solution (anytime mode) ---
    double best_cost_;
    base::PathPtr best_path_;

    // --- Metrics ---
    unsigned int motion_checks_;

    // --- Parameters (with defaults) ---
    double step_size_{0.0};           // 0 = auto-compute in setup()
    double connect_threshold_{0.0};    // 0 = auto-compute in setup()
    double p_min_{0.1};
    double p_max_{1.0};
    int max_failed_extensions_{500};
    double rewire_factor_{1.1};
    bool stop_on_first_solution_{true};
    bool enable_aeb_shortcut_{true};
    double biased_range_multiplier_{2.0};
    int interp_count_{30};

    // --- Random number generator (from OMPL) ---
    ompl::RNG rng_;

    // --- State sampler (cached, RRTConnect-style via si_->allocStateSampler()) ---
    mutable ompl::base::StateSamplerPtr sampler_;
};

}  // namespace geometric
}  // namespace ompl

#endif  // AEB_RRTSTAR_OMPL_AEBRRTSTAR_H_
