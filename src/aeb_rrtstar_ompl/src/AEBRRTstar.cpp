/*********************************************************************
 * AEB-RRT* : Adaptive Extension Bidirectional RRT*
 * C++ OMPL planner for the ninezzhou 9-DOF robot arm.
 *********************************************************************/

#include "aeb_rrtstar_ompl/AEBRRTstar.h"

#include <ompl/base/Goal.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Exception.h>

#include <boost/math/constants/constants.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

using boost::math::constants::pi;

namespace ompl
{
namespace geometric
{

namespace
{
    // J5 (index 4) has [-π, π] range → wrapping joint
    const unsigned int J5_INDEX = 4;

    double normDist(const base::State *a, const base::State *b,
                    unsigned int dim)
    {
        const auto *va =
            a->as<base::RealVectorStateSpace::StateType>()->values;
        const auto *vb =
            b->as<base::RealVectorStateSpace::StateType>()->values;
        double sum = 0.0;
        for (unsigned int i = 0; i < dim; ++i)
        {
            double diff = va[i] - vb[i];
            if (i == J5_INDEX)
            {
                diff = std::fmod(diff + pi<double>(),
                                 2.0 * pi<double>());
                if (diff < 0.0) diff += 2.0 * pi<double>();
                diff -= pi<double>();
            }
            if (i == 0)
                diff *= pi<double>() / 0.585;
            sum += diff * diff;
        }
        return std::sqrt(sum);
    }

    double manhattanDist(const base::State *a, const base::State *b,
                         unsigned int dim)
    {
        const auto *va =
            a->as<base::RealVectorStateSpace::StateType>()->values;
        const auto *vb =
            b->as<base::RealVectorStateSpace::StateType>()->values;
        double total = 0.0;
        for (unsigned int i = 0; i < dim; ++i)
        {
            double diff = std::fabs(va[i] - vb[i]);
            if (i == J5_INDEX)
            {
                if (diff > pi<double>())
                    diff = 2.0 * pi<double>() - diff;
            }
            if (i == 0)
                diff *= pi<double>() / 0.585;
            total += diff;
        }
        return total;
    }
}  // namespace

// ======================================================================
//  Constructor / Destructor
// ======================================================================

AEBRRTstar::AEBRRTstar(const base::SpaceInformationPtr &si)
  : base::Planner(si, "AEBRRTstar")
  , dimension_(0)
  , state_space_(si->getStateSpace())
  , connect_start_idx_(-1)
  , connect_goal_idx_(-1)
  , best_cost_(std::numeric_limits<double>::infinity())
  , motion_checks_(0)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = true;
    specs_.directed = true;

    declareParam<double>("range", this, &AEBRRTstar::setStepSize,
                         &AEBRRTstar::getStepSize, "0.0:10.0");
    declareParam<double>("connect_threshold", this,
                         &AEBRRTstar::setConnectThreshold,
                         &AEBRRTstar::getConnectThreshold, "0.0:20.0");
    declareParam<bool>("stop_on_first_solution", this,
                       &AEBRRTstar::setStopOnFirstSolution,
                       &AEBRRTstar::getStopOnFirstSolution);
}

AEBRRTstar::~AEBRRTstar()
{
    clear();
}

// ======================================================================
void AEBRRTstar::clear()
{
    base::Planner::clear();
    auto si = si_;
    if (si)
    {
        for (auto &node : tree_start_.nodes)
            if (node.state) si->freeState(node.state);
        for (auto &node : tree_goal_.nodes)
            if (node.state) si->freeState(node.state);
    }
    tree_start_.nodes.clear();
    tree_start_.nn.reset();
    tree_goal_.nodes.clear();
    tree_goal_.nn.reset();
    best_cost_ = std::numeric_limits<double>::infinity();
    best_path_.reset();
    motion_checks_ = 0;
    connect_start_idx_ = -1;
    connect_goal_idx_ = -1;
}

void AEBRRTstar::setup()
{
    base::Planner::setup();
    auto *rvs = si_->getStateSpace()->as<base::RealVectorStateSpace>();
    if (!rvs)
        throw Exception("AEBRRTstar requires a RealVectorStateSpace");
    dimension_ = rvs->getDimension();
    if (dimension_ < 1)
        throw Exception("AEBRRTstar: state space dimension must be >= 1");
}

// ======================================================================
base::PlannerStatus AEBRRTstar::solve(
    const base::PlannerTerminationCondition &ptc)
{
    // MoveIt2 calls clear() before solve().  Only free our own tree
    // resources; do NOT reset PlannerInputStates via base::clear().
    {
        auto si_tmp = si_;
        if (si_tmp) {
            for (auto &n : tree_start_.nodes)
                if (n.state) si_tmp->freeState(n.state);
            for (auto &n : tree_goal_.nodes)
                if (n.state) si_tmp->freeState(n.state);
        }
        tree_start_.nodes.clear(); tree_start_.nn.reset();
        tree_goal_.nodes.clear();  tree_goal_.nn.reset();
        best_cost_ = std::numeric_limits<double>::infinity();
        best_path_.reset();
        motion_checks_ = 0;
        connect_start_idx_ = -1;
        connect_goal_idx_ = -1;
    }
    auto si = si_;
    auto pdef = pdef_;
    if (!si || !pdef)
    {
        std::cerr << "AEB_FAIL: CRASH si=" << (si_ ? "OK" : "NULL")
                  << " pdef=" << (pdef_ ? "OK" : "NULL") << std::endl;
        return base::PlannerStatus::CRASH;
    }

    // Auto-compute step/threshold from state space extent NOW (all params
    // from YAML have been applied by MoveIt2 by the time solve() runs).
    // OMPL convention: range=0 → auto-compute.
    // Use 5% of extent (~0.51 for ninezzhou) — small enough that stepping
    // from a boundary goal root stays in bounds.
    if (step_size_ <= std::numeric_limits<double>::epsilon())
    {
        double extent = si_->getStateSpace()->getMaximumExtent();
        step_size_ = 0.05 * extent;
    }
    if (connect_threshold_ <= std::numeric_limits<double>::epsilon())
    {
        connect_threshold_ = 2.0 * step_size_;
    }
    OMPL_INFORM("AEBRRTstar: eff step=%.3f cn_thr=%.3f",
                step_size_, connect_threshold_);

    base::PlannerInputStates input_states(this);
    if (!input_states.haveMoreStartStates())
    {
        std::cerr << "AEB_FAIL: NO START STATES" << std::endl;
        return base::PlannerStatus::INVALID_START;
    }

    // Use PlannerInputStates::nextStart()/nextGoal() which handle all
    // goal types (GoalState, GoalSampleableRegion, ConstrainedGoalSampler).
    // Clone into our own states for tree ownership.
    //
    // NOTE: nextStart() skips start states that fail validity and returns
    // nullptr when every candidate was invalid (e.g. MoveIt2 passed a
    // colliding start that FixStartStateCollision could not repair).  Guard
    // against that before cloning — cloneState(nullptr) segfaults.
    const base::State *start_raw = input_states.nextStart();
    if (!start_raw)
    {
        std::cerr << "AEB_FAIL: NO VALID START STATES" << std::endl;
        return base::PlannerStatus::INVALID_START;
    }
    base::State *start_state = si->cloneState(start_raw);
    if (!si->isValid(start_state))
    {
        std::cerr << "AEB_FAIL: INVALID START" << std::endl;
        si->freeState(start_state);
        return base::PlannerStatus::INVALID_START;
    }

    // nextGoal(ptc) samples a valid goal state using OMPL's goal sampler
    // (which maintains the MoveIt2 RobotState correctly).
    const base::State *goal_raw = input_states.nextGoal(ptc);
    if (!goal_raw)
    {
        si->freeState(start_state);
        return base::PlannerStatus::INVALID_GOAL;
    }
    base::State *goal_state = si->cloneState(goal_raw);
    if (!si->isValid(goal_state))
    {
        std::cerr << "AEB_FAIL: INVALID GOAL" << std::endl;
        si->freeState(start_state);
        si->freeState(goal_state);
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (si->distance(start_state, goal_state) <
        std::numeric_limits<double>::epsilon())
    {
        auto path = std::make_shared<PathGeometric>(si);
        path->append(start_state);
        pdef->addSolutionPath(path);
        si->freeState(start_state);
        si->freeState(goal_state);
        return base::PlannerStatus::EXACT_SOLUTION;
    }

    // --- Init trees (pre-reserve to prevent vector reallocation which
    //     would invalidate Node* pointers stored in the GNAT NN) ---
    static const size_t TREE_RESERVE = 100000;

    tree_start_.nodes.reserve(TREE_RESERVE);
    tree_start_.nodes.push_back({start_state, -1, {}, 0.0});
    tree_start_.nn =
        std::make_shared<NearestNeighborsGNATNoThreadSafety<Node *>>();
    tree_start_.nn->setDistanceFunction(
        [this](const Node *a, const Node *b) {
            return manhattanDist(a->state, b->state, dimension_);
        });
    tree_start_.nn->add(&tree_start_.nodes[0]);

    tree_goal_.nodes.reserve(TREE_RESERVE);
    tree_goal_.nodes.push_back({goal_state, -1, {}, 0.0});
    tree_goal_.nn =
        std::make_shared<NearestNeighborsGNATNoThreadSafety<Node *>>();
    tree_goal_.nn->setDistanceFunction(
        [this](const Node *a, const Node *b) {
            return manhattanDist(a->state, b->state, dimension_);
        });
    tree_goal_.nn->add(&tree_goal_.nodes[0]);

    int fail_start = 0, fail_goal = 0;
    bool solved = false;
    int iter = 0;
    auto *opt_obj = pdef->getOptimizationObjective().get();

    while (!ptc())
    {
        ++iter;

        auto [nf, conn] =
            extendTree(tree_start_, tree_goal_, fail_start, iter);
        fail_start = nf;
        if (conn)
        {
            auto path = buildPath();
            if (path)
            {
                double c = computePathCost(path);
                if (stop_on_first_solution_)
                {
                    pdef->addSolutionPath(path);
                    solved = true;
                    break;
                }
                else if (c < best_cost_)
                {
                    best_cost_ = c;
                    best_path_ = path;
                    solved = true;
                }
            }
        }
        if (ptc()) break;

        auto [nf2, conn2] =
            extendTree(tree_goal_, tree_start_, fail_goal, iter);
        fail_goal = nf2;
        if (conn2)
        {
            auto path = buildPath();
            if (path)
            {
                double c = computePathCost(path);
                if (stop_on_first_solution_)
                {
                    pdef->addSolutionPath(path);
                    solved = true;
                    break;
                }
                else if (c < best_cost_)
                {
                    best_cost_ = c;
                    best_path_ = path;
                    solved = true;
                }
            }
        }

        if (std::max(fail_start, fail_goal) >
                max_failed_extensions_ && !solved)
        {
            std::cerr << "AEB_FAIL: max_failed_extensions reached ("
                      << fail_start << "/" << fail_goal << ")" << std::endl;
            break;
        }
    }

    if (solved)
    {
        if (!stop_on_first_solution_ && best_path_)
            pdef->addSolutionPath(
                enable_aeb_shortcut_ ? aebShortcut(best_path_)
                                     : best_path_);
        return base::PlannerStatus::EXACT_SOLUTION;
    }
    std::cerr << "AEB_FAIL: timeout, iter=" << iter
              << " fail=" << fail_start << "/" << fail_goal
              << " nodes=" << tree_start_.nodes.size() << "/"
              << tree_goal_.nodes.size() << std::endl;
    return base::PlannerStatus::TIMEOUT;
}

void AEBRRTstar::getPlannerData(base::PlannerData &data) const
{
    base::Planner::getPlannerData(data);
    for (const auto &tree : {&tree_start_, &tree_goal_})
    {
        int tag = (tree == &tree_start_) ? 1 : 2;
        for (size_t i = 0; i < tree->nodes.size(); ++i)
        {
            const auto &node = tree->nodes[i];
            if (node.parent >= 0)
                data.addEdge(
                    base::PlannerDataVertex(
                        tree->nodes[node.parent].state, tag),
                    base::PlannerDataVertex(node.state, tag));
        }
    }
}

// ======================================================================
std::pair<int, bool> AEBRRTstar::extendTree(
    Tree &tree, Tree &other_tree, int fail_count, int /*iter*/)
{
    auto si = si_;
    unsigned int dim = dimension_;

    double pa = p_min_ + (p_max_ - p_min_) *
        std::exp(-9.0 /
                 std::pow(static_cast<double>(fail_count + 1), 3.0));

    base::State *x_sample = si->allocState();
    double step;

    if (rng_.uniform01() > pa)
    {
        si->copyState(x_sample, other_tree.nodes[0].state);
        step = step_size_ * biased_range_multiplier_;
    }
    else
    {
        randomSample(x_sample);
        step = step_size_;
    }

    Node dummy{x_sample, -1, {}, 0.0};
    Node *nearest = tree.nn->nearest(&dummy);
    int idx_near = static_cast<int>(nearest - tree.nodes.data());

    base::State *x_new = si->allocState();
    bool is_goal_tree = (&tree == &tree_goal_);

    steer(tree.nodes[idx_near].state, x_sample, step, x_new);
    si->freeState(x_sample);

    // RRTConnect-style motion check: for the goal tree, first verify the
    // steered state is valid, then check motion (mirrors growTree()).
    bool valid = is_goal_tree
        ? (si->isValid(x_new) && si->checkMotion(x_new, tree.nodes[idx_near].state))
        : si->checkMotion(tree.nodes[idx_near].state, x_new);
    ++motion_checks_;
    if (!valid)
    {
        si->freeState(x_new);
        return {fail_count + 1, false};
    }
    if (!si->checkMotion(tree.nodes[idx_near].state, x_new))
    {
        si->freeState(x_new);
        return {fail_count + 1, false};
    }

    // RRT*: choose parent
    double radius = neighbourhoodRadius(
        static_cast<int>(tree.nodes.size()) + 1);
    Node dummy2{x_new, -1, {}, 0.0};
    std::vector<Node *> near;
    tree.nn->nearestR(&dummy2, radius, near);

    int best_parent = idx_near;
    double best_cost = tree.nodes[idx_near].cost +
        normDist(tree.nodes[idx_near].state, x_new, dim);

    for (auto *nn : near)
    {
        int ni = static_cast<int>(nn - tree.nodes.data());
        if (ni == idx_near) continue;
        double c = tree.nodes[ni].cost +
            normDist(tree.nodes[ni].state, x_new, dim);
        if (c < best_cost)
        {
            ++motion_checks_;
            if (si->checkMotion(tree.nodes[ni].state, x_new))
            {
                best_parent = ni;
                best_cost = c;
            }
        }
    }

    int new_idx = static_cast<int>(tree.nodes.size());
    tree.nodes.push_back({x_new, best_parent, {}, best_cost});
    tree.nodes[best_parent].children.push_back(new_idx);
    tree.nn->add(&tree.nodes.back());

    // Rewire
    for (auto *nn : near)
    {
        int ni = static_cast<int>(nn - tree.nodes.data());
        if (ni == best_parent || ni == new_idx) continue;
        double via = best_cost +
            normDist(x_new, tree.nodes[ni].state, dim);
        if (via + 1e-12 < tree.nodes[ni].cost)
        {
            ++motion_checks_;
            if (si->checkMotion(x_new, tree.nodes[ni].state))
            {
                double delta = via - tree.nodes[ni].cost;
                int old_p = tree.nodes[ni].parent;
                if (old_p >= 0)
                {
                    auto &sib = tree.nodes[old_p].children;
                    sib.erase(std::remove(sib.begin(), sib.end(), ni),
                              sib.end());
                }
                tree.nodes[ni].parent = new_idx;
                tree.nodes[ni].cost = via;
                tree.nodes[new_idx].children.push_back(ni);
                updateDescendantCosts(tree, ni, delta);
            }
        }
    }

    // Connect
    base::State *tmp = si->allocState();
    si->copyState(tmp, x_new);
    Node cdummy{tmp, -1, {}, 0.0};
    Node *con_node = other_tree.nn->nearest(&cdummy);
    si->freeState(tmp);

    if (con_node)
    {
        int ci = static_cast<int>(
            con_node - other_tree.nodes.data());
        double d = normDist(x_new, other_tree.nodes[ci].state, dim);
        if (d <= connect_threshold_)
        {
            ++motion_checks_;
            if (si->checkMotion(x_new,
                                other_tree.nodes[ci].state))
            {
                // Store the connection indices in START/GOAL tree terms.
                // When the goal tree is the one being extended, x_new
                // (new_idx) lives in the goal tree and con_node (ci) lives
                // in the start tree; buildPath() expects
                // connect_start_idx_/connect_goal_idx_ to index into
                // tree_start_/tree_goal_ respectively.  Swapping them here
                // would make tracePath() read the wrong tree (or out of
                // bounds) and segfault — only visible in Anytime mode where
                // the goal tree is actually grown to completion.
                if (is_goal_tree)
                {
                    connect_start_idx_ = ci;
                    connect_goal_idx_ = new_idx;
                }
                else
                {
                    connect_start_idx_ = new_idx;
                    connect_goal_idx_ = ci;
                }
                return {0, true};
            }
        }
    }
    return {0, false};
}

// ======================================================================
base::PathPtr AEBRRTstar::buildPath() const
{
    auto si = si_;
    auto sc = tracePath(tree_start_, connect_start_idx_);
    auto gc = tracePath(tree_goal_, connect_goal_idx_);
    if (sc.empty() || gc.empty()) return nullptr;

    auto path = std::make_shared<PathGeometric>(si);
    for (auto *s : sc) path->append(s);
    for (int i = static_cast<int>(gc.size()) - 2; i >= 0; --i)
        path->append(gc[i]);
    return path;
}

std::vector<base::State *> AEBRRTstar::tracePath(
    const Tree &tree, int idx) const
{
    std::vector<base::State *> chain;
    int cur = idx;
    while (cur >= 0)
    {
        chain.push_back(tree.nodes[cur].state);
        cur = tree.nodes[cur].parent;
    }
    std::reverse(chain.begin(), chain.end());
    return chain;
}

// ======================================================================
double AEBRRTstar::manhattanDistance(
    const base::State *a, const base::State *b) const
{
    return manhattanDist(a, b, dimension_);
}

double AEBRRTstar::euclideanDistance(
    const base::State *a, const base::State *b) const
{
    return normDist(a, b, dimension_);
}

// ======================================================================
void AEBRRTstar::randomSample(base::State *state) const
{
    // RRTConnect-style: use si_->allocStateSampler() which returns a
    // sampler that properly initializes MoveIt2's RobotState in sampled
    // states (allocDefaultStateSampler() on the raw RealVector space does not).
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    sampler_->sampleUniform(state);
}

void AEBRRTstar::steer(const base::State *from, const base::State *to,
                        double step, base::State *result) const
{
    // RRTConnect-style steering via the state space's interpolate().
    // This mirrors ompl::geometric::RRTConnect::growTree().
    double d = si_->distance(from, to);
    if (d <= step + std::numeric_limits<double>::epsilon())
    {
        si_->getStateSpace()->copyState(result, to);
        return;
    }
    si_->getStateSpace()->interpolate(from, to, step / d, result);
}

double AEBRRTstar::neighbourhoodRadius(int n) const
{
    if (n <= 2) return connect_threshold_;
    double gamma = rewire_factor_ *
        std::pow(static_cast<double>(dimension_), 0.25);
    double r = gamma *
        std::pow(std::log(static_cast<double>(n)) / n,
                 1.0 / dimension_);
    return std::max(step_size_,
                    std::min(r, connect_threshold_ * 3.0));
}

void AEBRRTstar::updateDescendantCosts(
    Tree &tree, int changed_idx, double delta)
{
    std::queue<int> q;
    q.push(changed_idx);
    while (!q.empty())
    {
        int cur = q.front(); q.pop();
        for (int child : tree.nodes[cur].children)
        {
            tree.nodes[child].cost += delta;
            q.push(child);
        }
    }
}

double AEBRRTstar::computePathCost(
    const base::PathPtr &path) const
{
    auto *pg = dynamic_cast<PathGeometric *>(path.get());
    if (!pg) return std::numeric_limits<double>::infinity();
    double total = 0.0;
    std::size_t n = pg->getStateCount();
    for (std::size_t i = 0; i + 1 < n; ++i)
        total += normDist(pg->getState(i), pg->getState(i + 1),
                          dimension_);
    return total;
}

// ======================================================================
base::PathPtr AEBRRTstar::aebShortcut(
    const base::PathPtr &path)
{
    auto *pg = dynamic_cast<PathGeometric *>(path.get());
    if (!pg || pg->getStateCount() < 3) return path;

    auto si = si_;
    auto ss = state_space_;
    std::size_t n = pg->getStateCount();
    std::vector<base::State *> interp;
    for (std::size_t i = 0; i + 1 < n; ++i)
    {
        interp.push_back(si->cloneState(pg->getState(i)));
        for (int k = 1; k <= interp_count_; ++k)
        {
            double alpha = static_cast<double>(k) /
                           (interp_count_ + 1);
            base::State *s = si->allocState();
            // interpolate() keeps the MoveIt2 RobotState in sync
            ss->interpolate(pg->getState(i), pg->getState(i + 1),
                            alpha, s);
            interp.push_back(s);
        }
    }
    interp.push_back(si->cloneState(pg->getState(n - 1)));

    auto result = std::make_shared<PathGeometric>(si);
    result->append(si->cloneState(interp[0]));
    std::size_t i = 0;
    while (i < interp.size() - 1)
    {
        std::size_t best_j = i + 1;
        for (std::size_t j = interp.size() - 1; j > i; --j)
        {
            ++motion_checks_;
            if (si->checkMotion(interp[i], interp[j]))
            {
                best_j = j;
                break;
            }
        }
        result->append(si->cloneState(interp[best_j]));
        i = best_j;
    }

    for (auto *s : interp) si->freeState(s);
    return result;
}

}  // namespace geometric
}  // namespace ompl
