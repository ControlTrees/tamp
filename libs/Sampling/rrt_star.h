#pragma once

#include <array>
#include <cstdlib>
#include <list>
#include <deque>
#include <functional>
#include <cassert>
#include <nearest_neighbor.h>
#include <sample_space.h>
#include <rrt_common.h>

template <std::size_t N>
struct RRTStarNode
{
  std::weak_ptr<RRTStarNode<N>> parent;
  std::array<double, N> state;
  std::list<std::shared_ptr<RRTStarNode<N>>> children;
  double cost{0}; // form root
  uint id{0};
};

template<uint N>
class RRTStarTree
{
public:
  RRTStarTree(const std::array<double, N> & root_state)
  {
    root_ = std::make_shared<RRTStarNode<N>>();
    root_->state = root_state;
    nodes_.push_back(root_);
  }

  std::shared_ptr<RRTStarNode<N>> get_node(uint id) const
  {
    return nodes_[id];
  }

  std::shared_ptr<RRTStarNode<N>> add_node(const std::array<double, N> & state)
  {
    auto node = std::make_shared<RRTStarNode<N>>();
    node->state = state;
    node->id = nodes_.size();
    nodes_.push_back(node);

    return node;
  }

  void add_edge(const std::shared_ptr<RRTStarNode<N>> & from, const std::shared_ptr<RRTStarNode<N>> & to) const
  {
    from->children.push_back(to);
    to->parent = from;
    to->cost = from->cost + norm2(from->state, to->state);
  }

  const std::vector<std::shared_ptr<RRTStarNode<N>>> & nodes() const
  {
    return nodes_;
  }

private:
  std::vector<std::shared_ptr<RRTStarNode<N>>> nodes_;
  std::shared_ptr<RRTStarNode<N>> root_;
};

template<typename S> // sample space
class RRTStar
{
public:
  RRTStar(const S & space, double max_step=1.0, double radius=1.0)
    : space_(space)
    , max_step_(max_step) // max tree expansion
    , radius_(radius)
  {

  }

  void set_state_checker(const std::function<bool(const std::array<double, S::dim> &)> & state_checker)
  {
    state_checker_ = state_checker;
  }

  void set_transition_checker(const std::function<bool(const std::array<double, S::dim> &, const std::array<double, S::dim> &)> & transition_checker)
  {
    transition_checker_ = transition_checker;
  }

  std::deque<std::array<double, S::dim>> get_path_to(const std::shared_ptr<RRTStarNode<S::dim>> & to)
  {
    std::deque<std::array<double, S::dim>> path;

    auto current = to;
    path.push_front(to->state);
    auto parent = current->parent.lock();

    while(parent)
    {
      path.push_front(parent->state);
      parent = parent->parent.lock();
    }

    return path;
  }

  std::deque<std::array<double, S::dim>> plan(const std::array<double, S::dim> & start,
                                              const std::function<bool(const std::array<double, S::dim> &)> & goal_cnd,
                                              uint n_iter_max)
  {
    assert(state_checker_);
    assert(transition_checker_);

    // grow tree
    rrttree_ = std::make_shared<RRTStarTree<S::dim>>(start);
    kdtree_ = std::make_unique<KDTree<S::dim>>(start);

    for(uint i = 0; i < n_iter_max; ++i)
    {
      auto s = space_.sample();
      const auto& node = kdtree_->nearest_neighbor(s);

      backtrack(node->state, s, max_step_);

      if(state_checker_(s))
      {  
        // get other neighbors
        auto neighbors = kdtree_->radius_neighbors(s, radius_);
        if(neighbors.size() > 0)
        {
          //assert(neighbors.front()->state[0] == node->state[0]);
          neighbors.pop_front(); // front must be equal to node per defition, we don't need to consider it
        }

        // choose neighbor leading to best cost
        auto best_from = rrttree_->get_node(node->id);
        double best_cost = best_from->cost + norm2(best_from->state, s);

        for(const auto& neighbor: neighbors)
        {
          const auto & from = rrttree_->get_node(neighbor->id);
          const double cost = from->cost + norm2(from->state, s);

          if(cost < best_cost)
          {
            best_from = from;
            best_cost = cost;
          }
        }

        if(transition_checker_(best_from->state, s))
        {
          // commit edge
          auto new_node = rrttree_->add_node(s);

          kdtree_->add_node(new_node->state, new_node->id);
          rrttree_->add_edge(best_from, new_node);

          if(goal_cnd(new_node->state))
          {
            // found solution
            final_nodes_.push_back(new_node);
          }

          // rewire
          for(const auto& neighbor: neighbors)
          {
            auto to = rrttree_->get_node(neighbor->id);
            if(best_cost + norm2(s, to->state) < to->cost)
            {
              to->parent = new_node;
              new_node->children.push_back(to);
            }
          }
        }
      }
    }

    // extract solutions
    std::vector<std::deque<std::array<double, S::dim>>> paths;
    std::vector<double> costs;
    paths.reserve(final_nodes_.size());
    costs.reserve(final_nodes_.size());

    if(!final_nodes_.empty())
    {
      const auto path = get_path_to(final_nodes_.back());
      const auto cost = get_cost(path);

      paths.push_back(path);
      costs.push_back(cost);
    }

    // return best
    uint best = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for(uint i = 0; i < paths.size(); ++i)
    {
      if(costs[i] < best_cost)
      {
        best = i;
        best_cost = costs[i];
      }
    }

    return final_nodes_.size() > 0 ? paths[best] : std::deque<std::array<double, S::dim>>();
  }

  std::shared_ptr<RRTStarTree<S::dim>> rrt_tree() const
  {
    return rrttree_;
  }

private:
  const S & space_;
  const double max_step_;
  const double radius_;
  std::shared_ptr<RRTStarTree<S::dim>> rrttree_;
  std::unique_ptr<KDTree<S::dim>> kdtree_;
  std::function<bool(const std::array<double, S::dim> &)> state_checker_;
  std::function<bool(const std::array<double, S::dim> &, const std::array<double, S::dim> &)> transition_checker_;
  std::list<std::shared_ptr<RRTStarNode<S::dim>>> final_nodes_;
};
