#pragma once

#include <array>
#include <cstdlib>
#include <list>
#include <deque>
#include <functional>
#include <cassert>
#include <nearest_neighbor.h>

inline double sample_01()
{
  return double(rand()) / RAND_MAX;
}

template <uint N>
class SampleSpace
{
public:
  SampleSpace(const std::array<std::pair<double, double>, N> & bounds)
    : bounds_(bounds)
  {

  }

  double sample_1d(uint i) const
  {
    return bounds_[i].first + sample_01() * (bounds_[i].second - bounds_[i].first);
  }

  std::array<double, N> sample() const
  {
    std::array<double, N> s;

    for(uint i = 0; i < N; ++i)
      s[i] = sample_1d(i);

    return s;
  }

  std::array<std::pair<double, double>, N> bounds() const { return bounds_; }

public:
  static constexpr uint dim = N;

private:
  std::array<std::pair<double, double>, N> bounds_;
};

template <std::size_t N>
struct RRTTreeNode
{
  std::weak_ptr<RRTTreeNode<N>> parent;
  std::array<double, N> state;
  std::list<std::shared_ptr<RRTTreeNode<N>>> children;
  uint id;
};

template<uint N>
class RRTTree
{
public:
  RRTTree(const std::array<double, N> & root_state)
  {
    root_ = std::make_shared<RRTTreeNode<N>>();
    root_->state = root_state;
    nodes_.push_back(root_);
  }

  std::shared_ptr<RRTTreeNode<N>> get_node(uint id) const
  {
    return nodes_[id];
  }

  std::shared_ptr<RRTTreeNode<N>> add_node(const std::array<double, N> & state)
  {
    auto node = std::make_shared<RRTTreeNode<N>>();
    node->state = state;
    node->id = nodes_.size();
    nodes_.push_back(node);

    return node;
  }

  void add_edge(const std::shared_ptr<RRTTreeNode<N>> & from, const std::shared_ptr<RRTTreeNode<N>> & to) const
  {
    from->children.push_back(to);
    to->parent = from;
  }

  const std::vector<std::shared_ptr<RRTTreeNode<N>>> & nodes() const
  {
    return nodes_;
  }

private:
  std::vector<std::shared_ptr<RRTTreeNode<N>>> nodes_;
  std::shared_ptr<RRTTreeNode<N>> root_;
};

template<typename S> // sample space
class RRT
{
public:
  RRT(const S & space, double max_step=1.0)
    : space_(space)
    , max_step_(max_step) // max tree expansion
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

  std::deque<std::array<double, S::dim>> get_path_to(const std::shared_ptr<RRTTreeNode<S::dim>> & to)
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

  double get_cost(const std::deque<std::array<double, S::dim>> & path) const
  {
    double cost = 0;

    for(uint i = 1; i < path.size(); ++i)
    {
      cost += norm2(path[i-1], path[i]);
    }

    return cost;
  }

  std::deque<std::array<double, S::dim>> plan(const std::array<double, S::dim> & start,
                                              const std::function<bool(const std::array<double, S::dim> &)> & goal_cnd,
                                              uint n_iter_max)
  {
    assert(state_checker_);
    assert(transition_checker_);

    // grow tree
    rrttree_ = std::make_shared<RRTTree<S::dim>>(start);
    kdtree_ = std::make_unique<KDTree<S::dim>>(start);

    for(uint i = 0; i < n_iter_max; ++i)
    {
      auto s = space_.sample();
      auto node = kdtree_->nearest_neighbor(s);

      backtrack(node->state, s);

      if(state_checker_(s))
      {  
        if(transition_checker_(node->state, s))
        {
          auto from = rrttree_->get_node(node->id);
          auto to = rrttree_->add_node(s);

          kdtree_->add_node(to->state, to->id);
          rrttree_->add_edge(from, to);

          if(goal_cnd(to->state))
          {
            // found solution
            final_nodes_.push_back(to);
          }
        }
      }
    }

    {
      {
      std::array<double, 2> s{0.5, 0.9};
      auto node = kdtree_->nearest_neighbor(s);

      std::cout << "nearest " << node->state[0] << " " << node->state[1] << std::endl;

      backtrack(node->state, s);

      auto from = rrttree_->get_node(node->id);
      auto to = rrttree_->add_node(s);

      kdtree_->add_node(to->state, to->id);
      rrttree_->add_edge(from, to);

      std::cout << "add " << s[0] << " " << s[1] << std::endl;
      }

      {
      std::array<double, 2> s{0.5, 0.9};
      auto node = kdtree_->nearest_neighbor(s);

      std::cout << "nearest " << node->state[0] << " " << node->state[1] << std::endl;

      backtrack(node->state, s);

      auto from = rrttree_->get_node(node->id);
      auto to = rrttree_->add_node(s);

      kdtree_->add_node(to->state, to->id);
      rrttree_->add_edge(from, to);

      std::cout << "add " << s[0] << " " << s[1] << std::endl;

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

  void backtrack(const std::array<double, S::dim>& from, std::array<double, S::dim>& to) const
  {
    // L1
    double max_step = 0;
    for(uint i = 0; i < S::dim; ++i)
    {
      const double delta = fabs(to[i] - from[i]);
      max_step = std::max(max_step, delta);
    }

    if(max_step > max_step_)
    {
      const double lambda = max_step_ / max_step;
      for(uint i = 0; i < S::dim; ++i)
      {
        to[i] = from[i] + (to[i] - from[i]) * lambda;
      }
    }

//    if(fabs(to[0] - 0.8) < 0.1 && fabs(to[1] - 0.15) < 0.1)
//    {
//      int a;
//      ++a;
//    }

    // L2
//    auto d = norm2(from, to);

//    if(d > max_step_)
//    {
//      const double lambda = max_step_ / d;

//      for(uint i = 0; i < S::dim; ++i)
//      {
//        to[i] = from[i] + (to[i] - from[i]) * lambda;
//      }
//    }
  }

  std::shared_ptr<RRTTree<S::dim>> rrt_tree() const
  {
    return rrttree_;
  }

private:
  const S & space_;
  const double max_step_;
  std::shared_ptr<RRTTree<S::dim>> rrttree_;
  std::unique_ptr<KDTree<S::dim>> kdtree_;
  std::function<bool(const std::array<double, S::dim> &)> state_checker_;
  std::function<bool(const std::array<double, S::dim> &, const std::array<double, S::dim> &)> transition_checker_;
  std::list<std::shared_ptr<RRTTreeNode<S::dim>>> final_nodes_;
};
