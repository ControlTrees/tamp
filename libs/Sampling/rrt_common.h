#pragma once
#include <array>
#include <deque>

template <std::size_t N>
double get_cost(const std::deque<std::array<double, N>> & path)
{
  double cost = 0;

  for(uint i = 1; i < path.size(); ++i)
  {
    cost += norm2(path[i-1], path[i]);
  }

  return cost;
}

template <std::size_t N>
void backtrack(const std::array<double, N>& from, std::array<double, N>& to, double max_step)
{
  // L1
  double step = 0;
  for(uint i = 0; i < N; ++i)
  {
    const double delta = fabs(to[i] - from[i]);
    step = std::max(step, delta);
  }

  if(step > max_step)
  {
    const double lambda = max_step / step;
    for(uint i = 0; i < N; ++i)
    {
      to[i] = from[i] + (to[i] - from[i]) * lambda;
    }
  }

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
