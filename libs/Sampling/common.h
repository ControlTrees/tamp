#pragma once
#include <array>
#include <deque>
#include <cmath>
#include <common.h>

template<std::size_t N>
double norm2(const std::array<double, N> & a, const std::array<double, N> & b)
{
  double sqdist = 0;
  for(auto i = 0; i < N; ++i)
  {
    const auto delta = a[i] - b[i];
    sqdist += delta * delta;
  }

  return sqrt(sqdist);
}

template<std::size_t N>
double norm1(const std::array<double, N> & a, const std::array<double, N> & b)
{
  double l1 = 0;
  for(auto i = 0; i < N; ++i)
  {
    l1 += std::fabs(a[i] - b[i]);
  }

  return l1;
}

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
  double step = norm1(from, to);

  if(step > max_step)
  {
    const double lambda = max_step / step;
    for(uint i = 0; i < N; ++i)
    {
      to[i] = from[i] + (to[i] - from[i]) * lambda;
    }
  }
}
