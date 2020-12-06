#pragma once

#include <cstdlib>
#include <array>

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
