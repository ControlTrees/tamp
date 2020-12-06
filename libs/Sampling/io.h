#pragma once

#include <fstream>
#include <array>
#include <vector>
#include <deque>
#include <memory>

class MapLoader
{
public:
  MapLoader(const std::string& filepath, const std::array<std::pair<double, double>, 2> & bounds)
    : bounds_(bounds)
  {
    uint w{0};
    uint h{0};

    std::ifstream input(filepath, std::ifstream::in);

    std::string content;
    std::getline(input, content); // format
    std::getline(input, content); // comments
    std::getline(input, content, ' '); // width
    w = std::stoi(content);

    std::getline(input, content);
    h = std::stoi(content);

    // init buffer and parse map
    buffer_ = std::vector<std::vector<uint>>(h);
    for(auto& line: buffer_)
    {
      line = std::vector<uint>(w);

      for(auto& v: line)
      {
        std::getline(input, content);
        v = std::stoi(content);
      }
    }

    input.close();

    ppm_ = w / (bounds[0].second - bounds[0].first);
  }

  bool is_state_valid(const std::array<double, 2> & xy) const
  {
    auto ij = get_ij(xy);

    return (buffer_[ij[0]][ij[1]] == 255);
  }

  const std::vector<std::vector<uint>> & buffer() const { return buffer_; }

  std::array<int, 2> get_ij(const std::array<double, 2> & xy) const
  {
    std::array<int, 2>ij;

    ij[0] = buffer_.size() - (xy[1] - bounds_[1].first) * ppm_;
    ij[1] = (xy[0] - bounds_[0].first) * ppm_;

    return ij;
  }

private:
  std::array<std::pair<double, double>, 2> bounds_;
  uint ppm_;

  std::vector<std::vector<uint>> buffer_;
};

template<uint W>
class MultiMap
{
public:
  MultiMap(std::array<std::string, W> filepaths, const std::array<std::pair<double, double>, 2> & bounds)
  {
    for(auto w = 0; w < filepaths.size(); ++w)
    {
      maps_.emplace_back(MapLoader(filepaths[w], bounds));
    }

    h_ = maps_[0].buffer().size();
    w_ = maps_[0].buffer()[0].size();

    // create fuse map, speed up!
    occupancies_ =   std::vector<std::vector<std::array<double, W>>>(h_);

    for(auto & line: occupancies_)
    {
      line = std::vector<std::array<double, W>>(w_);
    }

    for(auto i = 0; i < h_; ++i)
    {
      for(auto j = 0; j < w_; ++j)
      {
        for(auto w = 0; w < W; ++w)
        {
          occupancies_[i][j][w] = maps_[w].buffer()[i][j] == 255 ? 1 : 0;
        }
      }
    }
  }

  std::array<double, W> is_state_valid(const std::array<double, 2> & xy) const
  {
    const auto ij = maps_[0].get_ij(xy);
    return occupancies_[ij[0]][ij[1]];
  }

private:
  std::vector<MapLoader> maps_;
  std::vector<std::vector<std::array<double, W>>> occupancies_;
  uint w_;
  uint h_;
};

class Drawer
{
public:
  Drawer(const std::array<std::pair<double, double>, 2> & bounds, uint ppm)
    : bounds_(bounds)
    , ppm_(ppm)
  {
    uint w = (bounds_[0].second - bounds_[0].first) * ppm;
    uint h = (bounds_[1].second - bounds_[1].first) * ppm;

    buffer_ = std::vector<std::vector<uint>>(h);

    for(auto& line: buffer_)
    {
      line = std::vector<uint>(w, 10);
    }
  }

  template<typename T>
  void draw_tree(const std::shared_ptr<T> & tree)
  {
    for(const auto& node: tree->nodes())
    {
      auto from = node->state;
      auto from_ij = get_ij(from);

      for(const auto& child: node->children)
      {
        auto to = child->state;
        auto to_ij = get_ij(to);

        draw_line(from_ij, to_ij, 5);
      }
    }
  }

  void draw_path(std::deque<std::array<double, 2>> path)
  {
    for(uint i = 1; i < path.size(); ++i)
    {
      draw_line(get_ij(path[i-1]), get_ij(path[i]), 0);
    }
  }

  void draw_line(const std::array<int, 2> & from, const std::array<int, 2> & to, uint color)
  {
    auto di = std::abs(to[0] - from[0]);
    auto dj = std::abs(to[1] - from[1]);

    auto n = di > dj ? di : dj;

    for(double s = 0; s < n; ++s)
    {
      auto i = from[0] + s/n * (to[0] - from[0]);
      auto j = from[1] + s/n * (to[1] - from[1]);

      buffer_[i][j] = color;
    }
  }

  void save(const std::string & filepath) const
  {
    std::ofstream of;
    of.open(filepath);

    of << "P2" << std::endl;
    of << buffer_.front().size() << " " << buffer_.size() << std::endl;
    of << "10" << std::endl;

    for(const auto& line: buffer_)
    {
      for(uint i = 0; i < line.size() -1; ++i)
      {
        of << line[i] << " ";
      }
      of << line[line.size() - 1];
      of << std::endl;
    }

    of.close();
  }

private:
  std::array<int, 2> get_ij(const std::array<double, 2> & xy) const
  {
    std::array<int, 2>ij;

    ij[0] = buffer_.size() - (xy[1] - bounds_[1].first) * ppm_;
    ij[1] = (xy[0] - bounds_[0].first) * ppm_;

    return ij;
  }

private:
  std::array<std::pair<double, double>, 2> bounds_;
  uint ppm_;

  std::vector<std::vector<uint>> buffer_;
};
