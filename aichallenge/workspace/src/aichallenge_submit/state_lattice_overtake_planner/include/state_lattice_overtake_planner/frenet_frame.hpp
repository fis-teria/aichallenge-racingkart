#pragma once

#include "state_lattice_overtake_planner/types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace state_lattice_overtake_planner
{

class FrenetFrame
{
public:
  bool loadCsv(const std::string & path, std::string * error = nullptr);
  bool setReference(std::vector<ReferencePoint> points, std::string * error = nullptr);
  bool empty() const {return points_.size() < 2U;}
  double length() const {return length_m_;}
  const std::vector<ReferencePoint> & points() const {return points_;}

  double wrapS(double s) const;
  double forwardDeltaS(double from_s, double to_s) const;
  ReferencePoint interpolate(double s) const;
  ReferencePoint frenetToCartesian(double s, double d) const;
  FrenetPoint project(double x, double y, double yaw) const;
  FrenetPoint projectContinuous(
    double x, double y, double yaw, double expected_unwrapped_s,
    double search_half_width_m) const;
  std::size_t nearestIndex(double s) const;
  double unwrappedIndexS(long long index, std::size_t anchor_index, double anchor_s) const;

private:
  FrenetPoint projectInWindow(
    double x, double y, double yaw, double expected_unwrapped_s,
    double search_half_width_m, bool unrestricted) const;

  std::vector<ReferencePoint> points_;
  double length_m_{0.0};
};

}  // namespace state_lattice_overtake_planner
