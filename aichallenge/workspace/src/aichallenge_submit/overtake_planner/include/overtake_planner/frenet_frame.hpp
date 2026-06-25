#pragma once

#include "overtake_planner/types.hpp"

#include <string>
#include <vector>

namespace overtake_planner
{

class FrenetFrame
{
public:
  bool loadCsv(const std::string & path, std::string * error = nullptr);
  void setReference(std::vector<ReferencePoint> reference);

  bool empty() const { return reference_.empty(); }
  double length() const { return track_length_; }
  const std::vector<ReferencePoint> & reference() const { return reference_; }

  double wrapS(double s) const;
  double deltaS(double from_s, double to_s) const;
  FrenetPose cartesianToFrenet(double x, double y, double yaw) const;
  ReferencePoint interpolate(double s) const;
  ReferencePoint frenetToCartesian(double s, double d) const;

private:
  std::vector<ReferencePoint> reference_;
  double track_length_{0.0};
};

double normalizeAngle(double angle);

}  // namespace overtake_planner
