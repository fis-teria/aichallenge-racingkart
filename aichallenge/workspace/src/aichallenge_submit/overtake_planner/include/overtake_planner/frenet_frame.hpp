#pragma once

#include "overtake_planner/types.hpp"

#include <string>
#include <vector>

namespace overtake_planner
{

// 参照線の各sに対応する、lanelet路面端までのFrenet横位置。
// d_min/d_maxは車体中心境界ではない。SafetyEvaluator側で中心点marginと
// base_link基準の車体footprintをそれぞれ検査して安全回廊にする。
struct FrenetCorridorPoint
{
  double s{0.0};
  double d_min{0.0};
  double d_max{0.0};
};

struct FrenetCorridorBounds
{
  double d_min{0.0};
  double d_max{0.0};
};

class FrenetFrame
{
public:
  // 参照CSVを読み、Cartesian <-> Frenet変換で使う閉ループの中心線を作る。
  bool loadCsv(const std::string & path, std::string * error = nullptr);
  // 参照CSVと同じsサンプルで作った路面境界CSVを読む。参照と一致しない
  // profileは使わずfalseを返すため、別コースの回廊を誤適用しない。
  bool loadCorridorCsv(const std::string & path, std::string * error = nullptr);
  void setReference(std::vector<ReferencePoint> reference);
  void setCorridor(std::vector<FrenetCorridorPoint> corridor);

  bool empty() const { return reference_.empty(); }
  bool hasCorridor() const { return !corridor_.empty(); }
  double length() const { return track_length_; }
  const std::vector<ReferencePoint> & reference() const { return reference_; }

  double wrapS(double s) const;
  // ループコース上でfrom_sからto_sまで前向きに進んだ距離を返す。
  double deltaS(double from_s, double to_s) const;
  FrenetPose cartesianToFrenet(double x, double y, double yaw) const;
  ReferencePoint interpolate(double s) const;
  ReferencePoint frenetToCartesian(double s, double d) const;
  // corridor未ロード時は従来の固定d境界を返す。ロード済みなら隣接サンプルの
  // 狭い側を使い、補間で実境界より外へ広がらないようにする。
  FrenetCorridorBounds corridorBounds(double s, double fallback_d_min,
                                      double fallback_d_max) const;

private:
  std::vector<ReferencePoint> reference_;
  std::vector<FrenetCorridorPoint> corridor_;
  double track_length_{0.0};
};

double normalizeAngle(double angle);

}  // namespace overtake_planner
