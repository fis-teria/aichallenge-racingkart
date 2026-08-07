#include "state_lattice_overtake_planner/grid_map.hpp"

#include "state_lattice_overtake_planner/frenet_frame.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>

namespace state_lattice_overtake_planner
{
namespace
{
std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {return {};}
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

bool startsWith(const std::string & text, const std::string & prefix)
{
  return text.compare(0U, prefix.size(), prefix) == 0;
}

std::string yamlScalar(const std::string & line)
{
  const auto colon = line.find(':');
  if (colon == std::string::npos) {return {};}
  std::string value = trim(line.substr(colon + 1U));
  if (value.size() >= 2U && value.front() == '"' && value.back() == '"') {
    value = value.substr(1U, value.size() - 2U);
  }
  return value;
}

bool nextPgmToken(std::istream & stream, std::string * token)
{
  token->clear();
  char c = 0;
  while (stream.get(c)) {
    if (c == '#') {std::string ignored; std::getline(stream, ignored); continue;}
    if (!std::isspace(static_cast<unsigned char>(c))) {token->push_back(c); break;}
  }
  while (stream.get(c)) {
    if (std::isspace(static_cast<unsigned char>(c))) {break;}
    token->push_back(c);
  }
  return !token->empty();
}

std::vector<double> distanceTransform1d(const std::vector<double> & input)
{
  const int count = static_cast<int>(input.size());
  std::vector<int> sites(static_cast<std::size_t>(count));
  std::vector<double> boundaries(static_cast<std::size_t>(count + 1));
  std::vector<double> output(static_cast<std::size_t>(count));
  int envelope = 0;
  sites[0] = 0;
  boundaries[0] = -std::numeric_limits<double>::infinity();
  boundaries[1] = std::numeric_limits<double>::infinity();
  for (int q = 1; q < count; ++q) {
    double boundary = 0.0;
    do {
      const int site = sites[envelope];
      boundary = ((input[static_cast<std::size_t>(q)] + q * q) -
        (input[static_cast<std::size_t>(site)] + site * site)) / (2.0 * (q - site));
      if (boundary <= boundaries[static_cast<std::size_t>(envelope)]) {--envelope;}
      else {break;}
    } while (envelope >= 0);
    ++envelope;
    sites[envelope] = q;
    boundaries[static_cast<std::size_t>(envelope)] = boundary;
    boundaries[static_cast<std::size_t>(envelope + 1)] = std::numeric_limits<double>::infinity();
  }
  envelope = 0;
  for (int q = 0; q < count; ++q) {
    while (boundaries[static_cast<std::size_t>(envelope + 1)] < q) {++envelope;}
    const double delta = q - sites[envelope];
    output[static_cast<std::size_t>(q)] = delta * delta + input[static_cast<std::size_t>(sites[envelope])];
  }
  return output;
}

std::vector<float> euclideanDistanceTransform(
  std::size_t width, std::size_t height, const std::vector<std::uint8_t> & seeds,
  double resolution)
{
  constexpr double far = 1.0e12;
  std::vector<double> horizontal(width * height, far);
  std::vector<double> line;
  line.reserve(std::max(width, height));
  for (std::size_t y = 0; y < height; ++y) {
    line.assign(width, far);
    for (std::size_t x = 0; x < width; ++x) {
      if (seeds[y * width + x] != 0U) {line[x] = 0.0;}
    }
    const auto transformed = distanceTransform1d(line);
    std::copy(transformed.begin(), transformed.end(), horizontal.begin() + static_cast<std::ptrdiff_t>(y * width));
  }
  std::vector<float> distance(width * height, 0.0F);
  for (std::size_t x = 0; x < width; ++x) {
    line.resize(height);
    for (std::size_t y = 0; y < height; ++y) {line[y] = horizontal[y * width + x];}
    const auto transformed = distanceTransform1d(line);
    for (std::size_t y = 0; y < height; ++y) {
      distance[y * width + x] = static_cast<float>(std::sqrt(transformed[y]) * resolution);
    }
  }
  return distance;
}
}  // namespace

bool GridMap::load(const std::string & yaml_path, const PlannerConfig & config, std::string * error)
{
  std::ifstream yaml(yaml_path);
  if (!yaml) {if (error != nullptr) {*error = "failed to open map yaml: " + yaml_path;} return false;}
  std::string image;
  double occupied_threshold = 0.65;
  bool negate = false;
  std::string line;
  while (std::getline(yaml, line)) {
    line = trim(line);
    try {
      if (startsWith(line, "image:")) {image = yamlScalar(line);}
      else if (startsWith(line, "resolution:")) {resolution_m_ = std::stod(yamlScalar(line));}
      else if (startsWith(line, "occupied_thresh:")) {occupied_threshold = std::stod(yamlScalar(line));}
      else if (startsWith(line, "negate:")) {negate = std::stoi(yamlScalar(line)) != 0;}
      else if (startsWith(line, "origin:")) {
        const auto begin = line.find('[');
        const auto end = line.find(']');
        if (begin != std::string::npos && end != std::string::npos) {
          std::stringstream values(line.substr(begin + 1U, end - begin - 1U));
          std::string value;
          if (std::getline(values, value, ',')) {origin_x_m_ = std::stod(trim(value));}
          if (std::getline(values, value, ',')) {origin_y_m_ = std::stod(trim(value));}
        }
      } else if (!line.empty() && line.front() == '-' && origin_x_m_ == 0.0) {
        origin_x_m_ = std::stod(trim(line.substr(1U)));
        if (std::getline(yaml, line)) {origin_y_m_ = std::stod(trim(line.substr(line.find('-') + 1U)));}
      }
    } catch (...) {
      if (error != nullptr) {*error = "invalid map yaml field: " + line;}
      return false;
    }
  }
  if (image.empty() || resolution_m_ <= 0.0 ||
    std::abs(resolution_m_ - config.expected_map_resolution_m) > 1.0e-6)
  {
    if (error != nullptr) {*error = "map yaml is incomplete or has unexpected resolution";}
    return false;
  }
  const auto pgm_path = std::filesystem::path(yaml_path).parent_path() / image;
  if (!readPgm(pgm_path.string(), occupied_threshold, negate, 5, error)) {return false;}
  buildWallDistanceAndCost(config);
  ++static_generation_;
  return true;
}

bool GridMap::readPgm(
  const std::string & path, double occupied_threshold, bool negate,
  int minimum_component_cells, std::string * error)
{
  std::ifstream input(path, std::ios::binary);
  std::string magic, token;
  if (!input || !nextPgmToken(input, &magic) || (magic != "P5" && magic != "P2") ||
    !nextPgmToken(input, &token))
  {
    if (error != nullptr) {*error = "invalid PGM header: " + path;}
    return false;
  }
  width_ = static_cast<std::size_t>(std::stoul(token));
  if (!nextPgmToken(input, &token)) {return false;}
  height_ = static_cast<std::size_t>(std::stoul(token));
  if (!nextPgmToken(input, &token)) {return false;}
  const int max_value = std::stoi(token);
  if (width_ == 0U || height_ == 0U || max_value <= 0 || max_value > 255) {
    if (error != nullptr) {*error = "unsupported PGM dimensions or depth";}
    return false;
  }
  std::vector<std::uint8_t> pixels(width_ * height_, 0U);
  if (magic == "P5") {
    input.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    if (input.gcount() != static_cast<std::streamsize>(pixels.size())) {
      if (error != nullptr) {*error = "truncated PGM data";}
      return false;
    }
  } else {
    for (auto & pixel : pixels) {
      if (!nextPgmToken(input, &token)) {return false;}
      pixel = static_cast<std::uint8_t>(std::stoi(token));
    }
  }
  occupied_.assign(width_ * height_, 0U);
  for (std::size_t image_y = 0; image_y < height_; ++image_y) {
    const std::size_t cell_y = height_ - 1U - image_y;
    for (std::size_t x = 0; x < width_; ++x) {
      const double normalized =
        static_cast<double>(pixels[image_y * width_ + x]) / max_value;
      const double occupancy_probability = negate ? normalized : 1.0 - normalized;
      occupied_[cell_y * width_ + x] =
        occupancy_probability > occupied_threshold ? 1U : 0U;
    }
  }

  std::vector<std::uint8_t> visited(occupied_.size(), 0U);
  constexpr std::array<int, 8> dx{-1, 0, 1, -1, 1, -1, 0, 1};
  constexpr std::array<int, 8> dy{-1, -1, -1, 0, 0, 1, 1, 1};
  for (int y = 0; y < static_cast<int>(height_); ++y) {
    for (int x = 0; x < static_cast<int>(width_); ++x) {
      const auto start = index(x, y);
      if (occupied_[start] == 0U || visited[start] != 0U) {continue;}
      std::vector<std::size_t> component;
      std::queue<std::pair<int, int>> queue;
      queue.emplace(x, y);
      visited[start] = 1U;
      while (!queue.empty()) {
        const auto current = queue.front(); queue.pop();
        component.push_back(index(current.first, current.second));
        for (std::size_t k = 0; k < dx.size(); ++k) {
          const int nx = current.first + dx[k];
          const int ny = current.second + dy[k];
          if (nx < 0 || ny < 0 || nx >= static_cast<int>(width_) || ny >= static_cast<int>(height_)) {continue;}
          const auto ni = index(nx, ny);
          if (occupied_[ni] != 0U && visited[ni] == 0U) {visited[ni] = 1U; queue.emplace(nx, ny);}
        }
      }
      if (static_cast<int>(component.size()) < minimum_component_cells) {
        for (const auto cell_index : component) {occupied_[cell_index] = 0U;}
      }
    }
  }
  return true;
}

bool GridMap::buildReferenceLayer(const FrenetFrame & frame, const PlannerConfig & config)
{
  if (!initialized() || frame.empty()) {return false;}
  std::vector<std::uint8_t> seeds(width_ * height_, 0U);
  for (std::size_t i = 0; i < frame.points().size(); ++i) {
    const auto & a = frame.points()[i];
    const auto & b = frame.points()[(i + 1U) % frame.points().size()];
    const int samples = std::max(1, static_cast<int>(std::ceil(
      std::hypot(b.x - a.x, b.y - a.y) / (0.5 * resolution_m_))));
    for (int sample = 0; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      int x = 0, y = 0;
      if (!worldToCell(a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y), &x, &y)) {
        return false;
      }
      seeds[index(x, y)] = 1U;
    }
  }
  const auto distance = euclideanDistanceTransform(width_, height_, seeds, resolution_m_);
  reference_levels_.resize(distance.size());
  for (std::size_t i = 0; i < distance.size(); ++i) {
    reference_levels_[i] = static_cast<std::uint8_t>(referenceCostLevel(distance[i], config));
  }
  ++static_generation_;
  return true;
}

void GridMap::buildWallDistanceAndCost(const PlannerConfig & config)
{
  wall_level_9_distance_m_ = config.wall_distance_thresholds_m.front();
  wall_cost_max_distance_m_ = config.wall_distance_thresholds_m.back();
  wall_distance_m_ = euclideanDistanceTransform(width_, height_, occupied_, resolution_m_);
  wall_levels_.resize(occupied_.size());
  for (std::size_t i = 0; i < wall_levels_.size(); ++i) {
    wall_levels_[i] = static_cast<std::uint8_t>(wallCostLevel(wall_distance_m_[i], config));
  }
}

bool GridMap::worldToCell(double x, double y, int * cell_x, int * cell_y) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || !initialized()) {return false;}
  const int cx = static_cast<int>(std::floor((x - origin_x_m_) / resolution_m_));
  const int cy = static_cast<int>(std::floor((y - origin_y_m_) / resolution_m_));
  if (cx < 0 || cy < 0 || cx >= static_cast<int>(width_) || cy >= static_cast<int>(height_)) {return false;}
  if (cell_x != nullptr) {*cell_x = cx;}
  if (cell_y != nullptr) {*cell_y = cy;}
  return true;
}

Pose2d GridMap::cellCenter(int x, int y) const
{
  return {origin_x_m_ + (static_cast<double>(x) + 0.5) * resolution_m_,
    origin_y_m_ + (static_cast<double>(y) + 0.5) * resolution_m_, 0.0};
}

bool GridMap::occupied(int x, int y) const
{
  return x < 0 || y < 0 || x >= static_cast<int>(width_) || y >= static_cast<int>(height_) || occupied_[index(x, y)] != 0U;
}

double GridMap::wallDistance(int x, int y) const
{
  return x < 0 || y < 0 || x >= static_cast<int>(width_) || y >= static_cast<int>(height_) ? 0.0 : wall_distance_m_[index(x, y)];
}

int GridMap::wallLevel(int x, int y) const
{
  return x < 0 || y < 0 || x >= static_cast<int>(width_) || y >= static_cast<int>(height_) ? 9 : wall_levels_[index(x, y)];
}

int GridMap::maxWallLevelInFootprint(const Pose2d & pose, const Footprint & fp) const
{
  int center_x = 0;
  int center_y = 0;
  if (worldToCell(pose.x, pose.y, &center_x, &center_y)) {
    const double half_cell = resolution_m_ * std::sqrt(2.0) * 0.5;
    const double longitudinal = std::max(fp.front_m, fp.rear_m) + half_cell;
    const double lateral = std::max(fp.left_m, fp.right_m) + half_cell;
    const double radius = std::hypot(longitudinal, lateral) + half_cell;
    if (wallDistance(center_x, center_y) > radius + wall_cost_max_distance_m_) {
      return 0;
    }
  }

  const auto corners = footprintCorners(pose, fp);
  double min_x = corners[0].x, max_x = corners[0].x, min_y = corners[0].y, max_y = corners[0].y;
  for (const auto & corner : corners) {
    min_x = std::min(min_x, corner.x); max_x = std::max(max_x, corner.x);
    min_y = std::min(min_y, corner.y); max_y = std::max(max_y, corner.y);
  }
  int x0 = static_cast<int>(std::floor((min_x - origin_x_m_) / resolution_m_));
  int x1 = static_cast<int>(std::floor((max_x - origin_x_m_) / resolution_m_));
  int y0 = static_cast<int>(std::floor((min_y - origin_y_m_) / resolution_m_));
  int y1 = static_cast<int>(std::floor((max_y - origin_y_m_) / resolution_m_));
  int maximum = 0;
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const auto center = cellCenter(x, y);
      if (pointInsideFootprint(center.x, center.y, pose, fp)) {
        maximum = std::max(maximum, wallLevel(x, y));
        if (maximum >= 9) {return maximum;}
      }
    }
  }
  return maximum;
}

bool GridMap::footprintHitsWall(const Pose2d & pose, const Footprint & fp) const
{
  int center_x = 0;
  int center_y = 0;
  if (worldToCell(pose.x, pose.y, &center_x, &center_y)) {
    const double half_cell = resolution_m_ * std::sqrt(2.0) * 0.5;
    const double longitudinal = std::max(fp.front_m, fp.rear_m) + half_cell;
    const double lateral = std::max(fp.left_m, fp.right_m) + half_cell;
    const double radius = std::hypot(longitudinal, lateral) + half_cell;
    if (wallDistance(center_x, center_y) > radius + wall_level_9_distance_m_) {
      return false;
    }
  }

  const auto corners = footprintCorners(pose, fp);
  double min_x = corners[0].x;
  double max_x = corners[0].x;
  double min_y = corners[0].y;
  double max_y = corners[0].y;
  for (const auto & corner : corners) {
    min_x = std::min(min_x, corner.x);
    max_x = std::max(max_x, corner.x);
    min_y = std::min(min_y, corner.y);
    max_y = std::max(max_y, corner.y);
  }
  const int x0 = static_cast<int>(std::floor((min_x - origin_x_m_) / resolution_m_));
  const int x1 = static_cast<int>(std::floor((max_x - origin_x_m_) / resolution_m_));
  const int y0 = static_cast<int>(std::floor((min_y - origin_y_m_) / resolution_m_));
  const int y1 = static_cast<int>(std::floor((max_y - origin_y_m_) / resolution_m_));
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (wallLevel(x, y) < 9) {continue;}
      const auto center = cellCenter(x, y);
      if (pointInsideFootprint(center.x, center.y, pose, fp)) {return true;}
    }
  }
  return false;
}

std::size_t GridMap::index(int x, int y) const
{
  return static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x);
}

bool GridMap::pointInsideFootprint(double x, double y, const Pose2d & pose, const Footprint & fp) const
{
  const double dx = x - pose.x;
  const double dy = y - pose.y;
  const double longitudinal = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
  const double lateral = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
  const double half_cell = resolution_m_ * std::sqrt(2.0) * 0.5;
  return longitudinal >= -fp.rear_m - half_cell && longitudinal <= fp.front_m + half_cell &&
         lateral >= -fp.right_m - half_cell && lateral <= fp.left_m + half_cell;
}

}  // namespace state_lattice_overtake_planner
