#include "simple_state_lattice_planner/static_input_loader.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

namespace simple_state_lattice_planner {
namespace {

void fail(std::string *error, const std::string &message) {
  if (error != nullptr) *error = message;
}

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n\"'");
  const auto last = value.find_last_not_of(" \t\r\n\"'");
  return first == std::string::npos ? std::string{} : value.substr(first, last - first + 1U);
}

bool pgmToken(std::istream &stream, std::string *token) {
  while (stream >> *token) {
    if (!token->empty() && token->front() == '#') {
      std::string ignored;
      std::getline(stream, ignored);
      continue;
    }
    return true;
  }
  return false;
}

}  // namespace

bool loadReferenceCsv(const std::string &path, ReferenceWindow *reference,
                      std::string *error) {
  if (reference == nullptr) return false;
  std::ifstream input(path);
  if (!input) { fail(error, "reference_open_failed"); return false; }
  ReferenceWindow loaded;
  loaded.snapshot_id = 1U;
  loaded.frame_id = "map";
  loaded.stamp_sec = 1.0;
  std::string line;
  std::getline(input, line);
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    ReferencePoint point;
    double curvature = 0.0, speed = 0.0, acceleration = 0.0;
    if (!(row >> point.s_m >> point.x_m >> point.y_m >> point.yaw_rad >>
          curvature >> speed >> acceleration) ||
        !std::isfinite(point.s_m) || !std::isfinite(point.x_m) ||
        !std::isfinite(point.y_m) || !std::isfinite(point.yaw_rad) ||
        (!loaded.points.empty() && point.s_m <= loaded.points.back().s_m)) {
      fail(error, "reference_row_invalid"); return false;
    }
    loaded.points.push_back(point);
  }
  if (loaded.points.size() < 2U || loaded.points.size() > 100000U) {
    fail(error, "reference_size_invalid"); return false;
  }
  *reference = std::move(loaded);
  return true;
}

bool loadOccupancyGridYaml(const std::string &path, Costmap2D *costmap,
                           std::string *error) {
  if (costmap == nullptr) return false;
  std::ifstream yaml(path);
  if (!yaml) { fail(error, "map_yaml_open_failed"); return false; }
  std::string image_name;
  double resolution = 0.0, occupied_threshold = 0.65, free_threshold = 0.196;
  double origin_x = 0.0, origin_y = 0.0;
  int negate = 0;
  bool reading_origin = false;
  int origin_index = 0;
  std::string line;
  while (std::getline(yaml, line)) {
    const auto colon = line.find(':');
    if (colon != std::string::npos) {
      const std::string key = trim(line.substr(0U, colon));
      const std::string value = trim(line.substr(colon + 1U));
      reading_origin = key == "origin";
      if (key == "image") image_name = value;
      else if (key == "resolution") resolution = std::stod(value);
      else if (key == "occupied_thresh") occupied_threshold = std::stod(value);
      else if (key == "free_thresh") free_threshold = std::stod(value);
      else if (key == "negate") negate = std::stoi(value);
      continue;
    }
    if (reading_origin) {
      const auto dash = line.find('-');
      if (dash != std::string::npos && origin_index < 2) {
        const double value = std::stod(trim(line.substr(dash + 1U)));
        if (origin_index++ == 0) origin_x = value; else origin_y = value;
      }
    }
  }
  if (image_name.empty() || !std::isfinite(resolution) || resolution <= 0.0 ||
      origin_index != 2 || !(free_threshold < occupied_threshold)) {
    fail(error, "map_yaml_invalid"); return false;
  }
  const auto pgm_path = std::filesystem::path(path).parent_path() / image_name;
  std::ifstream pgm(pgm_path, std::ios::binary);
  std::string magic, token;
  if (!pgmToken(pgm, &magic) || (magic != "P5" && magic != "P2") ||
      !pgmToken(pgm, &token)) { fail(error, "map_pgm_header_invalid"); return false; }
  const auto width = static_cast<std::size_t>(std::stoull(token));
  if (!pgmToken(pgm, &token)) return false;
  const auto height = static_cast<std::size_t>(std::stoull(token));
  if (!pgmToken(pgm, &token)) return false;
  const int max_value = std::stoi(token);
  if (width == 0U || height == 0U || width > 1000000U / height ||
      max_value <= 0 || max_value > 255) {
    fail(error, "map_pgm_dimensions_invalid"); return false;
  }
  std::vector<unsigned char> pixels(width * height);
  if (magic == "P5") {
    pgm.get();
    pgm.read(reinterpret_cast<char *>(pixels.data()),
             static_cast<std::streamsize>(pixels.size()));
    if (pgm.gcount() != static_cast<std::streamsize>(pixels.size())) {
      fail(error, "map_pgm_truncated"); return false;
    }
  } else {
    for (auto &pixel : pixels) {
      if (!pgmToken(pgm, &token)) { fail(error, "map_pgm_truncated"); return false; }
      pixel = static_cast<unsigned char>(std::stoi(token));
    }
  }
  Costmap2D loaded;
  loaded.snapshot_id = 1U;
  loaded.stamp_sec = 1.0;
  loaded.resolution_m = resolution;
  loaded.origin_x_m = origin_x;
  loaded.origin_y_m = origin_y;
  loaded.width = width;
  loaded.height = height;
  loaded.cells.assign(width * height, kUnknownCost);
  for (std::size_t image_y = 0U; image_y < height; ++image_y) {
    const std::size_t cell_y = height - 1U - image_y;
    for (std::size_t x = 0U; x < width; ++x) {
      const double normalized = static_cast<double>(pixels[image_y * width + x]) / max_value;
      const double occupancy = negate != 0 ? normalized : 1.0 - normalized;
      loaded.cells[cell_y * width + x] = occupancy > occupied_threshold
          ? kOccupiedCost : (occupancy < free_threshold ? 0U : kUnknownCost);
    }
  }
  if (!loaded.valid()) { fail(error, "map_invalid"); return false; }
  *costmap = std::move(loaded);
  return true;
}

}  // namespace simple_state_lattice_planner
