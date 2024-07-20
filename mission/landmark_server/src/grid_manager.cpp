#include <iostream>
#include <landmark_server/grid_manager.hpp>

namespace landmark_server {

GridManager::GridManager(float resolution, uint32_t height, uint32_t width)
    : resolution_(resolution), height_(height), width_(width) {}

void GridManager::update_grid(
    int8_t *grid, const Eigen::Array<float, 2, Eigen::Dynamic> &polygon,
    int value) {
  handle_polygon(grid, polygon, value);
}

void GridManager::handle_polygon(
    int8_t *grid, const Eigen::Array<float, 2, Eigen::Dynamic> &vertices,
    int value) {
  // Convert to grid coordinates
  for (Eigen::Index i = 0; i < vertices.cols(); ++i) {
    Eigen::Index j = (i + 1) % vertices.cols();
    int x0 = static_cast<int>(vertices(0, i) / resolution_ + width_ / 2);
    int y0 = static_cast<int>(vertices(1, i) / resolution_ + height_ / 2);
    int x1 = static_cast<int>(vertices(0, j) / resolution_ + width_ / 2);
    int y1 = static_cast<int>(vertices(1, j) / resolution_ + height_ / 2);

    draw_line(x0, y0, x1, y1, grid, value);
  }
}

void GridManager::draw_line(int x0, int y0, int x1, int y1, int8_t *grid,
                            int value) {
  bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }
  int dx = x1 - x0;
  int dy = std::abs(y1 - y0);
  int error = 2 * dy - dx;
  int ystep = (y0 < y1) ? 1 : -1;
  int xbounds = steep ? height_ : width_;
  int ybounds = steep ? width_ : height_;
  int y = y0;

  for (int x = x0; x <= x1; x++) {
    if (steep) {
      if (x >= 0 && x < xbounds && y >= 0 && y < ybounds) {
        grid[y + x * height_] += value;
      }
    } else {
      if (y >= 0 && y < ybounds && x >= 0 && x < xbounds) {
        grid[y * width_ + x] += value;
      }
    }
    if (error > 0) {
      y += ystep;
      error -= 2 * (dx - dy);
    } else {
      error += 2 * dy;
    }
  }
}

} // namespace landmark_server