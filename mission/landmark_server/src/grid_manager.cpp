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
  if ( vertices.cols() > 1){
  fill_polygon(grid, vertices, value);
  }
}

void GridManager::fill_polygon(int8_t *grid,
                               const Eigen::Array<float, 2, Eigen::Dynamic> &polygon,
                               int value) {
      double max_x = polygon.row(0).maxCoeff();
      double min_x = polygon.row(0).minCoeff();
      double max_y = polygon.row(1).maxCoeff();
      double min_y = polygon.row(1).minCoeff();
      
      for (int x = min_x; x < max_x; x++) {
          for (int y = min_y; y < max_y; y++) {
            if (x >= 0 && x < static_cast<int>(width_) && y >= 0 && y < static_cast<int>(height_)) {
                if (point_in_polygon(x, y, polygon)) {
                    grid[y * width_ + x] += value;
                }
          }
        }  
      }
}

bool GridManager::point_in_polygon(int x, int y, const Eigen::Array<float, 2, Eigen::Dynamic> &polygon) {
    int i, j;
    bool c = false;
    for (i = 0, j = polygon.cols() - 1; i < polygon.cols(); j = i++) {
        if (((polygon(1, i) > y) != (polygon(1, j) > y)) &&
            (x < (polygon(0, j) - polygon(0, i)) * (y - polygon(1, i)) / (polygon(1, j) - polygon(1, i)) + polygon(0, i))
            )
            c = !c;
    }
    return c;
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