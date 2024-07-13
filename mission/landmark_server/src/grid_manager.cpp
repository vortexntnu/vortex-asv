#include <iostream>
#include <landmark_server/grid_manager.hpp>

namespace landmark_server {

// GridManager::GridManager(std::vector<int8_t> grid, float resolution, uint32_t
// height, uint32_t width)
//     : resolution_(resolution), height_(height), width_(width)
// {

//     if (grid.size() != height * width) {
//         throw std::runtime_error("Grid size does not match height * width");
//     }
//     grid_ = Eigen::Map<const Eigen::Array<int8_t, Eigen::Dynamic,
//     Eigen::Dynamic>>
//         (grid.data(), height, width);
// }

GridManager::GridManager(float resolution, uint32_t height, uint32_t width)
    : resolution_(resolution), height_(height), width_(width) {}

const GridManager::Grid &GridManager::get_grid() const { return grid_; }

void GridManager::update_grid(
    int8_t *grid, const Eigen::Array<float, 2, Eigen::Dynamic> &polygon,
    int value) {
  handle_polygon(grid, polygon, value);
}

// void GridManager::update_grid(const ShapeInfo& shape_info, int value)
// {
//     switch (shape_info.type)
//     {
//     case ShapeType::SPHERE:
//         handle_circle(shape_info.circle.x_centre, shape_info.circle.y_centre,
//         shape_info.circle.radius, value); break;
//     case ShapeType::PRISM:
//         handle_polygon(shape_info.polygon.vertices, value);
//     default:
//         break;
//     }
// }

void GridManager::draw_circle(
    int xc, int yc, int x, int y, int value,
    Eigen::Block<landmark_server::GridManager::Grid, -1, -1, false> &block,
    int xmin, int ymin) {
  // Translate coordinates to block local coordinates
  int bx = xc - xmin;
  int by = yc - ymin;

  if (bx + x < block.rows() && by + y < block.cols() && bx + x >= 0 &&
      by + y >= 0) {
    block(bx + x, by + y) = value;
    block(bx - x, by + y) = value;
    block(bx + x, by - y) = value;
    block(bx - x, by - y) = value;
    block(bx + y, by + x) = value;
    block(bx - y, by + x) = value;
    block(bx + y, by - x) = value;
    block(bx - y, by - x) = value;
  }
}

void GridManager::handle_circle(int xc, int yc, int r, int value) {
  int xmin = static_cast<int>((xc - r) / resolution_ + width_ / 2);
  int ymin = static_cast<int>((yc - r) / resolution_ + width_ / 2);
  int xmax = static_cast<int>((xc + r) / resolution_ + width_ / 2);
  int ymax = static_cast<int>((yc + r) / resolution_ + width_ / 2);

  if (xmin < 0 || ymin < 0 || xmax >= grid_.rows() || ymax >= grid_.cols()) {
    throw std::runtime_error("Circle exceeds grid boundaries");
  }

  auto block = grid_.block(xmin, ymin, xmax - xmin + 1, ymax - ymin + 1);
  int x = 0, y = r;
  int d = 3 - 2 * r;
  draw_circle(xc, yc, x, y, value, block, xmin, ymin);
  while (y >= x) {
    x++;
    if (d > 0) {
      y--;
      d = d + 4 * (x - y) + 10;
    } else {
      d = d + 4 * x + 6;
    }
    draw_circle(xc, yc, x, y, value, block, xmin, ymin);
  }
}

void GridManager::handle_polygon(
    int8_t *grid, const Eigen::Array<float, 2, Eigen::Dynamic> &vertices,
    int value) {
  // Determine the bounding box of the polygon
  // float min_x = vertices.row(0).minCoeff();
  // float min_y = vertices.row(1).minCoeff();
  // float max_x = vertices.row(0).maxCoeff();
  // float max_y = vertices.row(1).maxCoeff();

  // int xmin = static_cast<int>(min_x / resolution_ + width_ / 2);
  // int ymin = static_cast<int>(min_y / resolution_ + height_ / 2);
  // int xmax = static_cast<int>(max_x / resolution_ + width_ / 2);
  // int ymax = static_cast<int>(max_y / resolution_ + height_ / 2);

  // if (xmin < 0 || ymin < 0 || xmax >= height_ || ymax >= width_) {
  //     throw std::runtime_error("Polygon exceeds grid boundaries");
  // }

  // Draw the polygon
  for (Eigen::Index i = 0; i < vertices.cols(); ++i) {
    Eigen::Index j = (i + 1) % vertices.cols();
    std::cout << "i: " << i << " j: " << j << std::endl;
    int x0 = static_cast<int>(vertices(0, i) / resolution_ + width_ / 2);
    int y0 = static_cast<int>(vertices(1, i) / resolution_ + height_ / 2);
    int x1 = static_cast<int>(vertices(0, j) / resolution_ + width_ / 2);
    int y1 = static_cast<int>(vertices(1, j) / resolution_ + height_ / 2);

    draw_line(x0, y0, x1, y1, grid, value);
  }
}

void GridManager::draw_polygon(
    int x0, int y0, int x1, int y1,
    Eigen::Block<landmark_server::GridManager::Grid, -1, -1, false> &block,
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
  int xbounds = steep ? block.rows() : block.cols();
  int ybounds = steep ? block.cols() : block.rows();
  int y = y0;

  for (int x = x0; x <= x1; x++) {
    if (steep) {
      if (x >= 0 && x < xbounds && y >= 0 && y < ybounds) {
        block(y, x) += value;
      }
    } else {
      if (y >= 0 && y < ybounds && x >= 0 && x < xbounds) {
        block(x, y) += value;
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