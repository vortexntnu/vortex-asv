#include <Eigen/Dense>
#include <cstdint>
#include <tuple>
#include <vector>

namespace landmark_server {

enum class ShapeType : uint8_t { SPHERE = 0, PRISM = 1 };

struct Circle {
  float radius;
  float x_centre;
  float y_centre;
};

struct Point {
  float x;
  float y;
  float z;
};

struct Polygon {
  std::vector<Point> vertices;
};

struct Pose {
  float x;
  float y;
  float yaw;
};

struct ShapeInfo {
  ShapeType type;
  Circle circle;
  Polygon polygon;
  Pose pose;
};

class GridManager {

public:
  using Grid = Eigen::Array<int8_t, Eigen::Dynamic, Eigen::Dynamic>;

  // GridManager(std::vector<int8_t> grid, float resolution, uint32_t height,
  // uint32_t width);
  GridManager(float resolution, uint32_t height, uint32_t width);
  ~GridManager() = default;

  const Grid &get_grid() const;

  void update_grid(int8_t *grid,
                   const Eigen::Array<float, 2, Eigen::Dynamic> &polygon,
                   int value);

  std::tuple<int, int> world_to_grid(float x, float y);

  void handle_circle(int xc, int yc, int r, int value);

  void draw_circle(
      int xc, int yc, int x, int y, int value,
      Eigen::Block<landmark_server::GridManager::Grid, -1, -1, false> &block,
      int xmin, int ymin);

  void handle_polygon(int8_t *grid,
                      const Eigen::Array<float, 2, Eigen::Dynamic> &vertices,
                      int value);

  void draw_polygon(
      int x0, int y0, int x1, int y1,
      Eigen::Block<landmark_server::GridManager::Grid, -1, -1, false> &block,
      int value);
  void draw_line(int x0, int y0, int x1, int y1, int8_t *grid, int value);

private:
  Grid grid_;
  float resolution_;
  uint32_t height_;
  uint32_t width_;
};

} // namespace landmark_server