#include <Eigen/Dense>
#include <cstdint>
#include <tuple>
#include <vector>

namespace landmark_server {

/**
 * @class GridManager
 * @brief A class for managing the grid map.
 */
class GridManager {

public:
  GridManager(float resolution, uint32_t height, uint32_t width);

  ~GridManager() = default;

  void update_grid(int8_t *grid,
                   const Eigen::Array<float, 2, Eigen::Dynamic> &polygon,
                   int value);

  void handle_polygon(int8_t *grid,
                      const Eigen::Array<float, 2, Eigen::Dynamic> &vertices,
                      int value);

  void draw_line(int x0, int y0, int x1, int y1, int8_t *grid, int value);

  void fill_polygon(int8_t *grid,
                               const Eigen::Array<float, 2, Eigen::Dynamic> &polygon,
                               int value);

  bool point_in_polygon(int x, int y, const Eigen::Array<float, 2, Eigen::Dynamic> &polygon);


private:
  float resolution_;
  uint32_t height_;
  uint32_t width_;
};

} // namespace landmark_server