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

  /**
   * @brief Update the grid with a polygon and a value to update corresponding cells with.
   */
  void update_grid(int8_t *grid,
                   const Eigen::Array<float, 2, Eigen::Dynamic> &vertices,
                   int value);

  /**
   * @brief Draw a line on the grid and update the value of the cells it passes through.
   */
  void draw_line(int x0, int y0, int x1, int y1, int8_t *grid, int value);

  /**
   * @brief Fill a polygon on the grid and update the value of the cells it covers.
   */
  void fill_polygon(int8_t *grid,
                    const Eigen::Array<float, 2, Eigen::Dynamic> &polygon,
                    int value);

  /**
   * @brief Check if a point is inside a polygon.
   */
  bool point_in_polygon(int x, int y,
                        const Eigen::Array<float, 2, Eigen::Dynamic> &polygon);

private:
  float resolution_;
  uint32_t height_;
  uint32_t width_;
};

} // namespace landmark_server