#include <eigen3/Eigen/Dense>
#include <vector>
#include <list>

#include "mapping/mapping.hpp"
#include "mapping/scanner_metadata.hpp"

class noNoiseMethod : public mappingOccupancyGrid {
  public:
    noNoiseMethod();
    noNoiseMethod(occupancyGrid* map);

    void update(Eigen::Vector3d relative_position, Eigen::Vector4d relative_rotation,
                std::vector<float> laser_measurements);

    void set_scaner_metadata(scanerMetadata metadata);

    void fill_cells(std::list<Cell>& cell_list, float range_val);

  private:
    float angle_increment;
    scanerMetadata metadata;
};