#include <eigen3/Eigen/Dense>
#include <vector>

#include "mapping/mapping.hpp"
#include "mapping/scaner_metadata.hpp"

class noNoiseMethod : public mappingOccupancyGrid {
    public:
        noNoiseMethod();
        noNoiseMethod(occupancyGrid*);

        void update(Eigen::Vector3f, Eigen::Vector4f, std::vector<float>);

        void set_scaner_metadata(scanerMetadata);

    private:
        float angle_increment;
        scanerMetadata metadata;
};