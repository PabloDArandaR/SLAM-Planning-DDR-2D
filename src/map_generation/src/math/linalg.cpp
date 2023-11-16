#include "math/linalg.hpp"
#include <cmath>

Eigen::Matrix3f quaternionToRotation(Eigen::Vector4f quaternion) {
    Eigen::Matrix3d output;

    output.coeffRef(0, 0) = 2 * (std::pow(quaternion.coeffRef(0), 2) + std::pow(quaternion.coeffRef(1), 2)) - 1;
    output.coeffRef(0, 1) =
        2 * (quaternion.coeffRef(1) * quaternion.coeffRef(2) - quaternion.coeffRef(0) * quaternion.coeffRef(3));
    output.coeffRef(0, 2) =
        2 * (quaternion.coeffRef(1) * quaternion.coeffRef(3) + quaternion.coeffRef(0) * quaternion.coeffRef(2));

    output.coeffRef(1, 0) =
        2 * (quaternion.coeffRef(1) * quaternion.coeffRef(2) + quaternion.coeffRef(0) * quaternion.coeffRef(3));
    output.coeffRef(1, 1) = 2 * (std::pow(quaternion.coeffRef(0), 2) + std::pow(quaternion.coeffRef(2), 2)) - 1;
    output.coeffRef(1, 2) =
        2 * (quaternion.coeffRef(2) * quaternion.coeffRef(3) - quaternion.coeffRef(0) * quaternion.coeffRef(1));

    output.coeffRef(2, 0) =
        2 * (quaternion.coeffRef(1) * quaternion.coeffRef(3) - quaternion.coeffRef(0) * quaternion.coeffRef(3));
    output.coeffRef(2, 1) =
        2 * (quaternion.coeffRef(2) * quaternion.coeffRef(3) + quaternion.coeffRef(0) * quaternion.coeffRef(1));
    output.coeffRef(2, 2) = 2 * (std::pow(quaternion.coeffRef(0), 2) + std::pow(quaternion.coeffRef(3), 2)) - 1;

    return output;
}