#include <eigen3/Eigen/Dense>

namespace linalg {
    Eigen::Matrix3d quaternionToRotation(Eigen::Vector4d&);
    Eigen::Vector4d rotationToQuaternion(Eigen::Matrix3d&);
    Eigen::Matrix3d rotationX(double angle);
    Eigen::Matrix3d rotationY(double angle);
    Eigen::Matrix3d rotationZ(double angle);

    Eigen::Vector3d transformPoint(Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Matrix3d&);

    int sign(double r);
} // namespace linalg