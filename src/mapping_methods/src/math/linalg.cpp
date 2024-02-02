#include "math/linalg.hpp"
#include <cmath>

int linalg::sign(double r){
    if (r > 0){
        return 1;
    }
    else if (r < 0) {
        return -1;
    }
    else {
        return 0;
    }
}

Eigen::Matrix3d linalg::quaternionToRotation(Eigen::Vector4d& quaternion) {
    Eigen::Matrix3d output;

    double s {2/(std::pow(quaternion.coeff(0), 2) + std::pow(quaternion.coeff(1), 2) + std::pow(quaternion.coeff(2), 2) + std::pow(quaternion.coeff(3), 2))};
    double a{quaternion.coeff(0)}, b {quaternion.coeff(1)}, c{quaternion.coeff(2)}, d{quaternion.coeff(3)};
    double bs {b*s}, cs {c*s}, ds{d*s};
    double ab {a*bs}, ac {a*cs}, ad{a*ds};
    double bb {b*bs}, bc {b*cs}, bd{b*ds};
    double cc {c*cs}, cd {c*ds}, dd{d*ds};

    output.coeffRef(0, 0) = 1 - cc - dd;
    output.coeffRef(0, 1) = bc - ad;
    output.coeffRef(0, 2) = bd + ac;

    output.coeffRef(1, 0) = bc + ad;
    output.coeffRef(1, 1) = 1 - bb - dd;
    output.coeffRef(1, 2) = cd - ab;

    output.coeffRef(2, 0) = bd - ac;
    output.coeffRef(2, 1) = cd + ab;
    output.coeffRef(2, 2) = 1 - bb - cc ;

    return output;
}

Eigen::Vector4d linalg::rotationToQuaternion(Eigen::Matrix3d& rotation) {
    Eigen::Vector4d output;

    output.coeffRef(0) = std::sqrt(1 + rotation.coeff(0, 0) + rotation.coeff(1, 1) + rotation.coeff(2, 2)) / 2;
    output.coeffRef(1) = linalg::sign(rotation.coeff(2, 1) - rotation.coeff(1, 2)) * std::sqrt(1 + rotation.coeff(0, 0) - rotation.coeff(1, 1) - rotation.coeff(2, 2));
    output.coeffRef(2) = linalg::sign(rotation.coeff(0, 2) - rotation.coeff(2, 0)) * std::sqrt(1 - rotation.coeff(0, 0) + rotation.coeff(1, 1) - rotation.coeff(2, 2));
    output.coeffRef(1) = linalg::sign(rotation.coeff(1, 0) - rotation.coeff(0, 1)) * std::sqrt(1 - rotation.coeff(0, 0) - rotation.coeff(1, 1) + rotation.coeff(2, 2));

    return output;
}

Eigen::Matrix3d linalg::rotationX(double angle) {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation.coeffRef(1, 1) = std::cos(angle);
    rotation.coeffRef(1, 2) = -std::sin(angle);
    rotation.coeffRef(2, 2) = std::cos(angle);
    rotation.coeffRef(2, 1) = std::sin(angle);

    return rotation;
};

Eigen::Matrix3d linalg::rotationY(double angle) {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation.coeffRef(0, 0) = std::cos(angle);
    rotation.coeffRef(2, 0) = -std::sin(angle);
    rotation.coeffRef(2, 2) = std::cos(angle);
    rotation.coeffRef(0, 2) = std::sin(angle);

    return rotation;
};

Eigen::Matrix3d linalg::rotationZ(double angle) {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation.coeffRef(0, 0) = std::cos(angle);
    rotation.coeffRef(0, 1) = -std::sin(angle);
    rotation.coeffRef(1, 1) = std::cos(angle);
    rotation.coeffRef(1, 0) = std::sin(angle);

    return rotation;
};

Eigen::Vector3d linalg::transformPoint(Eigen::Vector3d& original_point, Eigen::Vector3d& relative_position,
                                       Eigen::Matrix3d& rotation_matrix) {
    return rotation_matrix * original_point + relative_position;
};