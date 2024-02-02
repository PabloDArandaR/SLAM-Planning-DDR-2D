#include "math/linalg.hpp"
#include <gtest/gtest.h>

TEST(test_translation, test_translation_1) {
    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};

    Eigen::Vector3d original{Eigen::Vector3d::Ones()};
    Eigen::Vector3d relative_position{Eigen::Vector3d::Zero()};

    ASSERT_EQ(original + relative_position, linalg::transformPoint(original, relative_position, rotation));
}

TEST(test_translation, test_translation_2) {
    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};

    Eigen::Vector3d original{Eigen::Vector3d::Ones()};
    Eigen::Vector3d relative_position{Eigen::Vector3d::Zero()};

    relative_position = Eigen::Vector3d::Ones() + Eigen::Vector3d::Ones();
    ASSERT_EQ(original + relative_position, linalg::transformPoint(original, relative_position, rotation));
}

TEST(test_translation, test_translation_3) {
    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};

    Eigen::Vector3d original{Eigen::Vector3d::Ones()};
    Eigen::Vector3d relative_position{Eigen::Vector3d::Zero()};

    relative_position = -Eigen::Vector3d::Ones() - Eigen::Vector3d::Ones() - Eigen::Vector3d::Ones();
    ASSERT_EQ(original + relative_position, linalg::transformPoint(original, relative_position, rotation));
}

TEST(test_translation, test_translation_4) {
    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};

    Eigen::Vector3d original{Eigen::Vector3d::Ones()};
    Eigen::Vector3d relative_position{Eigen::Vector3d::Zero()};

    relative_position =
        Eigen::Vector3d::Ones() + Eigen::Vector3d::Ones() + Eigen::Vector3d::Ones() + Eigen::Vector3d::Ones();
    ASSERT_EQ(original + relative_position, linalg::transformPoint(original, relative_position, rotation));
}

TEST(test_translation, test_translation_5) {
    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};

    Eigen::Vector3d original{Eigen::Vector3d::Ones()};
    Eigen::Vector3d relative_position{Eigen::Vector3d::Zero()};

    relative_position = Eigen::Vector3d::Ones() + Eigen::Vector3d::Ones() + Eigen::Vector3d::Ones() +
                        Eigen::Vector3d::Ones() + Eigen::Vector3d::Ones() + Eigen::Vector3d::Ones() +
                        Eigen::Vector3d::Ones();
    ASSERT_EQ(original + relative_position, linalg::transformPoint(original, relative_position, rotation));
}

TEST(test_rotation, test_rot_90_x) {
    Eigen::Vector3d relative_position{Eigen::Vector3d::Zero()};

    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d original;
    original << 1, 2, 3;
    Eigen::Vector3d result{Eigen::Vector3d::Zero()};

    // 90 degree rotation in X
    rotation << 1, 0, 0, 0, 0, -1, 0, 1, 0;
    result << original(0), -original(2), original(1);

    ASSERT_EQ(result, linalg::transformPoint(original, relative_position, rotation));
}

TEST(test_rotation, test_rot_90_y) {
    Eigen::Vector3d relative_position{Eigen::Vector3d::Zero()};

    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d original;
    original << 1, 2, 3;
    Eigen::Vector3d result{Eigen::Vector3d::Zero()};

    rotation << 0, 0, 1, 0, 1, 0, -1, 0, 0;
    result << original(2), original(1), -original(0);

    ASSERT_EQ(result, linalg::transformPoint(original, relative_position, rotation));
}

TEST(test_rotation, test_rot_90_z) {
    Eigen::Vector3d relative_position{Eigen::Vector3d::Zero()};

    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d original;
    original << 1, 2, 3;
    Eigen::Vector3d result{Eigen::Vector3d::Zero()};

    rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    result << -original(1), original(0), original(2);

    ASSERT_EQ(result, linalg::transformPoint(original, relative_position, rotation));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}