#include "math/linalg.hpp"
#include <gtest/gtest.h>

TEST(test_, test_translation_1) {
    Eigen::Matrix4f rotation{Eigen::Matrix4f::Identity()};

    Eigen::Vector4f original{Eigen::Vector4f::Ones()};
    Eigen::Vector4f relative_position{Eigen::Vector4f::Zero()};

    ASSERT_EQ(original + relative_position, linalg::transformPoint(original, relative_position, rotation));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}