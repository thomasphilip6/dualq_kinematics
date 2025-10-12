#include <gtest/gtest.h>
#include "dualq_kinematics/DualQuaternion.h"

TEST(dualq_kinematics, ConstructionTest)
{
    using Quaternion = Eigen::Quaterniond;
    using DualQuaternion = dualq_kinematics::DualQuaternion<double>;
    Quaternion l_quaternionIdentity(1.0,0.0,0.0,0.0);

    DualQuaternion l_dualqTest(l_quaternionIdentity, l_quaternionIdentity);
    EXPECT_EQ(l_dualqTest.getDualPart(),l_dualqTest.getRealPart) << "Construction of dualq by two quaternions for real and dual part fails";

}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}