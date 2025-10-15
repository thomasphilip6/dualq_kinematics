#include <gtest/gtest.h>
#include "dualq_kinematics/DualQuaternion.h"

TEST(dualq_kinematics, ConstructionTest)
{
    
    using DualQuaternion = dualq_kinematics::DualQuaternion<double>;
    const Eigen::Quaterniond l_quaternionIdentity(1.0,0.0,0.0,0.0);
    //Eigen::Quaterniond l_quaternion(0.965926, 0.258819, 0.0, 0.0); to test if gtest is properly used

    DualQuaternion l_dualqTest(l_quaternionIdentity, l_quaternionIdentity);
    EXPECT_EQ(l_dualqTest.getDualPart(),l_dualqTest.getRealPart()) << "Construction of dualq by two quaternions for real and dual part fails";

}

TEST(dualq_kinematics, isUnitTest)
{
    using DualQuaternion = dualq_kinematics::DualQuaternion<double>;
    const Eigen::Quaterniond l_realPart(1.0,0.0,0.0,0.0);
    const Eigen::Quaterniond l_dualPart(0.0,1.0,0.0,0.0);// must be unit so w=0
    const double l_tolerance = 0.01;

    DualQuaternion l_dualqUnit(l_realPart,l_dualPart);
    EXPECT_EQ(l_dualqUnit.isUnit(l_tolerance),true) << "isUnit() (to detect if a dq is a unit one) fails";

    DualQuaternion l_dualqNotUnit(l_realPart,l_realPart);
    EXPECT_EQ(l_dualqNotUnit.isUnit(l_tolerance),false) << "isUnit() (to detect if a dq is a unit one) fails";

}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}