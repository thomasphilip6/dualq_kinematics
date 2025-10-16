#include <gtest/gtest.h>
#include "dualq_kinematics/DualQuaternion.h"

using DualQuaternion = dualq_kinematics::DualQuaternion<double>;
const double l_tolerance = 1e-6;

TEST(dualq_kinematics, ConstructionTest)
{
    
    //using DualQuaternion = dualq_kinematics::DualQuaternion<double>;
    const Eigen::Quaterniond l_quaternionIdentity(1.0,0.0,0.0,0.0);
    //Eigen::Quaterniond l_quaternion(0.965926, 0.258819, 0.0, 0.0); to test if gtest is properly used

    DualQuaternion l_dualqTest(l_quaternionIdentity, l_quaternionIdentity);
    EXPECT_EQ(l_dualqTest.getDualPart(),l_dualqTest.getRealPart()) << "Construction of dualq by two quaternions for real and dual part fails";

}

TEST(dualq_kinematics, isUnitTest)
{
    std::cout << "isUnitTest" << std::endl;
    //using DualQuaternion = dualq_kinematics::DualQuaternion<double>;
    const Eigen::Quaterniond l_realPart(1.0,0.0,0.0,0.0);
    const Eigen::Quaterniond l_dualPart(0.0,1.0,0.0,0.0);
    const Eigen::Quaterniond l_realPartNotUnit(0.7,0.0,0.0,0.0);

    DualQuaternion l_dualqUnit(l_realPart,l_dualPart);
    bool l_isUnit = l_dualqUnit.isUnit(l_tolerance);
    EXPECT_EQ(l_isUnit,true) << "isUnit() (to detect if a dq is a unit one) fails";
    std::cout << "isUnitTest, expect true : " << l_isUnit << std::endl;

    DualQuaternion l_dualqNotUnit(l_realPartNotUnit,l_dualPart);
    l_isUnit = l_dualqNotUnit.isUnit(l_tolerance);
    EXPECT_EQ(l_isUnit,false) << "isUnit() (to detect if a dq is a unit one) fails";
    std::cout << "isUnitTest, expect false : " << l_isUnit << std::endl;

}

TEST(dualq_kinematics, conjugateTest)
{
    std::cout << "conjugateTest" << std::endl;
    const Eigen::Quaterniond l_realPart(1.0,0.3,0.2,0.6);
    const Eigen::Quaterniond l_dualPart(0.2,0.2,0.4,0.7);

    const Eigen::Quaterniond l_realPartConjugate(1.0,-0.3,-0.2,-0.6);
    const Eigen::Quaterniond l_dualPartConjugate(0.2,-0.2,-0.4,-0.7);

    const DualQuaternion l_dualq(l_realPart, l_dualPart);
    const DualQuaternion l_dualqExpectedConjugate(l_realPartConjugate,l_dualPartConjugate);
    const DualQuaternion l_conjugate = l_dualq.conjugate();
    const bool l_comparison = l_dualq.compare(l_conjugate,l_tolerance);
    EXPECT_EQ(l_comparison, true) << "conjugate() fails";
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}