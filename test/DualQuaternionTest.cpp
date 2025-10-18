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

TEST(dualq_kinematics, compareTest)
{
    std::cout << "compareTest" << std::endl;
    const Eigen::Quaterniond l_realPart(0.960634,0.027054,0.100967,0257401);
    const Eigen::Quaterniond l_dualPart(0.960900,-0.014874,0.112098,0.252752);
    const Eigen::Quaterniond l_dualPartOther(0.965006,-0.042133,0.011290,0.258573);

    DualQuaternion l_dualq(l_realPart, l_dualPart);
    DualQuaternion l_dualqOther(l_realPart, l_dualPartOther);
    EXPECT_TRUE(l_dualq.compare(l_dualq,l_tolerance)) << "compare two quaternions doesn't detect equality of two dq";
    EXPECT_TRUE(l_dualqOther.compare(l_dualqOther,l_tolerance)) << "compare two quaternions doesn't detect equality of two dq";
    EXPECT_FALSE(l_dualq.compare(l_dualqOther, l_tolerance)) << "compare two quaternions doesn't detect inequality of two dq";
    EXPECT_FALSE(l_dualqOther.compare(l_dualq, l_tolerance)) << "compare two quaternions doesn't detect inequality of two dq";
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
    const bool l_comparison = l_dualqExpectedConjugate.compare(l_conjugate,l_tolerance);
    EXPECT_TRUE(l_comparison) << "conjugate() fails";

    std::cout << "Initial dualq ";
    l_dualq.print();
    std::cout << "Expected conjugate is ";
    l_dualqExpectedConjugate.print();
    std::cout << "Obtained conjugate is";
    l_conjugate.print();
}

TEST(dualq_kinematics, invertTest)
{
    std::cout << "invertTest" << std::endl;
    const Eigen::Quaterniond l_realPart(1.0,0.0,0.0,0.0);
    const Eigen::Quaterniond l_dualPart(0.0,1.0,0.0,0.0);
    const Eigen::Quaterniond l_realPartConjugate(1.0,0.0,0.0,0.0);
    const Eigen::Quaterniond l_dualPartConjugate(0.0,-1.0,0.0,0.0);
    const Eigen::Quaterniond l_realPartNull(0.0,0.0,0.0,0.0);

    //todo test with non unit dual quaternion inverse
    DualQuaternion l_dualq(l_realPart, l_dualPart);
    DualQuaternion l_dualqNullRealPart(l_realPartNull, l_dualPart);
    const DualQuaternion l_dualqExpectedConjugate(l_realPartConjugate,l_dualPartConjugate);

    const bool l_result = l_dualq.invert();
    const bool l_comparison = l_dualq.compare(l_dualqExpectedConjugate, l_tolerance);
    EXPECT_TRUE(l_result);
    EXPECT_TRUE(l_comparison);
    EXPECT_FALSE(l_dualqNullRealPart.invert());

    std::cout << "Expected inverse is ";
    l_dualqExpectedConjugate.print();
    std::cout << "Obtained inverse is";
    l_dualq.print();
}

TEST(dualq_kinematics, inverseTest)
{
    std::cout << "inverseTest" << std::endl;
    const Eigen::Quaterniond l_realPart(1.0,0.0,0.0,0.0);
    const Eigen::Quaterniond l_dualPart(0.0,1.0,0.0,0.0);
    const Eigen::Quaterniond l_realPartConjugate(1.0,0.0,0.0,0.0);
    const Eigen::Quaterniond l_dualPartConjugate(0.0,-1.0,0.0,0.0);

    const Eigen::Quaterniond l_realPartNull(0.0,0.0,0.0,0.0);
    DualQuaternion l_dualq(l_realPart, l_dualPart);
    DualQuaternion l_dualqNullRealPart(l_realPartNull, l_dualPart);
    const DualQuaternion l_dualqExpectedConjugate(l_realPartConjugate,l_dualPartConjugate);

    std::optional<DualQuaternion> l_inverse = l_dualq.inverse();
    const bool l_comparison = l_inverse.value().compare(l_dualqExpectedConjugate, l_tolerance);
    EXPECT_TRUE(l_comparison);
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}