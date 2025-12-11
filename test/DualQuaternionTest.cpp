#include <gtest/gtest.h>
#include "dualq_kinematics/DualQuaternion.h"
#include "dualq_kinematics/ScrewCoordinates.h"
#include <chrono>

using DualQuaternion = dualq_kinematics::DualQuaternion<double>;
constexpr double l_tolerance = 1e-6;

TEST(dualq_kinematics, ConstructionTest)
{
    //Construction with two Quaternions, one real and one dual part
    const Eigen::Quaterniond l_quaternionIdentity(1.0,0.0,0.0,0.0);
    DualQuaternion l_dualqTest(l_quaternionIdentity, l_quaternionIdentity);
    EXPECT_EQ(l_dualqTest.getDualPart(),l_dualqTest.getRealPart()) << "Construction of dualq by two quaternions for real and dual part fails";

    //Construction with AxisAngle for orientation and vector for position (first rotation then translation)
    const Eigen::AngleAxis<double> l_angleAxis(M_PI / 4, Eigen::Vector3d::UnitZ());
    DualQuaternion l_angleAxisDQ(l_angleAxis, Eigen::Translation<double, 3>(10,12,45));
    EXPECT_TRUE(l_angleAxisDQ.getRealPart().isApprox(Eigen::Quaterniond(0.923880, 0.0, 0.0, 0.382683), l_tolerance)) << "Construction of dualq by angle axis fails";

    //Construction with AxisAngle for translation first then orientation
    DualQuaternion l_angleAxisDQTransFirst(Eigen::Translation<double, 3>(10,12,45), l_angleAxis);
    EXPECT_TRUE(l_angleAxisDQTransFirst.getRealPart().isApprox(Eigen::Quaterniond(0.923880, 0.0, 0.0, 0.382683), l_tolerance)) << "Construction of dualq by angle axis fails";
    EXPECT_FALSE(l_angleAxisDQTransFirst.getDualPart().isApprox(l_angleAxisDQ.getDualPart(), l_tolerance)) << "Construction of dualq by angle axis fails";

    //Construction with ScrewAxis
    const Eigen::Translation<double, 3> l_screwAxis(0.101351, 0.244683, 0.964291);
    const Eigen::Translation<double, 3> l_position(0.0,0.5, 0.0);
    const Eigen::Quaterniond l_expectedReal(0.0, 0.101351, 0.244683, 0.964291);
    const Eigen::Quaterniond l_expectedDual(0.0, 0.5*0.964261, 0.0, -0.5*0.101351);
    const DualQuaternion l_screwAxisDQ(l_screwAxis, l_position);
    const DualQuaternion l_screwAxisDQExpected( l_expectedReal, l_expectedDual);
    EXPECT_TRUE(l_screwAxisDQ.isApprox(l_screwAxisDQExpected, 0.0001)) << "Construction of dualq by screw axis fails";

    //Construction with quat and translation for rotation then translation
    const DualQuaternion l_quatTrans(Eigen::Quaterniond(0.923880, 0.0, 0.0, 0.382683), Eigen::Translation<double, 3>(10,12,45));
    EXPECT_TRUE(l_quatTrans.isApprox(l_angleAxisDQ, l_tolerance)) << "Construction of dualq by quaternion and translation fails";

    //Construction with quat and translation for translation then rotation
    const DualQuaternion l_transQuat(Eigen::Translation<double, 3>(10,12,45), Eigen::Quaterniond(0.923880, 0.0, 0.0, 0.382683));
    EXPECT_TRUE(l_transQuat.isApprox(l_angleAxisDQTransFirst, l_tolerance)) << "Construction of dualq by quaternion and translation fails";

    //using transformation matrix
    Eigen::Isometry3d l_transform = Eigen::Translation3d(10, 12, 45) * Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ());
    const DualQuaternion l_transformDQ(l_transform);
    EXPECT_TRUE(l_quatTrans.isApprox(l_transformDQ, l_tolerance)) << "Construction of dualq by transformation matrix fails";
}

TEST(dualq_kinematics, isUnitTest)
{
    //using DualQuaternion = dualq_kinematics::DualQuaternion<double>;
    const Eigen::Quaterniond l_realPart(1.0,0.0,0.0,0.0);
    const Eigen::Quaterniond l_dualPart(0.0,1.0,0.0,0.0);
    const Eigen::Quaterniond l_realPartNotUnit(0.7,0.0,0.0,0.0);

    DualQuaternion l_dualqUnit(l_realPart,l_dualPart);
    bool l_isUnit = l_dualqUnit.isUnit(l_tolerance);
    EXPECT_EQ(l_isUnit,true) << "isUnit() (to detect if a dq is a unit one) fails";

    DualQuaternion l_dualqNotUnit(l_realPartNotUnit,l_dualPart);
    l_isUnit = l_dualqNotUnit.isUnit(l_tolerance);
    EXPECT_EQ(l_isUnit,false) << "isUnit() (to detect if a dq is a unit one) fails";
}

TEST(dualq_kinematics, isApproxTest)
{
    const Eigen::Quaterniond l_realPart(0.960634,0.027054,0.100967,0.257401);
    const Eigen::Quaterniond l_dualPart(0.960900,-0.014874,0.112098,0.252752);
    const Eigen::Quaterniond l_dualPartOther(0.965006,-0.042133,0.011290,0.258573);

    DualQuaternion l_dualq(l_realPart, l_dualPart);
    DualQuaternion l_dualqOther(l_realPart, l_dualPartOther);
    EXPECT_TRUE(l_dualq.isApprox(l_dualq,l_tolerance)) << "compare two quaternions doesn't detect equality of two dq";
    EXPECT_TRUE(l_dualqOther.isApprox(l_dualqOther,l_tolerance)) << "compare two quaternions doesn't detect equality of two dq";
    EXPECT_FALSE(l_dualq.isApprox(l_dualqOther, l_tolerance)) << "compare two quaternions doesn't detect inequality of two dq";
    EXPECT_FALSE(l_dualqOther.isApprox(l_dualq, l_tolerance)) << "compare two quaternions doesn't detect inequality of two dq";
}

TEST(dualq_kinematics, multiplicationTest)
{
    const Eigen::Quaterniond l_realPart(0.960634,0.027054,0.100967,0.257401);
    const Eigen::Quaterniond l_dualPart(0.960900,-0.014874,0.112098,0.252752);

    Eigen::Quaterniond l_realPartExpected(0.960634*2.3,0.027054*2.3,0.100967*2.3,0.257401*2.3);
    Eigen::Quaterniond l_dualPartExpected(0.960900*2.3,-0.014874*2.3,0.112098*2.3,0.252752*2.3);

    DualQuaternion l_dualq(l_realPart, l_dualPart);
    DualQuaternion l_dualqMultiplied = l_dualq*2.3;

    //DualQuaternion(l_realPartExpected, l_dualPartExpected).print();
    //l_dualqMultiplied.print();

    EXPECT_TRUE(l_dualqMultiplied.isApprox(DualQuaternion(l_realPartExpected, l_dualPartExpected), l_tolerance)) << "QDual uaternion multiplication with scalar fails";
    l_dualqMultiplied = l_dualq*1.7;
    EXPECT_FALSE(l_dualqMultiplied.isApprox(DualQuaternion(l_realPartExpected, l_dualPartExpected), l_tolerance)) << "Dual Quaternion multiplication with scalar fails";

    const DualQuaternion l_sigma1(Eigen::Quaterniond(2.0, 0.0, 0.0, 1.0),Eigen::Quaterniond(1.0, 1.0, 0.0, 1.0));
    const DualQuaternion l_sigma2(Eigen::Quaterniond( 1.0, 0.0, 0.0, 1.0),Eigen::Quaterniond(1.0, 1.0, 0.0, 0.0));
    const DualQuaternion l_productExpected(Eigen::Quaterniond(1.0, 0.0, 0.0, 3.0), Eigen::Quaterniond(2.0, 3.0, 0.0, 3.0));
    const DualQuaternion l_product = l_sigma1*l_sigma2;
    EXPECT_TRUE(l_product.isApprox(l_productExpected,l_tolerance)) << "Dual Quaternion multiplication fails";
}

TEST(dualq_kinematics, conjugateTest)
{
    const Eigen::Quaterniond l_realPart(1.0,0.3,0.2,0.6);
    const Eigen::Quaterniond l_dualPart(0.2,0.2,0.4,0.7);

    const Eigen::Quaterniond l_realPartConjugate(1.0,-0.3,-0.2,-0.6);
    const Eigen::Quaterniond l_dualPartConjugate(0.2,-0.2,-0.4,-0.7);

    const DualQuaternion l_dualq(l_realPart, l_dualPart);
    const DualQuaternion l_dualqExpectedConjugate(l_realPartConjugate,l_dualPartConjugate);
    const DualQuaternion l_conjugate = l_dualq.conjugate();
    const bool l_comparison = l_dualqExpectedConjugate.isApprox(l_conjugate,l_tolerance);
    EXPECT_TRUE(l_comparison) << "conjugate() fails";
}

TEST(dualq_kinematics, invertTest)
{
    const Eigen::Quaterniond l_realPart(1.0,0.0,0.0,0.0);
    const Eigen::Quaterniond l_dualPart(0.0,1.0,0.0,0.0);
    const Eigen::Quaterniond l_realPartConjugate(1.0,0.0,0.0,0.0);
    const Eigen::Quaterniond l_dualPartConjugate(0.0,-1.0,0.0,0.0);
    const Eigen::Quaterniond l_realPartNull(0.0,0.0,0.0,0.0);

    //todo test with non unit dual quaternion inverse
    DualQuaternion l_dualq(l_realPart, l_dualPart);
    DualQuaternion l_dualqNullRealPart(l_realPartNull, l_dualPart);
    const DualQuaternion l_dualqExpectedConjugate(l_realPartConjugate,l_dualPartConjugate);

    const bool l_result = l_dualq.invert(l_tolerance);
    const bool l_comparison = l_dualq.isApprox(l_dualqExpectedConjugate, l_tolerance);
    EXPECT_TRUE(l_result) << "Inverting dual quaternion fails";
    EXPECT_TRUE(l_comparison) << "Inverting dual quaternion fails";
    EXPECT_FALSE(l_dualqNullRealPart.invert(l_tolerance)) << "Inverting dual quaternion fails";
}

TEST(dualq_kinematics, inverseTest)
{
    const Eigen::Quaterniond l_realPart(1.0,0.0,0.0,0.0);
    const Eigen::Quaterniond l_dualPart(0.0,1.0,0.0,0.0);
    const Eigen::Quaterniond l_realPartConjugate(1.0,0.0,0.0,0.0);
    const Eigen::Quaterniond l_dualPartConjugate(0.0,-1.0,0.0,0.0);

    const Eigen::Quaterniond l_realPartNull(0.0,0.0,0.0,0.0);
    DualQuaternion l_dualq(l_realPart, l_dualPart);
    DualQuaternion l_dualqNullRealPart(l_realPartNull, l_dualPart);
    const DualQuaternion l_dualqExpectedConjugate(l_realPartConjugate,l_dualPartConjugate);

    std::optional<DualQuaternion> l_inverse = l_dualq.inverse(l_tolerance);
    bool l_comparison = l_inverse.value().isApprox(l_dualqExpectedConjugate, l_tolerance);
    EXPECT_TRUE(l_comparison) << "Inverting dual quaternion fails";
    EXPECT_TRUE(l_inverse.has_value()) << "Inverting dual quaternion fails";
    EXPECT_FALSE(l_dualqNullRealPart.inverse(l_tolerance).has_value()) << "Inverting dual quaternion fails";
}

TEST(dualq_kinematics, getterTest)
{
    const Eigen::AngleAxis<double> l_angleAxis(M_PI / 4, Eigen::Vector3d::UnitZ());
    const Eigen::Quaterniond l_expectedQuaternion(0.923880, 0.0, 0.0, 0.382683);
    const Eigen::Translation<double, 3> l_translation(10.458, 12.957, 45.364);
    const Eigen::Transform<double,3, Eigen::Isometry> l_transform = Eigen::Translation3d(l_translation.x(), l_translation.y(), l_translation.z()) * l_angleAxis;
    DualQuaternion l_angleAxisDQ(l_angleAxis, l_translation);
    DualQuaternion l_angleAxisDQTransFirst(l_translation, l_angleAxis);
    EXPECT_TRUE(l_translation.isApprox(l_angleAxisDQ.getTranslation(true), l_tolerance)) << "getTranslation() out of dualq with rotation first fails";
    EXPECT_TRUE(l_translation.isApprox(l_angleAxisDQTransFirst.getTranslation(false), l_tolerance)) << "getTranslation() out of dualq with translation first fails";
    EXPECT_TRUE(l_expectedQuaternion.isApprox(l_angleAxisDQ.getRotation(), l_tolerance)) << "getRotation() out of dualq fails";
    EXPECT_TRUE(l_transform.isApprox(l_angleAxisDQ.getTransform(), l_tolerance)) << "getTransform() out of dualq fails";
    EXPECT_FALSE(l_transform.isApprox(l_angleAxisDQTransFirst.getTransform(), l_tolerance)) << "getTransform() out of dualq fails";
    EXPECT_TRUE(l_transform.linear().isApprox(l_angleAxisDQ.getRotationMatrix(), l_tolerance)) << "getRotationMatrix() out of dualq fails";
}

TEST(dualq_kinematics, quaternionExpTest)
{
    const Eigen::Quaterniond l_quat(1.0, 1.0, 0.0, 0.0);
    const Eigen::Quaterniond l_test = dualq_kinematics::quaternionExp(l_quat);
    const Eigen::Quaterniond l_expected(exp(1)*cos(1), 1.0*sin(1), 0, 0);
    EXPECT_TRUE(l_expected.isApprox(l_test, l_tolerance)) << "Quaternion exponential fails";
}

TEST(dualq_kinematics, exponentialTest)
{
    using Translation = Eigen::Translation<double, 3>;
    const double l_1 = 0.4;
    const double l_2 = 1.12;
    const double l_3 = 1.76;
    const double l_4 = 2.1;
    std::array<DualQuaternion, 7> l_joints = {
        DualQuaternion(Translation(0.0, 0.0, 1.0), Translation(0.0, 0.0, 0.0)),
        DualQuaternion(Translation(0.0, 1.0, 0.0), Translation(0.0, 0.0, l_1)),
        DualQuaternion(Translation(0.0, 0.0, 1.0), Translation(0.0, 0.0, l_1)),
        DualQuaternion(Translation(0.0, 1.0, 0.0), Translation(0.0, 0.0, l_2)),
        DualQuaternion(Translation(0.0, 0.0, 1.0), Translation(0.0, 0.0, l_2)),
        DualQuaternion(Translation(0.0, 1.0, 0.0), Translation(0.0, 0.0, l_3)),
        DualQuaternion(Translation(0.0, 0.0, 1.0), Translation(0.0, 0.0, l_3))
    };
    DualQuaternion l_ee(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), Translation(0.0, 0.0, l_4));

    std::array<double, 7> l_jointValues = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    auto l_start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < l_joints.size(); i++)
    {
        l_joints.at(i) = l_joints.at(i) * l_jointValues.at(i) * 0.5;
    }
    const DualQuaternion l_forwardKinematics = l_joints.at(0).dqExp() * l_joints.at(1).dqExp() * l_joints.at(2).dqExp() * l_joints.at(3).dqExp() * l_joints.at(4).dqExp() * l_joints.at(5).dqExp() * l_joints.at(6).dqExp() * l_ee;
    auto l_stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> l_ms = l_stop - l_start;
    std::cout << "Forward Kinematics with dual Quaternions took : " << l_ms.count() << " ms" << std::endl;
    DualQuaternion l_expected(Eigen::Quaterniond(-0.701, -0.000, 0.658, 0.275), Eigen::Quaterniond(-0.519693, -0.0389178, -0.422547, -0.313443));
    l_forwardKinematics.print();
    std::cout << " ------ " << std::endl;
    std::cout << l_forwardKinematics.getTransform().matrix() << std::endl;
    std::cout << " ------ " << std::endl;
    std::cout << l_forwardKinematics.getTranslation(true).x() << std::endl;
    std::cout << l_forwardKinematics.getTranslation(true).y() << std::endl;
    std::cout << l_forwardKinematics.getTranslation(true).z() << std::endl;
    EXPECT_TRUE(l_forwardKinematics.isApprox(l_expected, 0.001));
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}