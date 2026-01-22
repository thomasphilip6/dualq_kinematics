#include <gtest/gtest.h>
#include "dualq_kinematics/DualQuaternion.h"
#include "dualq_kinematics/PadenKahan.h"
#include <chrono>

constexpr double c_tolerance = 1e-3;
using FirstPadenKahan = dualq_kinematics::FirstPadenKahanProblem<double>;

TEST(dualq_kinematics, FirstPadenKahanProblemTest)
{
    //panda robot dimentions
    const double l_d1 = 0.333;
    const double l_d3 = 0.316;
    const double l_d5 = 0.384;
    //const double l_a5 = 0.0825;
    const double l_df = 0.107;
    const double l_a7 = 0.088;

    Eigen::Quaternion l_jointScrewAxis(0.0, 0.0, 0.0, 1.0);
    Eigen::Vector3d l_positionOnLine;
    Eigen::Vector3d l_startPoint;
    Eigen::Vector3d l_endPoint;
    l_positionOnLine << 0.0, 0.0, l_d1;
    l_startPoint << l_a7, 0.0, l_d1+l_d3+l_d5-l_df;
    l_endPoint << 0.0621879, 0.0622628, 0.926;

    FirstPadenKahan l_testResultFinite(l_positionOnLine,  l_jointScrewAxis, l_startPoint, l_endPoint);
    EXPECT_TRUE(l_testResultFinite.getResult().has_value()) << "Input on Paden Kahan 1st Problem should return a finite number";
    EXPECT_TRUE(FirstPadenKahan::compareFloatNum(0.786, l_testResultFinite.getResult().value(), c_tolerance)) << "Result of Paden Kahan Problem is Wrong";
    std::cout << "Result computed " << l_testResultFinite.getResult().value() << " rad" << std::endl;

    Eigen::Vector3d l_startPointOnLine;
    l_startPointOnLine << 0.0, 0.0, l_d1+l_d3+l_d5-l_df;
    FirstPadenKahan l_testResultInfinite1(l_positionOnLine, l_jointScrewAxis, l_startPointOnLine, l_endPoint);
    EXPECT_FALSE(l_testResultInfinite1.getResult().has_value()) << "Input with start on axis with Paden Kahan 1st Problem shouldn't return a finite number";

    FirstPadenKahan l_testResultInfinite2(l_positionOnLine, l_jointScrewAxis, l_endPoint, l_endPoint);
    EXPECT_FALSE(l_testResultInfinite2.getResult().has_value()) << "Input with start=end with Paden Kahan 1st Problem shouldn't return a finite number";
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
