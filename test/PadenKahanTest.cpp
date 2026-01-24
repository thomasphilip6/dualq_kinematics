#include <gtest/gtest.h>
#include "dualq_kinematics/DualQuaternion.h"
#include "dualq_kinematics/PadenKahan.h"
#include <chrono>

constexpr double c_tolerance = 1e-3;
using FirstPadenKahan = dualq_kinematics::FirstPadenKahanProblem<double>;
using SecondPadenKahan = dualq_kinematics::SecondPadenKahanProblem<double>;

TEST(dualq_kinematics, FirstPadenKahanProblemTest)
{
    //panda robot dimentions
    const double l_d1 = 0.333;
    const double l_d3 = 0.316;
    const double l_d5 = 0.384;
    //const double l_a5 = 0.0825;
    const double l_df = 0.107;
    const double l_a7 = 0.088;

    Eigen::Quaterniond l_jointScrewAxis(0.0, 0.0, 0.0, 1.0);
    Eigen::Quaterniond l_positionOnLine(0.0, 0.0, 0.0, l_d1);
    Eigen::Quaterniond l_startPoint(0.0, l_a7, 0.0, l_d1+l_d3+l_d5-l_df);
    Eigen::Quaterniond l_endPoint(0.0, 0.0621879, 0.0622628, 0.926);

    FirstPadenKahan l_testResultFinite(l_positionOnLine,  l_jointScrewAxis, l_startPoint, l_endPoint);
    EXPECT_TRUE(l_testResultFinite.getResult().has_value()) << "Input on Paden Kahan 1st Problem should return a finite number";
    EXPECT_TRUE(FirstPadenKahan::compareFloatNum(0.786, l_testResultFinite.getResult().value(), c_tolerance)) << "Result of Paden Kahan Problem is Wrong";
    std::cout << "Result computed " << l_testResultFinite.getResult().value() << " rad" << std::endl;

    Eigen::Quaterniond l_startPointOnLine(0.0, 0.0, 0.0, l_d1+l_d3+l_d5-l_df);
    FirstPadenKahan l_testResultInfinite1(l_positionOnLine, l_jointScrewAxis, l_startPointOnLine, l_endPoint);
    EXPECT_FALSE(l_testResultInfinite1.getResult().has_value()) << "Input with start on axis with Paden Kahan 1st Problem shouldn't return a finite number";

    FirstPadenKahan l_testResultInfinite2(l_positionOnLine, l_jointScrewAxis, l_endPoint, l_endPoint);
    EXPECT_FALSE(l_testResultInfinite2.getResult().has_value()) << "Input with start=end with Paden Kahan 1st Problem shouldn't return a finite number";
}

TEST(dualq_kinematics, SecondPadenKahanProblemTest)
{
    //panda robot dimentions
    const double l_d1 = 0.333;
    const double l_d3 = 0.316;
    const double l_d5 = 0.384;
    //const double l_a5 = 0.0825;
    const double l_df = 0.107;
    const double l_a7 = 0.088;

    const std::vector<double> l_expectedSigma1 = {0.35, -2.79159};
    const std::vector<double> l_expectedSigma2 = {-0.42, 0.125352};

    Eigen::Quaterniond l_joint1ScrewAxis(0.0, 0.0, 0.0, 1.0);
    Eigen::Quaterniond l_joint2ScrewAxis(0.0, 0.0, 1.0, 0.0);

    Eigen::Quaterniond l_startPoint(0.0, l_a7, 0.0, l_d1+l_d3+l_d5-l_df);
    Eigen::Quaterniond l_endPoint(0.0, -0.151662, -0.0553609, 0.910345);
    Eigen::Quaterniond l_intersectionPoint(0.0, 0.0, 0.0, l_d1);

    SecondPadenKahan l_testResultFinite(l_intersectionPoint, l_joint1ScrewAxis, l_joint2ScrewAxis, l_startPoint, l_endPoint);

    EXPECT_EQ(l_testResultFinite.getAngle1Result().size(), l_expectedSigma1.size());
    EXPECT_EQ(l_testResultFinite.getAngle2Result().size(), l_expectedSigma1.size());
    std::cout << "number of angle pairs " << l_testResultFinite.getAngle1Result().size() << std::endl;

    for (size_t i = 0; i < l_testResultFinite.getAngle1Result().size(); i++)
    {
        std::cout << "Angle 1 : " << l_testResultFinite.getAngle1Result().at(i) << " rad" << std::endl;
        EXPECT_TRUE(FirstPadenKahan::compareFloatNum(l_expectedSigma1.at(i), l_testResultFinite.getAngle1Result().at(i), 0.01));
        std::cout << "Angle 2 : " << l_testResultFinite.getAngle2Result().at(i) << " rad" << std::endl;
        EXPECT_TRUE(FirstPadenKahan::compareFloatNum(l_expectedSigma2.at(i), l_testResultFinite.getAngle2Result().at(i), 0.01));
    }
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
