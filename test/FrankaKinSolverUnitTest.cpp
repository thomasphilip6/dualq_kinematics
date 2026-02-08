#include <gtest/gtest.h>
#include "dualq_kinematics/DualQuaternion.h"
#include "dualq_kinematics/ScrewCoordinates.h"
#include "dualq_kinematics/PadenKahan.h"
#include <chrono>
#include "dualq_kinematics/FrankaKinSolver.h"

using DualQuaternion = dualq_kinematics::DualQuaternion<double>;
using FirstPadenKahan = dualq_kinematics::FirstPadenKahanProblem<double>;
using Translation = Eigen::Translation<double, 3>;
using Quaternion = Eigen::Quaterniond;
using Vector3 = Eigen::Matrix<double, 3, 1>;
using FrankaKinSolver = dualq_kinematics::FrankaKinSolver<double>;
constexpr double c_tolerance = 1e-4;

TEST(dualq_kinematics, computeWristPositionTest)
{
    //Construction
    const std::vector<Vector3> l_screwAxesRef = {
        Vector3(0.0, 0.0, 1.0),
        Vector3(0.0, 1.0, 0.0),
        Vector3(0.0, 0.0, 1.0),
        Vector3(0.0, -1.0, 0.0),
        Vector3(0.0, 0.0, 1.0),
        Vector3(0.0, -1.0, 0.0),
        Vector3(0.0, 0.0, -1.0),
    };

    const double l_d1 = 0.333;
    const double l_d3 = 0.316;
    const double l_d5 = 0.384;
    const double l_a5 = 0.0825;
    const double l_df = 0.107;
    const double l_a7 = 0.088;

    const std::vector<Vector3> l_positionsRef = {
        Vector3(0.0, 0.0, l_d1),
        Vector3(0.0, 0.0, l_d1),
        Vector3(0.0, 0.0, l_d1+l_d3),
        Vector3(l_a5, 0.0, l_d1+l_d3),
        Vector3(0.0, 0.0, l_d1+l_d3+l_d5),
        Vector3(0.0, 0.0, l_d1+l_d3+l_d5),
        Vector3(l_a7, 0.0, l_d1+l_d3+l_d5),
    };

    const Eigen::Isometry3d l_tip2BaseAtRestRef = Eigen::Translation3d(l_a7, 0.0, l_d1+l_d3+l_d5-l_df) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

    FrankaKinSolver l_testSolver(l_screwAxesRef, l_positionsRef, l_tip2BaseAtRestRef);

    Eigen::Isometry3d l_eeWanted = Eigen::Isometry3d::Identity();
    l_eeWanted.translation() << -0.348869, 0.46065, 0.947152;
    l_eeWanted.linear() <<
    -0.0703608,  0.662334,  -0.745898,
    0.331076,   0.72087,    0.60888,
    0.940977,  -0.204108,  -0.270004;

    Vector3 l_wristResult;
    Vector3 l_expected(-0.262866, 0.366365, 0.893237);
    l_testSolver.computeWristPosition(l_eeWanted, 0.0, l_wristResult);
    EXPECT_TRUE(l_wristResult.isApprox(l_expected, c_tolerance)) << "Wrist found false";

    l_eeWanted.translation() << -0.348869, 0.46065, 0.947152;
    l_eeWanted.linear() <<
    0.418382,  0.51826, -0.745898,
    0.743729, 0.275922,   0.60888,
    0.521368,   -0.80949, -0.270004;

    l_testSolver.computeWristPosition(l_eeWanted, 0.785, l_wristResult);
    EXPECT_TRUE(l_wristResult.isApprox(l_expected, c_tolerance)) << "Wrist found false";
    


}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
