#include <gtest/gtest.h>
#include "dualq_kinematics/DualQuaternion.h"
#include "dualq_kinematics/ScrewCoordinates.h"
#include "dualq_kinematics/FrankaKinSolver.h"

//ROS2
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/utils/robot_model_test_utils.h>

//C++
#include <map>
#include <string>
#include <memory>

//GeoFIK
//#include "dualq_kinematics/geofik.h"

const std::string c_pandaTipLink = "panda_link8";
constexpr int c_repetitions = 1000;
constexpr double c_tolerance = 1e-5;
constexpr double c_toleranceLow = 1e-3;

using ScrewCoordinates = dualq_kinematics::ScrewCoordinates<double>;
using FrankaKinSolver = dualq_kinematics::FrankaKinSolver<double>;
using Vector3 = Eigen::Matrix<double, 3, 1>; 

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("FrankaKinSolverTest");

void printEigenIsometry(const Eigen::Isometry3d& p_transformationMatrix)
{
    RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << p_transformationMatrix.translation() << "\n");
    RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << p_transformationMatrix.rotation() << "\n");
}

void printJointVector(std::vector<double>& vector)
{
    RCLCPP_INFO_STREAM(LOGGER, 
        " " << vector.at(0) <<
        " " << vector.at(1) <<
        " " << vector.at(2) <<
        " " << vector.at(3) <<
        " " << vector.at(4) <<
        " " << vector.at(5) <<
        " " << vector.at(6)
        << "\n"
    );
}

TEST(dualq_kinematics, FrankaKinSolverTest)
{
    auto const node = std::make_shared<rclcpp::Node>(
        "FrankaKinSolver_test",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    RCLCPP_INFO_STREAM(LOGGER, "Loading robot model from " << node->get_name() << "." << ROBOT_DESCRIPTION_PARAM);
    std::cout << "Retrieving Robot Model" << std::endl;
    rdf_loader::RDFLoader rdf_loader(node, ROBOT_DESCRIPTION_PARAM);
    moveit::core::RobotModelPtr l_robotModel = std::make_shared<moveit::core::RobotModel>(rdf_loader.getURDF(), rdf_loader.getSRDF());
    ASSERT_TRUE(bool(l_robotModel)) << "Failed to load robot model";

    ScrewCoordinates l_screwCoord(*l_robotModel, c_pandaTipLink);
    const ScrewCoordinates* l_screwCoordPtr = &l_screwCoord;

    FrankaKinSolver l_frankaKin(l_screwCoordPtr);

    //Constructing Robot State
    moveit::core::RobotStatePtr l_robotState(new moveit::core::RobotState(l_robotModel));
    l_robotState->enforceBounds();
    l_robotState->setToDefaultValues();
    const moveit::core::JointModelGroup* l_jointModelGroup = l_robotModel->getJointModelGroup("panda_arm");

    // ------------ FK Test ---------------- //
    std::vector<double> l_jointValuesReady_rad ={0.14, -0.785, 0, -2.356, 0, 1.571, 0.785};
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    Eigen::Isometry3d l_computedResult;
    l_frankaKin.computeTipFK(l_jointValuesReady_rad, l_computedResult);
    const Eigen::Isometry3d& l_eeStateMoveIt1 = l_robotState->getGlobalLinkTransform("panda_link8");
    printEigenIsometry(l_eeStateMoveIt1);
    printEigenIsometry(l_computedResult);
    EXPECT_EQ(l_eeStateMoveIt1.isApprox(l_computedResult, c_tolerance), 1) << "FK fails";

    l_jointValuesReady_rad ={0.14, -0.0, 0, -2.356, 0, 1.421, 0.785};
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    auto l_start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < c_repetitions; i++)
    {
        l_frankaKin.computeTipFK(l_jointValuesReady_rad, l_computedResult);
    }
    auto l_stop = std::chrono::high_resolution_clock::now();
    double l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count() / c_repetitions;
    RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver FK took (over 1000 repetitions) " << l_micros << " us.");
    
    EXPECT_EQ(l_robotState->getGlobalLinkTransform("panda_link8").isApprox(l_computedResult, c_tolerance), 1) << "FK fails";

    // ------------ computeWristPosition Test ---------------- //
    
    l_jointValuesReady_rad ={2.13528, 0.45, 0.16, -0.42, 0.18, 2.14, 0.0};;
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    const Eigen::Isometry3d& l_eeWanted1 = l_robotState->getGlobalLinkTransform("panda_link8"); 

    Vector3 l_wrist;
    l_frankaKin.computeWristPosition(l_eeWanted1, l_jointValuesReady_rad.at(6), l_wrist);
    EXPECT_TRUE(l_wrist.isApprox(l_robotState->getGlobalLinkTransform("panda_link6").translation(), c_tolerance)) << "Compute Wrist fails";

    l_jointValuesReady_rad ={2.13528, 0.45, 0.16, -0.42, 0.18, 2.14, 0.785};
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    const Eigen::Isometry3d& l_eeWantedForIK = l_robotState->getGlobalLinkTransform("panda_link8");
    l_frankaKin.computeWristPosition(l_eeWantedForIK, l_jointValuesReady_rad.at(6), l_wrist);

    EXPECT_TRUE(l_wrist.isApprox(l_robotState->getGlobalLinkTransform("panda_link6").translation(), c_tolerance)) << "Compute Wrist with q7 !=0 fails";

    l_jointValuesReady_rad ={2.13528, 0.45, 0.16, -0.42, 0.18, 2.14, 0.0};
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    l_robotState->getGlobalLinkTransform("panda_link8"); 

    // ------------ compute6DOFIK Test ---------------- //
    
    
    std::vector<std::vector<double>> l_allIKSolutions;
    l_allIKSolutions.reserve(8);
    l_start = std::chrono::high_resolution_clock::now();
    l_frankaKin.compute6DOFIK(l_eeWantedForIK, 0.0, l_allIKSolutions);
    l_stop = std::chrono::high_resolution_clock::now();
    l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count();
    RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK took (singe call) " << l_micros << " us.");

    RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK returned " << l_allIKSolutions.size() << " solutions ");
    if(l_allIKSolutions.size() <= 0)
    {
        EXPECT_TRUE(false);
    }

    size_t l_solutionsInBounds = 0;

    //Numerical values to avoid MoveIt
    Eigen::Isometry3d l_eeWanted = Eigen::Isometry3d::Identity();
    l_eeWanted.translation() << -0.348869, 0.46065, 0.947152;
    l_eeWanted.linear() <<
    -0.0703608,  0.662334,  -0.745898,
    0.331076,   0.72087,    0.60888,
    0.940977,  -0.204108,  -0.270004;

    for (auto &&l_solution : l_allIKSolutions)
    {
        l_robotState->setJointGroupPositions(l_jointModelGroup, l_solution);
        //enforcing bounds will change MoveIt so it falls within bounds, thus the test cannot pass
        bool l_inBounds = l_robotState->satisfiesBounds();
        if(l_inBounds)
        {
            l_solutionsInBounds++;
        }
        
        RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver Solution is ");
        printJointVector(l_solution);

        l_robotState->getGlobalLinkTransform("panda_link8");//updates l_eeWantedForIK
        EXPECT_TRUE(l_eeWantedForIK.isApprox(l_eeWanted, c_toleranceLow)) << "IK returned wrong solutions";

        Eigen::Isometry3d l_computedFK;
        l_frankaKin.computeTipFK(l_solution, l_computedFK);
        EXPECT_TRUE(l_computedFK.isApprox(l_eeWanted, c_tolerance));
    }
    RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK returned " << l_solutionsInBounds << " solutions within bounds "); 

    l_jointValuesReady_rad ={2.13528, 0.45, 0.16, -0.42, 0.18, 2.14, 0.785};
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    l_robotState->getGlobalLinkTransform("panda_link8");

    l_eeWanted.translation() << -0.348869, 0.46065, 0.947152;
    l_eeWanted.linear() <<
    0.418382,  0.51826, -0.745898,
    0.743729, 0.275922,   0.60888,
    0.521368,   -0.80949, -0.27000;
    
    l_start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < c_repetitions; i++)
    {
        l_frankaKin.compute6DOFIK(l_eeWantedForIK, 0.785, l_allIKSolutions);
    }
    l_stop = std::chrono::high_resolution_clock::now();
    l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count();
    RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK took (over 1000 tries) " << l_micros/c_repetitions << " us.");

    RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK returned " << l_allIKSolutions.size() << " solutions ");
    if(l_allIKSolutions.size() <= 0)
    {
        EXPECT_TRUE(false);
    }
    l_solutionsInBounds = 0;
    for (auto &&l_solution : l_allIKSolutions)
    {
        l_robotState->setJointGroupPositions(l_jointModelGroup, l_solution);
        //enforcing bounds will change MoveIt so it falls within bounds, thus the test cannot pass
        //l_robotState->enforceBounds();
        bool l_inBounds = l_robotState->satisfiesBounds();
        std::vector<double> l_moveItJointValues_rad;
        l_robotState->copyJointGroupPositions(l_jointModelGroup, l_moveItJointValues_rad);

        RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver Solution is ");
        printJointVector(l_solution);
        //RCLCPP_INFO_STREAM(LOGGER, "MoveIt current joints are " );
        //printJointVector(l_moveItJointValues_rad);
        
        if(l_inBounds)
        {
            l_solutionsInBounds++;
        }
        l_robotState->getGlobalLinkTransform("panda_link8"); //updates MoveIt
        EXPECT_TRUE(l_eeWantedForIK.isApprox(l_eeWanted, c_toleranceLow)) << "IK returned wrong solutions";

        Eigen::Isometry3d l_computedFK;
        l_frankaKin.computeTipFK(l_solution, l_computedFK);
        EXPECT_TRUE(l_computedFK.isApprox(l_eeWanted, c_tolerance));
    }
    RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK returned " << l_solutionsInBounds << " solutions within bounds "); 


    // benchmark with GEOFIK //
    // std::array<double, 9> ROE = { -0.189536, 0.0420467, -0.980973,
    //     0.404078, -0.907217, -0.116958,
    //    -0.894873, -0.418557, 0.15496 };
    // std::array<double, 3> r = { 0.23189, -0.0815989, 0.607269 };
    // double q7 = 0.771925;
    // std::array<std::array<double, 7>, 8> qsols;
    // unsigned int nsols;
    // const double q1_sing = 3.14 / 2.0;
    
    // l_start = std::chrono::high_resolution_clock::now();
    // nsols = franka_ik_q7(r, ROE, q7, qsols, q1_sing);
    // l_stop = std::chrono::high_resolution_clock::now();
    // l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count();
    // RCLCPP_INFO_STREAM(LOGGER, "GeoFIK took (single call) " << l_micros << " us." << " nb of solutions " << nsols);

    // r = {0.61674948,0.32278029,0.56790512};
    // ROE = {0.6688331000000003,0.3170534383478098,0.6724130000000006,-0.6398146000000005,-0.2150772409286401,0.7378204999999999,0.3785492999999998,-0.9236984268883508,0.05900459999999996};
    // q7 = -0.37218362471412003 ;

    // l_start = std::chrono::high_resolution_clock::now();
    // for (size_t i = 0; i < c_repetitions; i++)
    // {
    //     nsols = franka_ik_q7(r, ROE, q7, qsols, q1_sing);
    // }
    // l_stop = std::chrono::high_resolution_clock::now();
    // l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count();
    // RCLCPP_INFO_STREAM(LOGGER, "GeoFIK took (over 1000 tries) " << l_micros/c_repetitions << " us.");
    // RCLCPP_INFO_STREAM(LOGGER, "GeoFIK returned (over 1000 tries) " << nsols << " solutions");

    // l_jointValuesReady_rad = {-1.74, 0.36, 1.79, -1.81, 0.06, -0.006, 0.77};
    // l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    // l_robotState->getGlobalLinkTransform("panda_link8");

    // // test Franka solver on same input
    // std::vector<std::vector<double>> l_allIKSolutions2;
    // l_allIKSolutions2.reserve(8);

    // l_start = std::chrono::high_resolution_clock::now();
    // l_frankaKin.compute6DOFIK(l_eeWantedForIK, l_jointValuesReady_rad.at(6), l_allIKSolutions2);
    // l_stop = std::chrono::high_resolution_clock::now();
    // l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count();
    // RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK took (singe call) " << l_micros << " us.");
    // RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK returned " << l_allIKSolutions.size() << " solutions ");

}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}