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
// #include "dualq_kinematics/geofik.h"

const std::string c_pandaTipLink = "panda_link8";
constexpr u_int8_t c_repetitionsLow = 100;
constexpr u_int16_t c_repetitions = 2000;
constexpr double c_tolerance = 1e-5;
constexpr double c_toleranceLow = 1e-3;
constexpr double c_redundancyTolerance = 1e-2;

using ScrewCoordinates = dualq_kinematics::ScrewCoordinates<double>;
using FrankaKinSolver = dualq_kinematics::FrankaKinSolver<double>;
using FirstPadenKahan = dualq_kinematics::FirstPadenKahanProblem<double>;
using Vector3 = Eigen::Matrix<double, 3, 1>; 

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("FrankaKinSolverTest");

void printEigenIsometry(const Eigen::Isometry3d& p_transformationMatrix)
{
    RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << p_transformationMatrix.translation() << "\n");
    RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << p_transformationMatrix.rotation() << "\n");
}

bool containsNan(const std::vector<double>& p_vector)
{
    for (auto &&l_joint : p_vector)
    {
        if(isnan(l_joint))
        {
            return true;
        }
    }  
    return false;
}

void printJointVector(const std::vector<double>& vector)
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

void checkIKResults(
    const std::vector<std::vector<double>>& p_results, 
    const Eigen::Isometry3d& p_eeWanted, 
    const moveit::core::JointModelGroup* p_jointModelGroup, 
    moveit::core::RobotStatePtr p_robotState,
    const FrankaKinSolver& p_frankaKin,
    const Eigen::Isometry3d& p_link8State,
    const std::string& p_solverName = "FrankaKinSolver"
)
{
    //RCLCPP_INFO_STREAM(LOGGER, p_solverNmae + " IK returned " << p_results.size() << " solutions ");
    if(p_results.size() == 0)
    {
        EXPECT_TRUE(false);
    }

    size_t l_solutionsInBounds = 0;
    for(auto &&l_solution : p_results)
    {
        p_robotState->setJointGroupPositions(p_jointModelGroup, l_solution);
        //enforcing bounds will change MoveIt so it falls within bounds, thus the test cannot pass
        bool l_inBounds = p_robotState->satisfiesBounds();
        if(l_inBounds)
        {
            l_solutionsInBounds++;
        }
        //printJointVector(l_solution);

        p_robotState->getGlobalLinkTransform("panda_link8");//updates l_eeCurrentState
        bool l_moveItCheck = p_link8State.isApprox(p_eeWanted, c_tolerance);
        EXPECT_TRUE(l_moveItCheck) << p_solverName + " IK returned wrong solutions (MoveIt Check)";

        Eigen::Isometry3d l_computedFK;
        p_frankaKin.computeTipFK(l_solution, l_computedFK);
        bool l_customFKCheck = l_computedFK.isApprox(p_eeWanted, c_tolerance);
        EXPECT_TRUE(l_customFKCheck) << p_solverName + " IK returned wrong solutions (Custom FK)";

        //Usefull to put a breakpoint here for precision investigation
        if(!l_customFKCheck || !l_moveItCheck)
        {
            RCLCPP_INFO_STREAM(LOGGER, " Precision drop around a singularity ? ");
            l_moveItCheck = p_link8State.isApprox(p_eeWanted, c_toleranceLow);
            l_customFKCheck = l_computedFK.isApprox(p_eeWanted, c_toleranceLow);
            if(!l_customFKCheck || !l_moveItCheck)
            {
                RCLCPP_INFO_STREAM(LOGGER, p_solverName + " fails at low precision ");
            }
        }
    }
    RCLCPP_INFO_STREAM(LOGGER, p_solverName + " IK returned " << l_solutionsInBounds << " solutions within bounds "); 
    RCLCPP_INFO_STREAM(LOGGER, " ---- ");
}

void computeMoveItSwivel(moveit::core::RobotStatePtr p_robotState, double& p_moveItSwivel)
{
    const Eigen::Isometry3d& l_joint4ToBase = p_robotState->getGlobalLinkTransform("panda_link4");
    const Eigen::Isometry3d& l_joint6ToBase = p_robotState->getGlobalLinkTransform("panda_link6");
    //l_d1 is substracted as elbow and wrist are expressend in shoulder reference frame to get real SEW angle
    const double l_d1 = 0.333;
    Eigen::Vector3d l_s4(l_joint4ToBase(0,2), l_joint4ToBase(1,2), l_joint4ToBase(2,2));
    Eigen::Vector3d l_r4(l_joint4ToBase(0,3), l_joint4ToBase(1,3), l_joint4ToBase(2,3) - l_d1);
    Eigen::Vector3d l_r6(l_joint6ToBase(0,3), l_joint6ToBase(1,3), l_joint6ToBase(2,3)- l_d1);

    Eigen::Vector3d l_n1 = l_r6.cross( Eigen::Vector3d(0,0,1));
    Eigen::Vector3d l_n2 = l_r6.cross(l_r4);
    const double l_sign = l_n2.dot(l_s4);
    if (l_sign < 0)
    {
        l_n2 = l_n2 * (-1);
    }
    l_n1.normalize();
    l_n2.normalize();
    l_r6.normalize(); //so it becomes a screw axis
    p_moveItSwivel = std::atan2(
            l_r6.dot(l_n1.cross(l_n2)),
            l_n1.dot(l_n2)
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

    ScrewCoordinates l_screwCoord(l_robotModel, c_pandaTipLink);
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
    l_frankaKin.computeTipFK(l_jointValuesReady_rad, l_computedResult);
    auto l_stop = std::chrono::high_resolution_clock::now();
    double l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count();
    RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver FK took (single, cold call) " << l_micros << " us.");
    
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
    const Eigen::Isometry3d& l_eeCurrentState = l_robotState->getGlobalLinkTransform("panda_link8");
    l_frankaKin.computeWristPosition(l_eeCurrentState, l_jointValuesReady_rad.at(6), l_wrist);

    EXPECT_TRUE(l_wrist.isApprox(l_robotState->getGlobalLinkTransform("panda_link6").translation(), c_tolerance)) << "Compute Wrist with q7 !=0 fails";

    l_jointValuesReady_rad ={2.13528, 0.0, 0.16, -0.42, 0.18, 2.14, 0.0};
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    l_robotState->getGlobalLinkTransform("panda_link8"); 

    // ------------ compute6DOFIK Test ---------------- //
    
    // ----------- 1 - Shoulder Singularity Test ---------------- //
    
    //l_frankaKin.setEmergencyQ1(l_jointValuesReady_rad.at(0));
    std::vector<std::vector<double>> l_allIKSolutions;
    l_allIKSolutions.reserve(8);
    l_start = std::chrono::high_resolution_clock::now();
    l_frankaKin.compute6DOFIK(l_eeCurrentState, l_jointValuesReady_rad.at(6), l_allIKSolutions);
    for(auto &&l_solution : l_allIKSolutions)
    {
        printJointVector(l_solution);
    }
    
    l_stop = std::chrono::high_resolution_clock::now();
    l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count();
    RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK took (single call) " << l_micros << " us.");

    Eigen::Isometry3d l_eeWanted = l_eeCurrentState;

    checkIKResults(l_allIKSolutions, l_eeWanted, l_jointModelGroup, l_robotState, l_frankaKin, l_eeCurrentState);
    
    l_jointValuesReady_rad = {0.7, 0.0, 2.1, -1.7, 2.1, 2.5, 1.0};
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    l_robotState->getGlobalLinkTransform("panda_link8");

    l_eeWanted = l_eeCurrentState;
    
    l_frankaKin.setEmergencyQ1(l_jointValuesReady_rad.at(0));
    l_start = std::chrono::high_resolution_clock::now();
    l_frankaKin.compute6DOFIK(l_eeCurrentState, l_jointValuesReady_rad.at(6), l_allIKSolutions);
    for(auto &&l_solution : l_allIKSolutions)
    {
        printJointVector(l_solution);
    }

    l_stop = std::chrono::high_resolution_clock::now();
    l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count();

    RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK took (single call)" << l_micros << " us.");
    checkIKResults(l_allIKSolutions, l_eeWanted, l_jointModelGroup, l_robotState, l_frankaKin, l_eeCurrentState);

    // ----------- 2 - Out of workspace ---------------- //

    Eigen::Isometry3d l_outOfWorkspaceEE = Eigen::Isometry3d::Identity();
    l_outOfWorkspaceEE.translation() << -0.348869, 3.46065, 0.947152;
    l_outOfWorkspaceEE.linear() <<
    -0.0703608,  0.662334,  -0.745898,
    0.331076,   0.72087,    0.60888,
    0.940977,  -0.204108,  -0.270004;

    l_frankaKin.compute6DOFIK(l_outOfWorkspaceEE, 0.0, l_allIKSolutions);
    EXPECT_EQ(0, l_allIKSolutions.size()) << "Out of bounds EE Pose must not return any solution";
    l_frankaKin.compute6DOFIK(l_outOfWorkspaceEE, 0.7, l_allIKSolutions);
    EXPECT_EQ(0, l_allIKSolutions.size()) << "Out of bounds EE Pose must not return any solution";
    l_frankaKin.compute6DOFIK(l_outOfWorkspaceEE, 0.2, l_allIKSolutions);
    EXPECT_EQ(0, l_allIKSolutions.size()) << "Out of bounds EE Pose must not return any solution";

    // ----------- 3 - Small regular tests and ---------------- //


    l_jointValuesReady_rad = {0.7, 0.43, 2.1, -1.7, 2.1, 2.5, 1.0};
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    l_robotState->getGlobalLinkTransform("panda_link8");

    l_start = std::chrono::high_resolution_clock::now();
    l_frankaKin.compute6DOFIK(l_eeCurrentState, l_jointValuesReady_rad.at(6), l_allIKSolutions);
    l_stop = std::chrono::high_resolution_clock::now();
    l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count();
    for(auto &&l_solution : l_allIKSolutions)
    {
        printJointVector(l_solution);
    }

    RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK took (single call) " << l_micros << " us.");

    l_eeWanted = l_eeCurrentState;
    checkIKResults(l_allIKSolutions, l_eeWanted, l_jointModelGroup, l_robotState, l_frankaKin, l_eeCurrentState);
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    l_robotState->getGlobalLinkTransform("panda_link8");

    //Interesting Values to debug
    //l_jointValuesReady_rad = {-0.7, 1.45, 0.7, -1.7, 1.7, 3.1, 0.42};
    //l_jointValuesReady_rad = {0,0,0,0,0,0,0};
    //l_jointValuesReady_rad = { -2.8792, -1.19837, 1.21447, -0.529867, 2.26389, 2.71011, 1.65679};
    //l_jointValuesReady_rad = { 0.189909, 1.02324, -1.20815, -0.404138, 1.5708, 2.86532, 1.65679};
    //l_jointValuesReady_rad = {2.55169, -1.18741, 1.30594, 0.000285864, -0.993168, 3.33982, -0.0756937};
    l_jointValuesReady_rad = {-0.724018, -0.844134, -1.21779, -0.708274, -2.80283, 0.96974, 2.35744};
    l_frankaKin.setEmergencyQ5(l_jointValuesReady_rad.at(4));
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    l_robotState->getGlobalLinkTransform("panda_link8");

    l_start = std::chrono::high_resolution_clock::now();
    l_frankaKin.compute6DOFIK(l_eeCurrentState, l_jointValuesReady_rad.at(6), l_allIKSolutions);
    l_stop = std::chrono::high_resolution_clock::now();
    l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count();
    for(auto &&l_solution : l_allIKSolutions)
    {
        printJointVector(l_solution);
    }

    RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK took (single call) " << l_micros << " us.");
    
    l_eeWanted = l_eeCurrentState;
    checkIKResults(l_allIKSolutions, l_eeWanted, l_jointModelGroup, l_robotState, l_frankaKin, l_eeCurrentState);

    // --------------------------------- 4 - computeSwivelAngle and compute7DOFIK tests  -------------------------------------- //

    //l_jointValuesReady_rad = {-2.3404, -1.02822, 2.17959, 0.00957927, 0.145148, 1.80714, 2.69931};
    l_jointValuesReady_rad = {0.7, 0.43, 2.1, -1.7, 2.1, 2.5, 1.0};
    l_frankaKin.setEmergencyQ5(l_jointValuesReady_rad.at(4));
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    l_robotState->getGlobalLinkTransform("panda_link8");

    std::optional<double> l_swivel;
    l_frankaKin.computeSwivelAngle(l_eeCurrentState, l_jointValuesReady_rad.at(4), l_jointValuesReady_rad.at(5), l_jointValuesReady_rad.at(6), l_swivel);   
    double l_moveItSwivel;
    computeMoveItSwivel(l_robotState,l_moveItSwivel);
    
    EXPECT_TRUE(FirstPadenKahan::compareFloatNum(l_swivel.value(), l_moveItSwivel, c_tolerance)) << "Swivel Angle computation doesn't work (MoveIt comparison)";

    l_frankaKin.compute7DOFIK(l_eeCurrentState, l_swivel.value(), l_allIKSolutions);

    //Check IK
    l_eeWanted = l_eeCurrentState;
    checkIKResults(l_allIKSolutions, l_eeWanted, l_jointModelGroup, l_robotState, l_frankaKin, l_eeCurrentState);
    
    for (auto &&l_solution : l_allIKSolutions)
    {
        l_frankaKin.computeSwivelAngle(l_eeCurrentState, l_solution.at(4), l_solution.at(5), l_solution.at(6), l_swivel);     
        EXPECT_TRUE(FirstPadenKahan::compareFloatNum(l_swivel.value(), l_moveItSwivel, c_redundancyTolerance)) << "Redundancy Solution did not return satisfying SEW angle";

    }
    
    // ----------- 5 - Existence and Number of solutions tests (GEOFIK used for comparison) ---------------- //

    // std::array<std::array<double, 7>, 8> l_geoFIKSolutions;
    // std::array<double, 9> l_ROE;
    // std::array<double, 3> l_r;
    for(size_t i=0; i <= c_repetitionsLow; i++)
    {
        //Create a Target
        //setToRandomPositions returns a joint configuration within bounds, so if FrankaKin doesn't find a solution, there is a problem
        l_robotState->setToRandomPositions(l_jointModelGroup);
        l_robotState->getGlobalLinkTransform("panda_link8");
        l_robotState->copyJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);

        //Compute with FrankaKinSolver
        l_frankaKin.setEmergencyQ1(l_jointValuesReady_rad.at(0));
        l_frankaKin.compute6DOFIK(l_eeCurrentState, l_jointValuesReady_rad.at(6), l_allIKSolutions);
        
        l_eeWanted = l_eeCurrentState;
        //RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK results " );
        checkIKResults(l_allIKSolutions, l_eeWanted, l_jointModelGroup, l_robotState, l_frankaKin, l_eeCurrentState);
        // for(auto &&l_solution : l_allIKSolutions)
        // {
        //     printJointVector(l_solution);
        // }
        //RCLCPP_INFO_STREAM(LOGGER, " ----------- " );

        //RCLCPP_INFO_STREAM(LOGGER, "GeoFIK IK results " );
        
        // ------------------------------- Compare with GeoFIK --------------------------------//

        // const std::array<double, 7> l_q = {l_jointValuesReady_rad.at(0), l_jointValuesReady_rad.at(1), l_jointValuesReady_rad.at(2), l_jointValuesReady_rad.at(3), l_jointValuesReady_rad.at(4), l_jointValuesReady_rad.at(5), l_jointValuesReady_rad.at(6)};
        // Eigen::Matrix4d l_eigenFIKPose = franka_fk(l_q, 'E');

        // //Build ROE in row major order for GEOFIK
        // l_ROE = {
        //     l_eigenFIKPose(0,0), l_eigenFIKPose(0,1), l_eigenFIKPose(0,2),
        //     l_eigenFIKPose(1,0), l_eigenFIKPose(1,1), l_eigenFIKPose(1,2),
        //     l_eigenFIKPose(2,0), l_eigenFIKPose(2,1), l_eigenFIKPose(2,2)
        // };       

        // l_r = { l_eigenFIKPose(0,3), l_eigenFIKPose(1,3), l_eigenFIKPose(2, 3)};
        // unsigned int l_nsolsGeoFIK = franka_ik_q7(l_r, l_ROE, l_q.at(6), l_geoFIKSolutions, l_q.at(0));
        // size_t l_numberSolutions = 0;
        // std::vector<std::vector<double>> l_geoFIKSolutionInBounds;

        // for (unsigned int j = 0; j < l_nsolsGeoFIK; j++)
        // {
        //     for (size_t i = 0; i < l_jointValuesReady_rad.size(); i++)
        //     {
        //         l_jointValuesReady_rad.at(i) = l_geoFIKSolutions.at(j).at(i);
        //     }
        //     //printJointVector(l_jointValuesReady_rad);
        //     l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
        //     bool l_inBounds = l_robotState->satisfiesBounds();
        //     if(l_inBounds && !containsNan(l_jointValuesReady_rad))
        //     {
        //         l_numberSolutions++;

        //         //Create a vector of vector for GeoFIK result to be checked
        //         //Check to see if GeoFIK drops precision at some point
                
        //         l_geoFIKSolutionInBounds.push_back(l_jointValuesReady_rad);
        //     }
                    
        // }

        // checkIKResults(l_geoFIKSolutionInBounds, l_eeWanted, l_jointModelGroup, l_robotState, l_frankaKin, l_eeCurrentState, "GeoFIK");
        // //RCLCPP_INFO_STREAM(LOGGER, " ----------- " );
        // //RCLCPP_INFO_STREAM(LOGGER, " ----------- Next -----------  " );

        // //EXPECT_EQ(l_numberSolutions, l_allIKSolutions.size()) << "GeoFIK and FrankaKinSolver do not return the same number of solutions";
        // if(l_numberSolutions != l_allIKSolutions.size())
        // {
        //     RCLCPP_INFO_STREAM(LOGGER, "FrankaKinSolver IK results " );
        //     for(auto &&l_solution : l_allIKSolutions)
        //     {
        //         printJointVector(l_solution);
        //     }
        //     RCLCPP_INFO_STREAM(LOGGER, " ----------- " );

        //     RCLCPP_INFO_STREAM(LOGGER, "GeoFIK IK results " );

        //     for(auto &&l_solution : l_geoFIKSolutionInBounds)
        //     {
        //         printJointVector(l_solution);
        //     }
        //     RCLCPP_INFO_STREAM(LOGGER, " ----------- " );
        //     RCLCPP_INFO_STREAM(LOGGER, " ----------- Next -----------  " );
        // }

    }

    // ----------- 6 - Performance tests (GEOFIK used for comparison) ---------------- //

    //Mean time over 2000 random valid poses
    l_micros = 0.0;

    for(size_t i=0; i <= c_repetitions; i++)
    {
        //Create a Target
        //setToRandomPositions returns a joint configuration within bounds, so if FrankaKin doesn't find a solution, there is a problem
        l_robotState->setToRandomPositions(l_jointModelGroup);
        l_robotState->getGlobalLinkTransform("panda_link8");
        l_robotState->copyJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);

        //Compute with FrankaKinSolver
        l_frankaKin.setEmergencyQ1(l_jointValuesReady_rad.at(0));

        l_start = std::chrono::high_resolution_clock::now();
        l_frankaKin.compute6DOFIK(l_eeCurrentState, l_jointValuesReady_rad.at(6), l_allIKSolutions);
        l_stop = std::chrono::high_resolution_clock::now();
        l_micros = l_micros + std::chrono::duration<double, std::micro>(l_stop - l_start).count();

        if(l_allIKSolutions.size() == 0)
        {
            EXPECT_TRUE(false) << "FrankaKin fails to find a solution in performance test";
            RCLCPP_INFO_STREAM(LOGGER, " FrankaKinSolver fails at this joint vector target :  ");
            printJointVector(l_jointValuesReady_rad);
        }
        
    }

    RCLCPP_INFO_STREAM(LOGGER, " FrankaKinSolver mean time over 2000 random poses :  " << (l_micros/c_repetitions) << " us." );
    //l_micros = 0.0;

    // for(size_t i=0; i <= c_repetitions; i++)
    // {
    //     //Create a Target
    //     //setToRandomPositions returns a joint configuration within bounds, so if GeoFIK doesn't find a solution, there is a problem
    //     l_robotState->setToRandomPositions(l_jointModelGroup);
    //     l_robotState->getGlobalLinkTransform("panda_link8");
    //     l_robotState->copyJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);

    //     const std::array<double, 7> l_q = {l_jointValuesReady_rad.at(0), l_jointValuesReady_rad.at(1), l_jointValuesReady_rad.at(2), l_jointValuesReady_rad.at(3), l_jointValuesReady_rad.at(4), l_jointValuesReady_rad.at(5), l_jointValuesReady_rad.at(6)};
    //     Eigen::Matrix4d l_eigenFIKPose = franka_fk(l_q, 'E');

    //     //Build ROE in row major order for GEOFIK
    //     l_ROE = {
    //         l_eigenFIKPose(0,0), l_eigenFIKPose(0,1), l_eigenFIKPose(0,2),
    //         l_eigenFIKPose(1,0), l_eigenFIKPose(1,1), l_eigenFIKPose(1,2),
    //         l_eigenFIKPose(2,0), l_eigenFIKPose(2,1), l_eigenFIKPose(2,2)
    //     };       

    //     l_r = { l_eigenFIKPose(0,3), l_eigenFIKPose(1,3), l_eigenFIKPose(2, 3)};
    //     l_start = std::chrono::high_resolution_clock::now();
    //     unsigned int l_nsolsGeoFIK = franka_ik_q7(l_r, l_ROE, l_q.at(6), l_geoFIKSolutions, l_q.at(0));
    //     l_stop = std::chrono::high_resolution_clock::now();
    //     l_micros = l_micros + std::chrono::duration<double, std::micro>(l_stop - l_start).count();

    //     if(l_nsolsGeoFIK == 0)
    //     {
    //         EXPECT_TRUE(false) << "GeoFIK fails to find a solution in performance test";
    //     }     
    // }

    // RCLCPP_INFO_STREAM(LOGGER, " GeoFIK mean time over 2000 random poses :  " << (l_micros/c_repetitions) << " us." );

}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}