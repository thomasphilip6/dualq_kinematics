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

const std::string c_pandaTipLink = "panda_link8";
//const size_t c_repetitions = 3;
constexpr double c_tolerance = 1e-6;

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

    FrankaKinSolver l_frankaKin(l_screwCoord);

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
    l_frankaKin.computeTipFK(l_jointValuesReady_rad, l_computedResult);
    EXPECT_EQ(l_robotState->getGlobalLinkTransform("panda_link8").isApprox(l_computedResult, c_tolerance), 1) << "FK fails";

    // ------------ computeWristPosition Test ---------------- //
    
    l_jointValuesReady_rad ={2.13528, 0.45, 0.16, -0.42, 0.18, 2.14, 0.0};;
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    const Eigen::Isometry3d& l_eeWanted = l_robotState->getGlobalLinkTransform("panda_link8"); 

    Vector3 l_wrist;
    l_frankaKin.computeWristPosition(l_eeWanted, l_jointValuesReady_rad.at(6), l_wrist);
    EXPECT_TRUE(l_wrist.isApprox(l_robotState->getGlobalLinkTransform("panda_link6").translation(), c_tolerance)) << "Compute Wrist fails";

    l_jointValuesReady_rad ={2.13528, 0.45, 0.16, -0.42, 0.18, 2.14, 0.785};;
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    l_frankaKin.computeWristPosition(l_robotState->getGlobalLinkTransform("panda_link8"), l_jointValuesReady_rad.at(6), l_wrist);
    EXPECT_TRUE(l_wrist.isApprox(l_robotState->getGlobalLinkTransform("panda_link6").translation(), c_tolerance)) << "Compute Wrist with q7 !=0 fails";

}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}