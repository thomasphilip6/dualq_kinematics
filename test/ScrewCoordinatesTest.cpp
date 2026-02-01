#include <gtest/gtest.h>
#include "dualq_kinematics/DualQuaternion.h"
#include "dualq_kinematics/ScrewCoordinates.h"

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

constexpr uint8_t c_dofNumberPanda = 7;
constexpr double c_tolerance = 1e-3;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ScrewCoordinatesTest");

using ScrewCoordinates = dualq_kinematics::ScrewCoordinates<double>;
using Vector3 = Eigen::Matrix<double, 3, 1>; 

void printJointsInfo(moveit::core::RobotModelPtr p_robotModelPtr)
{
    const urdf::ModelInterfaceSharedPtr l_urdf = p_robotModelPtr->getURDF();
    RCLCPP_INFO_STREAM(LOGGER, "RobotModel has  :  " << p_robotModelPtr->getActiveJointModels().size() << " active joints");
    RCLCPP_INFO_STREAM(LOGGER, "URDF has  :  " << l_urdf->joints_.size() << " joints");
    RCLCPP_INFO_STREAM(LOGGER, "URDF has  :  " << p_robotModelPtr->getEndEffectors().size() << "end effectors");
}

void printJntNames(std::vector<std::string> p_jntNames)
{
    for (auto &&l_jntName : p_jntNames)
    {
        RCLCPP_INFO_STREAM(LOGGER, "Valid Joint :  " << l_jntName);
    }
      
}

void printScrewInfo(ScrewCoordinates& p_screwCoord)
{
    for (auto &&l_axis : p_screwCoord.getScrewAxes())
    {
        RCLCPP_INFO_STREAM(LOGGER, "Screw Axis :  [" << l_axis.x() << ", " << l_axis.y() << ", " << l_axis.z() << " ]");
    }

    for (auto &&l_pos : p_screwCoord.getPositions())
    {
        RCLCPP_INFO_STREAM(LOGGER, "Position :  [" << l_pos.x() << ", " << l_pos.y() << ", " << l_pos.z() << " ]");
    }
    RCLCPP_INFO_STREAM(LOGGER, "Tip2BaseInit :  [" << p_screwCoord.getTip2BaseInit().matrix() << " ]");
}

TEST(dualq_kinematics, ScrewCoordinatesConstructionTest)
{
    auto const node = std::make_shared<rclcpp::Node>(
        "ScrewCoordinates_test",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    RCLCPP_INFO_STREAM(LOGGER, "Loading robot model from " << node->get_name() << "." << ROBOT_DESCRIPTION_PARAM);
    std::cout << "Retrieving Robot Model" << std::endl;
    rdf_loader::RDFLoader rdf_loader(node, ROBOT_DESCRIPTION_PARAM);
    moveit::core::RobotModelPtr l_robotModel = std::make_shared<moveit::core::RobotModel>(rdf_loader.getURDF(), rdf_loader.getSRDF());
    //const moveit::core::RobotModel l_model = *l_robotModel.get();
    RCLCPP_INFO_STREAM(LOGGER, "Robot Model ready to by passed to ScrewCoordinates ");
    ASSERT_TRUE(bool(l_robotModel)) << "Failed to load robot model";
    //printJointsInfo(l_robotModel);

    //Using RobotModelLoader for debug
    //robot_model_loader::RobotModelLoader loader(node);

    //Test construction 
    ScrewCoordinates l_screwCoord(*l_robotModel, "panda_link8");

    //Test if vector sizes matches number of DOF
    auto l_positions = l_screwCoord.getPositions();
    auto l_screwAxes = l_screwCoord.getScrewAxes();
    auto l_jointNames = l_screwCoord.getJointsNames();
    EXPECT_EQ(l_positions.size(), c_dofNumberPanda) << "DOF number and position vector size differ";
    EXPECT_EQ(l_screwAxes.size(), c_dofNumberPanda) << "DOF number and screwAxes vector size differ";
    EXPECT_EQ(l_jointNames.size(), c_dofNumberPanda) << "DOF number and jointNames vector size differ";
    RCLCPP_INFO_STREAM(LOGGER, "Screw Coordinates Constructed, number of Joints :  " << l_positions.size());
    printJntNames(l_jointNames);
    printScrewInfo(l_screwCoord);

    //The following are the references 
    const std::array<Vector3, c_dofNumberPanda> l_screwAxesRef = {
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

    const std::array<Vector3, c_dofNumberPanda> l_positionsRef = {
        Vector3(0.0, 0.0, l_d1),
        Vector3(0.0, 0.0, l_d1),
        Vector3(0.0, 0.0, l_d1+l_d3),
        Vector3(l_a5, 0.0, l_d1+l_d3),
        Vector3(0.0, 0.0, l_d1+l_d3+l_d5),
        Vector3(0.0, 0.0, l_d1+l_d3+l_d5),
        Vector3(l_a7, 0.0, l_d1+l_d3+l_d5),
    };

    const std::array<std::string, c_dofNumberPanda> l_jointNamesRef = {
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    };

    Eigen::Isometry3d l_tip2BaseAtRestRef = Eigen::Translation3d(l_a7, 0.0, l_d1+l_d3+l_d5-l_df) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

    for (size_t i = 0; i < c_dofNumberPanda; i++)
    {
        EXPECT_TRUE(l_screwAxesRef.at(i).isApprox(l_screwAxes.at(i), c_tolerance)) << "Computed Screw axis must match reference";
        EXPECT_TRUE(l_positionsRef.at(i).isApprox(l_positions.at(i), c_tolerance)) << "Computed Position on screw axis must match reference";
        
    }
    EXPECT_TRUE(std::equal(l_jointNames.begin(), l_jointNames.end(), l_jointNamesRef.begin())) << "Active joint names must match reference";
    EXPECT_TRUE(l_tip2BaseAtRestRef.isApprox(l_screwCoord.getTip2BaseInit(), c_tolerance)) << "Computed TF tip2Base initial must match reference";

} 

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}