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

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ScrewCoordinatesTest");

using ScrewCoordinates = dualq_kinematics::ScrewCoordinates<double>;

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
    const moveit::core::RobotModel l_model = *l_robotModel.get();
    RCLCPP_INFO_STREAM(LOGGER, "Robot Model ready to by passed to ScrewCoordinates ");
    ASSERT_TRUE(bool(l_robotModel)) << "Failed to load robot model";
    //printJointsInfo(l_robotModel);

    //Using RobotModelLoader for debug
    //robot_model_loader::RobotModelLoader loader(node);

    //Test construction 
    ScrewCoordinates l_screwCoord(l_model);

    //Test if vector sizes matches number of DOF
    auto l_positions = l_screwCoord.getPositions();
    auto l_screwAxes = l_screwCoord.getScrewAxes();
    auto l_jointNames = l_screwCoord.getJointsNames();
    EXPECT_EQ(l_positions.size(), 7) << "DOF number and position vector size differ";
    EXPECT_EQ(l_screwAxes.size(), 7) << "DOF number and screwAxes vector size differ";
    EXPECT_EQ(l_jointNames.size(), 7) << "DOF number and jointNames vector size differ";
    RCLCPP_INFO_STREAM(LOGGER, "Screw Coordinates Constructed, number of Joints :  " << l_positions.size());
    printJntNames(l_jointNames);
} 

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}