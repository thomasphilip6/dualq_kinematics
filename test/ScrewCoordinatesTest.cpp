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

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ScrewCoordinatesTest");

using ScrewCoordinates = dualq_kinematics::ScrewCoordinates<double>;

TEST(dualq_kinematics, ScrewCoordinatesConstructionTest)
{
    rclcpp::init(0, nullptr);
    auto const node = std::make_shared<rclcpp::Node>(
        "ScrewCoordinates_test",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    RCLCPP_INFO_STREAM(LOGGER, "Loading robot model from " << node->get_name() << "." << ROBOT_DESCRIPTION_PARAM);
    std::cout << "Retrieving Robot Model" << std::endl;
    rdf_loader::RDFLoader rdf_loader(node, ROBOT_DESCRIPTION_PARAM);
    moveit::core::RobotModelPtr l_robotModel = std::make_shared<moveit::core::RobotModel>(rdf_loader.getURDF(), rdf_loader.getSRDF());
    const moveit::core::RobotModel l_model = *l_robotModel.get();
    ASSERT_TRUE(bool(l_robotModel)) << "Failed to load robot model";

    //Test construction 
    std::cout << "Trying Screw Coordinates Construction" << std::endl;
    ScrewCoordinates l_screwCoord(l_model);

    //Test if vector sizes matches number of DOF
    auto l_positions = l_screwCoord.getPositions();
    auto l_screwAxes = l_screwCoord.getScrewAxes();
    auto l_jointNames = l_screwCoord.getJointsNames();
    EXPECT_EQ(l_positions.size(), 7) << "DOF number and position vector size differ";
    EXPECT_EQ(l_screwAxes.size(), 7) << "DOF number and screwAxes vector size differ";
    EXPECT_EQ(l_jointNames.size(), 7) << "DOF number and jointNames vector size differ";
    RCLCPP_INFO_STREAM(LOGGER, "Screw Coordinates Constructed, number of Joints :  " << l_positions.size());

    rclcpp::shutdown();
} 

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}