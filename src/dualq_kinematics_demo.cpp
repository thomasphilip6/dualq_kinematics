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
#include <chrono>

//link of the robot considered the tip of the kinematic chain
const std::string c_pandaTipLink = "panda_link8";
using DualQuaternion = dualq_kinematics::DualQuaternion<double>;
using ScrewCoordinates = dualq_kinematics::ScrewCoordinates<double>;
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

//Demo showing forward kinematics with dual quaternions and screw theory on a MoveIt configured robot

void dualQuaternionForwardKinematics(std::vector<DualQuaternion>& p_screwDQ, DualQuaternion& p_tip2BaseInit, std::vector<double>& p_jointValuesRad, Eigen::Isometry3d& p_tip2BaseComputed)
{
    DualQuaternion l_tip2BaseDQComputed = (p_screwDQ.at(0)*p_jointValuesRad.at(0)).dqExp();
    for (size_t i = 1; i < p_jointValuesRad.size(); i++)
    {
        l_tip2BaseDQComputed = l_tip2BaseDQComputed * (p_screwDQ.at(i)*p_jointValuesRad.at(i)).dqExp();
    }
    l_tip2BaseDQComputed = l_tip2BaseDQComputed * p_tip2BaseInit;
    p_tip2BaseComputed = l_tip2BaseDQComputed.getTransform();    
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "dualq_kinematics_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    const auto& LOGGER = node->get_logger();

    //Loading robot and constructing screw coordinates
    rdf_loader::RDFLoader rdf_loader(node, ROBOT_DESCRIPTION_PARAM);
    moveit::core::RobotModelPtr l_robotModel = std::make_shared<moveit::core::RobotModel>(rdf_loader.getURDF(), rdf_loader.getSRDF());
    
    RCLCPP_INFO_STREAM(LOGGER, "Starting Screw Coordinates construction");
    auto l_start = std::chrono::high_resolution_clock::now();
    ScrewCoordinates l_screwCoord(*l_robotModel, c_pandaTipLink);
    auto l_stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> l_ms = l_stop - l_start;
    RCLCPP_INFO_STREAM(LOGGER, "Screw Coordinates constructed in " << l_ms.count() << " ms");

    //Constructing Dual Quaternions from Screw Coordinates
    RCLCPP_INFO_STREAM(LOGGER, "Computing " << l_screwCoord.getJointsNames().size()+1 << " dual quaternions representing the screw coordinates");
    l_start = std::chrono::high_resolution_clock::now();
    std::vector<DualQuaternion> l_screwDQ;
    for (size_t i = 0; i < l_screwDQ.size(); i++)
    {
        l_screwDQ.push_back(DualQuaternion( l_screwCoord.getScrewAxes().at(i), l_screwCoord.getPositions().at(i) ));
    }
    DualQuaternion l_tip2BaseInit(l_screwCoord.getTip2BaseInit());
    l_stop = std::chrono::high_resolution_clock::now();
    l_ms = l_stop - l_start;
    RCLCPP_INFO_STREAM(LOGGER, "Dual Quaternions constructions took " << l_ms.count() << " ms");

    //Constructing Robot State
    moveit::core::RobotStatePtr l_robotState(new moveit::core::RobotState(l_robotModel));
    l_robotState->setToDefaultValues();
    const moveit::core::JointModelGroup* l_jointModelGroup = l_robotModel->getJointModelGroup("panda_arm");
    const std::vector<std::string>& l_jointNames = l_jointModelGroup->getVariableNames();

    std::vector<double> l_jointValues_rad;

    l_robotState->enforceBounds();
    l_robotState->setToRandomPositions(l_jointModelGroup);
    l_robotState->copyJointGroupPositions(l_jointModelGroup, l_jointValues_rad);
    for (std::size_t i = 0; i < l_jointNames.size(); ++i)
    {
        RCLCPP_INFO(LOGGER, "Joint %s: %f", l_jointNames[i].c_str(), l_jointValues_rad[i]);
    }
    const Eigen::Isometry3d& l_eeStateMoveIt = l_robotState->getGlobalLinkTransform("panda_link8");
    RCLCPP_INFO_STREAM(LOGGER, "Translation from MoveIt: \n" << l_eeStateMoveIt.translation() << "\n");
    RCLCPP_INFO_STREAM(LOGGER, "Rotation from MoveIt: \n" << l_eeStateMoveIt.rotation() << "\n");
    Eigen::Isometry3d l_eeStateDQ;
    l_start = std::chrono::high_resolution_clock::now();
    dualQuaternionForwardKinematics(l_screwDQ, l_tip2BaseInit, l_jointValues_rad, l_eeStateDQ);
    l_stop = std::chrono::high_resolution_clock::now();
    l_ms = l_stop - l_start;
    RCLCPP_INFO_STREAM(LOGGER, "Dual Quaternions FK took " << l_ms.count() << " ms");
    RCLCPP_INFO_STREAM(LOGGER, "Translation from DQ FK: \n" << l_eeStateDQ.translation() << "\n");
    RCLCPP_INFO_STREAM(LOGGER, "Rotation from DQ FK: \n" << l_eeStateDQ.rotation() << "\n");

    rclcpp::shutdown();
    return 0; 

}