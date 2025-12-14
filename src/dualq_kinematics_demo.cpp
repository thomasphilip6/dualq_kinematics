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

//Tolerance of match between MoveIt Forward Kinematics and Dual Quaternion FK
constexpr double c_tolerance = 1e-6;
using DualQuaternion = dualq_kinematics::DualQuaternion<double>;
using ScrewCoordinates = dualq_kinematics::ScrewCoordinates<double>;
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("dualq_kinematics_demo");

//Demo showing forward kinematics with dual quaternions and screw theory on a MoveIt configured robot

/**
 * @brief Prints Screw Coordinates information: axes, positions and tip2BaseInit
 * @param p_screwCoord ScrewCoordinates instance to be printed
 */
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

/**
 * @brief Prints Eigen Isometry information as translation and rotation
 * @param p_screwCoord Eigen::Isometry instance to be printed
 */
void printEigenIsometry(const Eigen::Isometry3d& p_transformationMatrix)
{
    RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << p_transformationMatrix.translation() << "\n");
    RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << p_transformationMatrix.rotation() << "\n");
}

/**
 * @brief Computes the forward kinematics using dual quaternions
 * @param p_screwDQ ScrewCoordinates of the robot
 * @param p_tip2BaseInit Initial tip2Base transform (robot at rest)
 * @param p_jointValues_rad Joint Values to be applied to the joints
 * @param p_tip2BaseComputed The result of the forward kinematics computation as an Eigen Isometry
 */
void dualQuaternionForwardKinematics(const std::vector<DualQuaternion>& p_screwDQ, const DualQuaternion& p_tip2BaseInit, std::vector<double>& p_jointValues_rad, Eigen::Isometry3d& p_tip2BaseComputed)
{
    DualQuaternion l_tip2BaseDQComputed = (p_screwDQ.at(0) * (p_jointValues_rad.at(0) * 0.5)).dqExp();
    for (size_t i = 1; i < p_jointValues_rad.size(); i++)
    {
        l_tip2BaseDQComputed = l_tip2BaseDQComputed * (p_screwDQ.at(i)* (p_jointValues_rad.at(i) * 0.5)).dqExp();
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

    //Loading robot and constructing screw coordinates
    rdf_loader::RDFLoader rdf_loader(node, ROBOT_DESCRIPTION_PARAM);
    moveit::core::RobotModelPtr l_robotModel = std::make_shared<moveit::core::RobotModel>(rdf_loader.getURDF(), rdf_loader.getSRDF());
    
    RCLCPP_INFO_STREAM(LOGGER, "Starting Screw Coordinates construction");
    auto l_start = std::chrono::high_resolution_clock::now();
    ScrewCoordinates l_screwCoord(*l_robotModel, c_pandaTipLink);
    auto l_stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> l_ms = l_stop - l_start;
    RCLCPP_INFO_STREAM(LOGGER, "Screw Coordinates constructed in " << l_ms.count() << " ms");
    //printScrewInfo(l_screwCoord);

    //Constructing Dual Quaternions from Screw Coordinates
    RCLCPP_INFO_STREAM(LOGGER, "Computing " << l_screwCoord.getJointsNames().size()+1 << " dual quaternions representing the screw coordinates");
    l_start = std::chrono::high_resolution_clock::now();
    std::vector<DualQuaternion> l_screwDQ;
    for (size_t i = 0; i < l_screwCoord.getJointsNames().size(); i++)
    {
        l_screwDQ.push_back(DualQuaternion( l_screwCoord.getScrewAxes().at(i), l_screwCoord.getPositions().at(i) ));
    }

    //Check that all dual quaternions are unit dual quaternions
    for (auto &&l_dq : l_screwDQ)
    {
        try
        {
            const bool l_isUnit = l_dq.isUnit(c_tolerance);
            if(!l_isUnit)
            {
                throw(l_isUnit);
            }
        }
        //todo change to a parameter to an error for better error handling
        catch(const bool& e)
        {
            RCLCPP_INFO_STREAM(LOGGER, "DQ is unit: " << e);
        }
        
    }
    
    DualQuaternion l_tip2BaseInit(l_screwCoord.getTip2BaseInit());
    l_stop = std::chrono::high_resolution_clock::now();
    l_ms = l_stop - l_start;
    RCLCPP_INFO_STREAM(LOGGER, "Dual Quaternions constructions took " << l_ms.count() << " ms");

    //Constructing Robot State
    moveit::core::RobotStatePtr l_robotState(new moveit::core::RobotState(l_robotModel));
    l_robotState->enforceBounds();
    l_robotState->setToDefaultValues();
    const moveit::core::JointModelGroup* l_jointModelGroup = l_robotModel->getJointModelGroup("panda_arm");
    //const std::vector<std::string>& l_jointNames = l_jointModelGroup->getVariableNames();

    //First Try, apply joint values, compute and compare FK
    std::vector<double> l_jointValuesReady_rad ={0, -0.785, 0, -2.356, 0, 1.571, 0.785};
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    std::vector<double> l_jointValues_rad;
    l_robotState->copyJointGroupPositions(l_jointModelGroup, l_jointValues_rad);
    const Eigen::Isometry3d& l_eeStateMoveIt = l_robotState->getGlobalLinkTransform("panda_link8");
    l_stop = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(LOGGER, "MoveIt FK Returned : ");
    printEigenIsometry(l_eeStateMoveIt);

    Eigen::Isometry3d l_eeStateDQ;
    l_start = std::chrono::high_resolution_clock::now();
    dualQuaternionForwardKinematics(l_screwDQ, l_tip2BaseInit, l_jointValues_rad, l_eeStateDQ);
    l_stop = std::chrono::high_resolution_clock::now();
    l_ms = l_stop - l_start;
    RCLCPP_INFO_STREAM(LOGGER, "Dual Quaternions FK took " << l_ms.count() << " ms. And result is: ");
    printEigenIsometry(l_eeStateDQ);
    RCLCPP_INFO_STREAM(LOGGER, "Dual Quaternions FK and MoveIt FK match: " << l_eeStateDQ.isApprox(l_eeStateMoveIt, c_tolerance));

    //Second Try to make sure twists were not damaged
    l_robotState->setToRandomPositions(l_jointModelGroup);
    l_robotState->copyJointGroupPositions(l_jointModelGroup, l_jointValues_rad);
    const Eigen::Isometry3d& l_eeStateMoveIt2 = l_robotState->getGlobalLinkTransform("panda_link8");
    RCLCPP_INFO(LOGGER, "MoveIt FK Returned : ");
    printEigenIsometry(l_eeStateMoveIt2);

    l_start = std::chrono::high_resolution_clock::now();
    dualQuaternionForwardKinematics(l_screwDQ, l_tip2BaseInit, l_jointValues_rad, l_eeStateDQ);
    l_stop = std::chrono::high_resolution_clock::now();
    l_ms = l_stop - l_start;
    RCLCPP_INFO_STREAM(LOGGER, "Dual Quaternions FK took " << l_ms.count() << " ms. And result is: ");
    printEigenIsometry(l_eeStateDQ);
    RCLCPP_INFO_STREAM(LOGGER, "Dual Quaternions FK and MoveIt FK match: " << l_eeStateDQ.isApprox(l_eeStateMoveIt2, c_tolerance));

    rclcpp::shutdown();
    return 0; 

}