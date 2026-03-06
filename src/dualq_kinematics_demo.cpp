#include "dualq_kinematics/DualQuaternion.h"
#include "dualq_kinematics/ScrewCoordinates.h"
#include "dualq_kinematics/PadenKahan.h"
#include "dualq_kinematics/FrankaKinSolver.h"

//ROS2
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//C++
#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <thread> 

//link of the robot considered the tip of the kinematic chain
const std::string c_pandaTipLink = "panda_link8";

//Tolerance of match between MoveIt Forward Kinematics and Dual Quaternion FK
constexpr double c_tolerance = 1e-6;
using DualQuaternion = dualq_kinematics::DualQuaternion<double>;
using ScrewCoordinates = dualq_kinematics::ScrewCoordinates<double>;
using FrankaKinSolver = dualq_kinematics::FrankaKinSolver<double>;
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
static const std::string PLANNING_GROUP = "panda_arm";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("dualq_kinematics_demo");
constexpr int c_repetitions = 1000;

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


void printEigenIsometry(const Eigen::Isometry3d& p_transformationMatrix)
{
    RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << p_transformationMatrix.translation() << "\n");
    RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << p_transformationMatrix.rotation() << "\n");
}

/**
 * @brief Prints a vector of joint values
 * @param p_screwCoord vector to be printed
 */
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "dualq_kinematics_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    //adds executor for moveItVisualTools
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    //Loading robot and constructing screw coordinates
    //rdf_loader::RDFLoader rdf_loader(node, ROBOT_DESCRIPTION_PARAM);
    //moveit::core::RobotModelPtr l_robotModel = std::make_shared<moveit::core::RobotModel>(rdf_loader.getURDF(), rdf_loader.getSRDF());

    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::core::RobotModelConstPtr l_robotModel = move_group.getRobotModel();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* l_jointModelGroup = l_robotModel->getJointModelGroup(PLANNING_GROUP);

    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "panda_link0", "dual_quaternions_demo", l_robotModel};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    moveit_visual_tools.publishText(text_pose, "Dual_Quaternions_Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    moveit_visual_tools.trigger();
    
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
    FrankaKinSolver l_frankaKin(l_screwCoord);

    l_stop = std::chrono::high_resolution_clock::now();
    l_ms = l_stop - l_start;
    RCLCPP_INFO_STREAM(LOGGER, "Dual Quaternions constructions & FrankaKinSolver took " << l_ms.count() << " ms");

    //Constructing Robot State
    moveit::core::RobotStatePtr l_robotState(new moveit::core::RobotState(l_robotModel));
    l_robotState->enforceBounds();
    l_robotState->setToDefaultValues();
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

    // ------------------------- FK Demo ----------------------- //

    moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to begin demo");
    moveit_visual_tools.publishText(text_pose, "Forward_Kinematics", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    moveit_visual_tools.trigger();

    Eigen::Isometry3d l_eeStateDQ;
    l_start = std::chrono::high_resolution_clock::now();
    l_frankaKin.computeTipFK(l_jointValues_rad, l_eeStateDQ);
    l_stop = std::chrono::high_resolution_clock::now();
    l_ms = l_stop - l_start;
    RCLCPP_INFO_STREAM(LOGGER, "Dual Quaternions FK took " << l_ms.count() << " ms. And result is: ");
    printEigenIsometry(l_eeStateDQ);
    RCLCPP_INFO_STREAM(LOGGER, "Dual Quaternions FK and MoveIt FK match: " << l_eeStateDQ.isApprox(l_eeStateMoveIt, c_tolerance));

    //Second Try to make sure twists were not damaged
    l_robotState->setToRandomPositions(l_jointModelGroup);
    l_robotState->copyJointGroupPositions(l_jointModelGroup, l_jointValues_rad);
    l_robotState->getGlobalLinkTransform("panda_link8");
    RCLCPP_INFO(LOGGER, "MoveIt FK Returned : ");
    printEigenIsometry(l_eeStateMoveIt);

    //Make Robot Move
    move_group.setJointValueTarget(l_jointValues_rad);
    moveit::planning_interface::MoveGroupInterface::Plan l_plan;
    bool l_planSuccess = (move_group.plan(l_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(l_planSuccess)
    {
        move_group.execute(l_plan);
    }

    moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to see pose that FK computed");
    moveit_visual_tools.trigger();


    l_start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < c_repetitions; i++)
    {
        l_frankaKin.computeTipFK(l_jointValues_rad, l_eeStateDQ);
    }
    l_stop = std::chrono::high_resolution_clock::now();
    double l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count();
    RCLCPP_INFO_STREAM(LOGGER, "Dual Quaternions FK took (over 1000 tries) " << l_micros/c_repetitions << " us. And result is: ");
    printEigenIsometry(l_eeStateDQ);
    RCLCPP_INFO_STREAM(LOGGER, "Dual Quaternions FK and MoveIt FK match: " << l_eeStateDQ.isApprox(l_eeStateMoveIt, c_tolerance));

    moveit_visual_tools.publishAxisLabeled(l_eeStateDQ, "Pose_obtained");
    moveit_visual_tools.trigger();

    // ------------------------- IK Demo ----------------------- //

    l_jointValuesReady_rad = {2.13528, 0.49, 0.16, -0.42, 0.18, 2.14, 0.785};
    l_robotState->setJointGroupPositions(l_jointModelGroup, l_jointValuesReady_rad);
    l_robotState->getGlobalLinkTransform("panda_link8");
    const Eigen::Isometry3d l_eeWanted = l_eeStateMoveIt;
  
    moveit_visual_tools.publishText(text_pose, "Inverse_Kinematics", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    moveit_visual_tools.trigger();
    moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to execute IK computations");
    moveit_visual_tools.publishAxisLabeled(l_eeWanted, "Pose_wanted");
    moveit_visual_tools.trigger();

    std::vector<std::vector<double>> l_IKSolutions = l_frankaKin.compute6DOFIK(l_eeStateMoveIt, l_jointValuesReady_rad.at(6));

    for (auto &&l_solution : l_IKSolutions)
    {
        bool l_inBounds = l_jointModelGroup->satisfiesPositionBounds(l_solution.data(), 0.0);
        printJointVector(l_solution);
        if(l_inBounds)
        {
            l_robotState->setJointGroupPositions(l_jointModelGroup, l_solution);
            l_robotState->getGlobalLinkTransform("panda_link8");
            RCLCPP_INFO_STREAM(LOGGER, "IK solution within bounds match wanted one: " << l_eeWanted.isApprox(l_eeStateMoveIt, c_tolerance));
            move_group.setJointValueTarget(l_solution);
            bool l_planSuccess = (move_group.plan(l_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if(l_planSuccess)
            {
                move_group.execute(l_plan);
            }
            else
            {
                RCLCPP_ERROR(LOGGER, "Planing failed!");
            }


        }
    }

    // ------------------------- IK Performance Demo ----------------------- //

    moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to execute performance IK computations");
    moveit_visual_tools.publishText(text_pose, "Inverse_Kinematics_Performance", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    moveit_visual_tools.trigger();
    
    l_start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < c_repetitions; i++)
    {
        l_IKSolutions = l_frankaKin.compute6DOFIK(l_eeStateMoveIt, l_jointValuesReady_rad.at(6));
    }
    l_stop = std::chrono::high_resolution_clock::now();
    l_micros = std::chrono::duration<double, std::micro>(l_stop - l_start).count();
    RCLCPP_INFO_STREAM(LOGGER, "IK took (over 1000 tries) " << l_micros/c_repetitions << " us. And result is: ");

    rclcpp::shutdown();
    spinner.join();
    return 0; 

}