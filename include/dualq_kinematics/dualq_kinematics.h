//  \todo:  deal with license

/*  Author: Thomas Philip
    Desc:   Use Kinematics plugin interface to use dual quaternions for Forward Kinematics
            \todo: add inverse kinematics
*/

#pragma once

//ROS2
#include <rclcpp/rclcpp.hpp>

//System
#include <memory>

//ROS2 msgs
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/kinematic_solver_info.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/robot_state.h>

namespace dualq_kinematics
{
    class dualQuaternionsKinematicsPlugin : public kinematics::KinematicsBase
    {
        public:

            dualQuaternionsKinematicsPlugin();

            bool getPositionFK(
                const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                std::vector<geometry_msgs::msg::Pose>& poses) const override;

            bool initialize(
                const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                const std::string& group_name, const std::string& base_name,
                const std::vector<std::string>& tip_frames, double search_discretization) override;

            void setValues(
                const std::string& robot_description,
                const std::string& group_name,
                const std::string& base_frame,
                const std::string& tip_frame,
                double search_discretization) override;
        
        
        protected:

            std::string robot_description_;
            std::string group_name_;
            std::string base_frame_;
            std::vector<std::string> tip_frames_;
        
        private:
                

    };
}