#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <array>
#include <string>
#include <optional>
#include <vector>
#include <map>

#include <cmath>

namespace dualq_kinematics
{
    
    template<typename Scalar>
    class ScrewCoordinates 
    {
        public:

            using Translation = Eigen::Translation<Scalar, 3>;
            using Transform = Eigen::Transform<Scalar,3, Eigen::Isometry>;
            using Quaternion = Eigen::Quaternion<Scalar>;

            ScrewCoordinates(const moveit::core::RobotModel& robot_model);
        
        private:
            std::vector<Translation> m_screwAxis;
            std::vector<Translation> m_positions;
            std::vector<std::string> m_joints;
            Transform l_ee;

            void transformToScrewCoordinates(std::vector<urdf::Pose>& p_jnt2ParentPoses);
            
    };
}

#include "dualq_kinematics/ScrewCoordinates.tpp"