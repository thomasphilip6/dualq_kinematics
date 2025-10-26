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

#include <cmath>

namespace dualq_kinematics
{
    
    template<typename Scalar>
    class ScrewCoordinates 
    {
        public:

            using Translation = Eigen::Translation<Scalar, 3>;
            using Transform = Eigen::Transform<Scalar,3, Eigen::Isometry>;

            ScrewCoordinates(const moveit::core::RobotModel& robot_model);
        
        private:
            std::vector<Translation> l_screwAxis;
            std::vector<Translation> l_positions;
            std::vector<std::string> l_joint;
            Transform l_ee;
            
    }
};

#include "dualq_kinematics/ScrewCoordinates.tpp"