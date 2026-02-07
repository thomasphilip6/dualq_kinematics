#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <optional>
#include <iostream>

#include <cmath>

namespace dualq_kinematics
{

    template<typename Scalar>
    /**
     * @class FrankaKinSolver
     * @brief todo
     */
    class FrankaKinSolver 
    {
        public:

            using Quaternion = Eigen::Quaternion<Scalar>;
            using AngleAxis = Eigen::AngleAxis<Scalar>;
            using Translation = Eigen::Translation<Scalar, 3> ;
            using Transform = Eigen::Transform<Scalar,3, Eigen::Isometry>;
            using RotationMatrix = Eigen::Matrix<Scalar, 3, 3>;
            using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
            using ScrewCoordinates = dualq_kinematics::ScrewCoordinates<Scalar>;
            using DualQuaternion = dualq_kinematics::DualQuaternion<Scalar>;

            static constexpr double c_tolerance = 1e-6;

            FrankaKinSolver(const ScrewCoordinates& p_screwCoordinates);

            void computeTipFK(std::vector<double>& p_jointValues_rad, Eigen::Isometry3d& p_tip2BaseComputed) const;

            void computeWristPosition(const Eigen::Isometry3d& p_tip2BaseWanted, const Scalar p_q7, Vector3& p_wrist) const;
        
        private:

            std::optional<ScrewCoordinates> m_screwCoordinates;
            std::optional<DualQuaternion> m_tip2BaseInit;
            std::vector<DualQuaternion> m_screwCoordinatesDualQ;

    };
}

#include "dualq_kinematics/FrankaKinSolver.tpp"