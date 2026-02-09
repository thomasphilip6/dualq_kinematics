#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "dualq_kinematics/DualQuaternion.h"
#include "dualq_kinematics/PadenKahan.h"
#include "dualq_kinematics/ScrewCoordinates.h"

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

            using FirstPadenKahan = dualq_kinematics::FirstPadenKahanProblem<Scalar>;
            using SecondPadenKahan = dualq_kinematics::SecondPadenKahanProblem<Scalar>;
            using ThirdPadenKahan = dualq_kinematics::ThirdPadenKahanProblem<Scalar>;



            static constexpr Scalar c_dof = 7;

            static constexpr Scalar c_tolerance = 1e-6;

            FrankaKinSolver(const ScrewCoordinates& p_screwCoordinates);

            FrankaKinSolver(const std::vector<Vector3> p_screwAxes, const std::vector<Vector3> p_positions, const Eigen::Isometry3d p_tip2BaseInit);

            void computeTipFK(std::vector<double>& p_jointValues_rad, Eigen::Isometry3d& p_tip2BaseComputed) const;

            void computeWristPosition(const Eigen::Isometry3d& p_tip2BaseWanted, const Scalar p_q7, Vector3& p_wrist) const;

            std::vector<std::vector<Scalar>> compute6DOFIK(const Eigen::Isometry3d& p_tip2BaseWanted, const Scalar p_q7) const;
        
        private:

            std::optional<ScrewCoordinates> m_screwCoordinates;
            std::optional<DualQuaternion> m_tip2BaseInit;
            std::vector<DualQuaternion> m_screwCoordinatesDualQ;

            //depreciated, useful for unit test
            std::vector<Vector3> m_screwAxes;
            std::vector<Vector3> m_positionsOnScrew; 
            Eigen::Isometry3d m_tip2BaseInitTf;
    };
}

#include "dualq_kinematics/FrankaKinSolver.tpp"