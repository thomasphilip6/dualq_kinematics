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
     * @brief Solves Inverse and Forward Kinematics for the Panda Franka Emika Robot as described in an urdf. Details on the mathematical procedure are provided in the README
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

            /**
             * @brief Constructor for the solver 
             * @param p_screwCoordinatesPtr pointer to the ScrewCoordinates object
             */
            explicit FrankaKinSolver(const ScrewCoordinates* p_screwCoordinatesPtr);

            /**
             * @brief Sets an emergency for q1 to be used in singular position
             * @param p_q1Value_rad 
             * @return returns true if value is within bounds
             */
            bool setEmergencyQ1(const Scalar& p_q1Value_rad);

            /**
             * @brief Sets an emergency for q5 to be used in singular position
             * @param p_q5Value_rad 
             * @return returns true if value is within bounds
             */
            bool setEmergencyQ5(const Scalar& p_q5Value_rad);

            /**
             * @brief Computes Forward Kinematics for the tip of the robot
             * @param p_jointValues_rad joint values 
             * @param p_tip2BaseComputed computed transform
             */
            void computeTipFK(const std::vector<double>& p_jointValues_rad, Eigen::Isometry3d& p_tip2BaseComputed) const noexcept;

            /**
             * @brief Computes Elbow Position, TODO : will find swivel angle
             */
            void computeElbowPosition(const Eigen::Isometry3d& p_tip2BaseWanted, const Scalar& p_q5, const Scalar& p_q6, const Scalar& p_q7, Quaternion& p_screw) const noexcept;

            /**
             * @brief Computes the position of the wrist (intersection of joint 5&6) from a target pose and q7
             * @param p_tip2BaseWanted target pose
             * @param p_q7 q7 value in rad
             * @param p_wrist vector reprensenting the wrist position
             */
            void computeWristPosition(const Eigen::Isometry3d& p_tip2BaseWanted, const Scalar& p_q7, Vector3& p_wrist) const noexcept;

            /**
             * @brief Solves the Inverse Kinematics as 6DOF with q7 fixed
             * @param p_tip2BaseWanted target pose
             * @param p_q7 q7 value in rad
             * @param p_solutions vector of joint values within bounds solving the IK
             */
            void compute6DOFIK(const Eigen::Isometry3d& p_tip2BaseWanted, const Scalar& p_q7, std::vector<std::vector<Scalar>>& p_solutions) const noexcept;
        
        private:

            // Shared pointer of ScrewCoordinates object
            std::shared_ptr<ScrewCoordinates> m_screwCoordinatesPtr;

            // Tip2BaseInit shared pointer
            std::shared_ptr<DualQuaternion> m_tip2BaseInitPtr;

            // Screw Coordinates reprensented as dual quaternions
            std::vector<DualQuaternion> m_screwCoordinatesDualQ;

            // q1 to be used in singular position
            Scalar m_emergencyQ1_rad;

            // q5 to be used in singular position
            Scalar m_emergencyQ5_rad;
    };
}

#include "dualq_kinematics/FrankaKinSolver.tpp"