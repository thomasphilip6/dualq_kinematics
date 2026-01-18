#pragma once

#include "dualq_kinematics/DualQuaternion.h"
#include "dualq_kinematics/ScrewCoordinates.h"

#include <cmath>

namespace dualq_kinematics
{

    //todo add Doxygen comments
    template<typename Scalar>
    /**
     * @class FirstPadenKahanProblem
     * @brief Solves the first Paden-Kahan subproblem with quaternions
     */
    class FirstPadenKahanProblem
    {
        public:

            using Translation = Eigen::Translation<Scalar, 3>;
            using Transform = Eigen::Transform<Scalar,3, Eigen::Isometry>;
            using Quaternion = Eigen::Quaternion<Scalar>;
            using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
            using DualQuaternion = dualq_kinematics::DualQuaternion<Scalar>;
            using ScrewCoordinates = dualq_kinematics::ScrewCoordinates<Scalar>;
            static constexpr Scalar c_tolerance = 1e-6;

            FirstPadenKahanProblem(Vector3& p_pointOnLine, Quaternion& p_axis, Vector3& p_startPoint, Vector3& p_endPoint);

            const std::optional<Scalar>& getResult() const;

            static bool compareFloatNum(Scalar p_a, Scalar p_b, Scalar p_tolerance);
        
        private:

            std::optional<Scalar> m_resultAngle_rad;
    };

}

#include "dualq_kinematics/PadenKahan.tpp"