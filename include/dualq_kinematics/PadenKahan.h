#pragma once

#include "dualq_kinematics/DualQuaternion.h"
#include "dualq_kinematics/ScrewCoordinates.h"

namespace dualq_kinematics
{

    //todo add Doxygen comments
    template<typename Scalar>
    class FirstPadenKahanProblem
    {
        public:

            using Translation = Eigen::Translation<Scalar, 3>;
            using Transform = Eigen::Transform<Scalar,3, Eigen::Isometry>;
            using Quaternion = Eigen::Quaternion<Scalar>;
            using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
            using DualQuaternion = dualq_kinematics::DualQuaternion<Scalar>;
            using ScrewCoordinates = dualq_kinematics::ScrewCoordinates<Scalar>;

            FirstPadenKahanProblem(Vector3& p_pointOnLine, Vector3& p_axis, Vector3& p_startPoint, Vector3& p_endPoint);

            bool solveProblem();

            const Scalar& getResult() const;
        
        private:
            Scalar m_resultAngle;

    };

}

#include "dualq_kinematics/PadenKahan.tpp"