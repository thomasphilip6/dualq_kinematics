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
            static constexpr Scalar c_tolerance = 1e-4;

            FirstPadenKahanProblem();

            FirstPadenKahanProblem(const Quaternion& p_pointOnLine, const Quaternion& p_axis, const Quaternion& p_startPoint, const Quaternion& p_endPoint);

            void compute(const Quaternion& p_pointOnLine, const Quaternion& p_axis, const Quaternion& p_startPoint, const Quaternion& p_endPoint);

            void computeFromProjectedPoints(const Quaternion& p_axis, const Quaternion& p_xProjected, const Quaternion& p_yProjected, bool p_checkConditions);

            const std::optional<Scalar>& getResult() const;

            static bool compareFloatNum(Scalar p_a, Scalar p_b, Scalar p_tolerance);
        
        private:

            std::optional<Scalar> m_resultAngle_rad;
    };

    template<typename Scalar>
    /**
     * @class SecondPadenKahanProblem
     * @brief Solves the second Paden-Kahan subproblem with quaternions
     */
    class SecondPadenKahanProblem
    {
        public:

            using Quaternion = Eigen::Quaternion<Scalar>;
            using Vector3 = Eigen::Matrix<Scalar, 3, 1>; 
            using FirstPadenKahanProblem = dualq_kinematics::FirstPadenKahanProblem<Scalar>;
            using DualQuaternion = dualq_kinematics::DualQuaternion<Scalar>;

            SecondPadenKahanProblem();

            SecondPadenKahanProblem(const Quaternion& p_pointOnLines, const Quaternion& p_axis1, const Quaternion& p_axis2, const Quaternion& p_startPoint, const Quaternion& p_endPoint);

            void compute(const Quaternion& p_pointOnLines, const Quaternion& p_axis1, const Quaternion& p_axis2, const Quaternion& p_startPoint, const Quaternion& p_endPoint);

            const std::vector<Scalar>& getAngle1Result() const;

            const std::vector<Scalar>& getAngle2Result() const;

            static std::vector<Quaternion> computeIntersection(const Quaternion& p_axis1, const Quaternion& p_axis2, Quaternion& p_x, Quaternion& p_y);
        
        private:

            std::vector<Scalar> m_resultsAngle1_rad;
            std::vector<Scalar> m_resultsAngle2_rad;

            std::vector<FirstPadenKahanProblem> m_firstRotations;
            std::vector<FirstPadenKahanProblem> m_secondRotations;

            Scalar squaredNormOfQuatVectPart(Quaternion& p_quat);

    };

    template<typename Scalar>
    /**
     * @class ThirdPadenKahanProblem
     * @brief Solves the extended 2nd Paden-Kahan subproblem with quaternions : two rotations about not intersecting axes (they can't be colinear)
     */
    class SecondPadenKahanProblemExt
    {
        public:

            using Quaternion = Eigen::Quaternion<Scalar>;
            using Vector3 = Eigen::Matrix<Scalar, 3, 1>; 
            using FirstPadenKahanProblem = dualq_kinematics::FirstPadenKahanProblem<Scalar>;
            using DualQuaternion = dualq_kinematics::DualQuaternion<Scalar>;

            SecondPadenKahanProblemExt();

            SecondPadenKahanProblemExt(const Quaternion& p_pointOnLine1, const Quaternion& p_pointOnLine2,  const Quaternion& p_axis1, const Quaternion& p_axis2, const Quaternion& p_startPoint, const Quaternion& p_endPoint);

            void compute(const Quaternion& p_pointOnLine1, const Quaternion& p_pointOnLine2, const Quaternion& p_axis1, const Quaternion& p_axis2, const Quaternion& p_startPoint, const Quaternion& p_endPoint);

            const std::vector<Scalar>& getAngle1Result() const;

            const std::vector<Scalar>& getAngle2Result() const;
    
        private:

            std::vector<Scalar> m_resultsAngle1_rad;
            std::vector<Scalar> m_resultsAngle2_rad;

            std::vector<FirstPadenKahanProblem> m_firstRotations;
            std::vector<FirstPadenKahanProblem> m_secondRotations;

    };

    template<typename Scalar>
    /**
     * @class ThirdPadenKahanProblem
     * @brief Solves the third Paden-Kahan subproblem with quaternions
     */
    class ThirdPadenKahanProblem
    {
        public:

            using Quaternion = Eigen::Quaternion<Scalar>;
            using Vector3 = Eigen::Matrix<Scalar, 3, 1>; 
            using FirstPadenKahanProblem = dualq_kinematics::FirstPadenKahanProblem<Scalar>;
            using DualQuaternion = dualq_kinematics::DualQuaternion<Scalar>;

            ThirdPadenKahanProblem();

            ThirdPadenKahanProblem(const Quaternion& p_pointOnLine, const Quaternion& p_axis, const Quaternion& p_startPoint, const Quaternion& p_endPoint, const Scalar p_distanceToEnd);

            void compute(const Quaternion& p_pointOnLine, const Quaternion& p_axis, const Quaternion& p_startPoint, const Quaternion& p_endPoint, const Scalar p_distanceToEnd);

            const std::vector<Scalar>& getResults() const;
        
        private:
            std::vector< Scalar > m_results;
            FirstPadenKahanProblem m_theta0Problem;

    };

}

#include "dualq_kinematics/PadenKahan.tpp"