#pragma once

#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

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

            // Tolerance used to check if problem can be solved or not
            static constexpr Scalar c_tolerance = 1e-5;

            /**
            * @brief Construction with no arguments
             */
            FirstPadenKahanProblem();

            /**
            * @brief Construction with all arguments, solves the Paden Kahan Subproblem 1
            * @param p_pointOnLine point on the screw axis
            * @param p_paxis screw axis
            * @param p_startPoint point p before rotation
            * @param p_endPoint point after rotation
            * @param p_minValue_rad minimum value that the angle can take in rad
            * @param p_maxValue_rad maximum value that the angle can take in rad
             */
            FirstPadenKahanProblem(
                const Quaternion& p_pointOnLine, 
                const Quaternion& p_axis, 
                const Quaternion& p_startPoint, 
                const Quaternion& p_endPoint,
                const double& p_minValue_rad,
                const double& p_maxValue_rad
            );

            /**
            * @brief Solves the Paden Kahan Subproblem 1
            * @param p_pointOnLine point on the screw axis
            * @param p_paxis screw axis
            * @param p_startPoint point p before rotation
            * @param p_endPoint point after rotation
            * @param p_minValue_rad minimum value that the angle can take in rad
            * @param p_maxValue_rad maximum value that the angle can take in rad
             */
            inline void compute(
                const Quaternion& p_pointOnLine, 
                const Quaternion& p_axis, 
                const Quaternion& p_startPoint, 
                const Quaternion& p_endPoint,
                const double& p_minValue_rad,
                const double& p_maxValue_rad
            ) noexcept;

            // to prepare a new version of compute to be compared for speed effiency
            static inline void compute(const Quaternion& p_pointOnLine, const Quaternion& p_axis, const Quaternion& p_startPoint, const Quaternion& p_endPoint, Scalar& p_result) noexcept;

            /**
            * @brief Solves the Paden Kahan Subproblem 1 from the projection of start and end point projected on the circle
            * @param p_paxis screw axis
            * @param p_xProjected start point projected non the circle
            * @param p_yProjected end point projected non the circle
            * @param p_minValue_rad minimum value that the angle can take in rad
            * @param p_maxValue_rad maximum value that the angle can take in rad
             */
            inline void computeFromProjectedPoints(
                const Quaternion& p_axis, 
                const Quaternion& p_xProjected, 
                const Quaternion& p_yProjected,
                const double& p_minValue_rad,
                const double& p_maxValue_rad
            ) noexcept;

            /**
             * @brief Returns the result of the Subproblem 1 as an optional (value has to be within bounds)
             * @return the result of the subproblem
             */
            const std::optional<Scalar>& getResult() const;

            /**
             * @brief Returns the flag of singularity 
             * @return false if subproblem couldn't be solved due to a projection problem
             */
            const bool& getSingularityStatus() const;

            /**
             * @brief Compares two floating point numbers
             * @param p_a first number to be compared
             * @param p_b second number to be compared
             * @param p_tolerance tolerance to be used for comparison
             * @return true the numbers are the same with the tolerance provided
             */
            static bool compareFloatNum(Scalar p_a, Scalar p_b, Scalar p_tolerance) noexcept;
        
        private:
            
            // Result of Subprobem
            std::optional<Scalar> m_resultAngle_rad;

            // Singularity flag
            bool m_singularityFlag;
    };

    template<typename Scalar>
    /**
     * @class SecondPadenKahanProblem
     * @brief Solves the second Paden-Kahan subproblem with quaternions. 
     * 
     * Wikipedia definition : "This problem corresponds to rotating p around the axis of ξ2 by θ2, then rotating it around the axis of ξ1 by θ1 , so that the final location of p is coincident with q"
     */
    class SecondPadenKahanProblem
    {
        public:

            using Quaternion = Eigen::Quaternion<Scalar>;
            using Vector3 = Eigen::Matrix<Scalar, 3, 1>; 
            using FirstPadenKahanProblem = dualq_kinematics::FirstPadenKahanProblem<Scalar>;
            using DualQuaternion = dualq_kinematics::DualQuaternion<Scalar>;

            /**
            * @brief Construction with no arguments
             */
            SecondPadenKahanProblem();

            /**
            * @brief Construction with all arguments, solves the Paden Kahan Subproblem 2
            * @param p_pointOnLines point on both screw axes
            * @param p_paxis1 screw axis 1 ξ1
            * @param p_paxis2 screw axis 2 ξ2
            * @param p_startPoint point p before rotations
            * @param p_endPoint point after rotations
            * @param p_minValue1_rad minimum value that the angle1 θ1 can take in rad
            * @param p_maxValue1_rad maximum value that the angle1 θ1 take in rad
            * @param p_minValue2_rad minimum value that the angle2 θ2 can take in rad 
            * @param p_maxValue2_rad maximum value that the angle2 θ2 can take in rad
             */
            SecondPadenKahanProblem(
                const Quaternion& p_pointOnLines, 
                const Quaternion& p_axis1, 
                const Quaternion& p_axis2, 
                const Quaternion& p_startPoint, 
                const Quaternion& p_endPoint,
                const double& p_minValue1_rad,
                const double& p_maxValue1_rad,
                const double& p_minValue2_rad,
                const double& p_maxValue2_rad
            );

            /**
            * @brief Solves the Paden Kahan Subproblem 2
            * @param p_pointOnLines point on both screw axes
            * @param p_paxis1 screw axis 1 ξ1
            * @param p_paxis2 screw axis 2 ξ2
            * @param p_startPoint point p before rotations
            * @param p_endPoint point after rotations
            * @param p_minValue1_rad minimum value that the angle1 θ1 can take in rad
            * @param p_maxValue1_rad maximum value that the angle1 θ1 can take in rad
            * @param p_minValue2_rad minimum value that the angle2 θ2 can take in rad
            * @param p_maxValue2_rad maximum value that the angle2 θ2 can take in rad
             */
            bool compute(
                const Quaternion& p_pointOnLines, 
                const Quaternion& p_axis1, 
                const Quaternion& p_axis2, 
                const Quaternion& p_startPoint, 
                const Quaternion& p_endPoint,
                const double& p_minValue1_rad,
                const double& p_maxValue1_rad,
                const double& p_minValue2_rad,
                const double& p_maxValue2_rad
            ) noexcept;

            /**
            * @brief Returns solutions for the first rotation θ1 around ξ1
            * @return vector of solutions for the first angle (to be combined with the one for the second angle)
            */
            const std::vector<Scalar>& getAngle1Result() const;

            /**
            * @brief Returns solutions for the second rotation θ2 around ξ2
            * @return vector of solutions for the second angle (to be combined with the one for the first angle)
            */
            const std::vector<Scalar>& getAngle2Result() const;

            /**
            * @brief Fills a vector of quaternions representing the intersection points of the two circles
            * @param p_axis1 ξ1
            * @param p_axis2 ξ2
            * @param p_x starting point projected
            * @param p_y end point projected
            * @param p_intersections vector of intersection points (as pure quaternions)
            */
            inline static void computeIntersection(const Quaternion& p_axis1, const Quaternion& p_axis2, Quaternion& p_x, Quaternion& p_y, std::vector<Quaternion>& p_intersections) noexcept;
        
        private:

            // Results for first rotation
            std::vector<Scalar> m_resultsAngle1_rad;

            // Results for second rotation
            std::vector<Scalar> m_resultsAngle2_rad;

            // Vector of 1st Paden Kahan Subproblems used for rotation around axis 1 : θ1 around ξ1 from intersection point to end point
            std::vector<FirstPadenKahanProblem> m_axis1Rotations;
            
            // Vector of 1st Paden Kahan Subproblems used for rotation around axis 2 : θ2 around ξ2 from start point point to intersection point
            std::vector<FirstPadenKahanProblem> m_axis2Rotations;

            /**
             * @brief Returns the squared norm of the vector part of a quaternion
             * @param p_quat 
             * @return squared norm
             */
            Scalar squaredNormOfQuatVectPart(Quaternion& p_quat);

    };

    template<typename Scalar>
    /**
     * @class SecondPadenKahanProblemExt
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

            std::vector<FirstPadenKahanProblem> m_axis1Rotations;
            std::vector<FirstPadenKahanProblem> m_axis2Rotations;

    };

    template<typename Scalar>
    /**
     * @class ThirdPadenKahanProblem
     * @brief Solves the third Paden-Kahan subproblem with quaternions
     * 
     * Wikipedia definition : In this problem, a point p is rotated about an axis ξ until the point is a distance δ from a point q
     */
    class ThirdPadenKahanProblem
    {
        public:

            using Quaternion = Eigen::Quaternion<Scalar>;
            using Vector3 = Eigen::Matrix<Scalar, 3, 1>; 
            using FirstPadenKahanProblem = dualq_kinematics::FirstPadenKahanProblem<Scalar>;
            using DualQuaternion = dualq_kinematics::DualQuaternion<Scalar>;

            /**
            * @brief Construction with no arguments
            */
            ThirdPadenKahanProblem();

            /**
            * @brief Construction with all arguments, solves the Paden Kahan Subproblem 3
            * @param p_pointOnLine point on both screw axis
            * @param p_paxis screw axis 
            * @param p_startPoint point p before rotations
            * @param p_endPoint point after rotations
            * @param p_distanceToEnd distance to the end point δ that must be respected
            * @param p_minValue_rad minimum value that the angle can take in rad
            * @param p_maxValue_rad maximum value that the angle can take in rad
            */
            ThirdPadenKahanProblem(
                const Quaternion& p_pointOnLine, 
                const Quaternion& p_axis, 
                const Quaternion& p_startPoint, 
                const Quaternion& p_endPoint, 
                const Scalar p_distanceToEnd,
                const double& p_minValue_rad,
                const double& p_maxValue_rad
            );

            /**
            * @brief Solves the Paden Kahan Subproblem 3
            * @param p_pointOnLine point on both screw axis
            * @param p_paxis screw axis 
            * @param p_startPoint point p before rotations
            * @param p_endPoint point after rotations
            * @param p_distanceToEnd distance to the end point δ that must be respected
            * @param p_minValue_rad minimum value that the angle can take in rad
            * @param p_maxValue_rad maximum value that the angle can take in rad
            */
            void compute(
                const Quaternion& p_pointOnLine, 
                const Quaternion& p_axis, 
                const Quaternion& p_startPoint, 
                const Quaternion& p_endPoint, 
                const Scalar p_distanceToEnd,
                const double& p_minValue_rad,
                const double& p_maxValue_rad
            ) noexcept;

            /**
            * @brief Returns the solutions
            * @return vector of solutions
            */
            const std::vector<Scalar>& getResults() const;
        
        private:
            
            // Results
            std::vector< Scalar > m_results;

    };

}

#include "dualq_kinematics/PadenKahan.tpp"