#pragma once

// #include <Eigen/src/Geometry/Quaternion.h>
// #include <Eigen/src/Geometry/AngleAxis.h>
// #include <Eigen/src/Geometry/Transform.h>
// #include <Eigen/src/Geometry/Translation.h>
// #include <Eigen/src/Geometry/Transform.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <optional>

namespace dualq_kinematics
{

    template<typename Scalar>
    class DualQuaternion 
    {
        public:

            using Quaternion = Eigen::Quaternion<Scalar>;
            using AngleAxis = Eigen::AngleAxis<Scalar>;
            using Translation = Eigen::Translation<Scalar, 3> ;
            using Transform = Eigen::Transform<Scalar,3, Eigen::Isometry>;

            /**
             * @brief Construction with two Quaternions, one real and one dual part
             */
            DualQuaternion(
                Quaternion& p_realPart,
                Quaternion& p_dualPart
            );

            /**
             * @brief Construction with AxisAngle for orientation and vector for position
             */
            DualQuaternion(
                AngleAxis& p_angleAxis,
                Translation& p_translation
            );

            /**
             * @brief Construction with screw axis (roation Axis & position)
             */
            DualQuaternion(
                Translation& p_rotationAxis,
                Translation& p_position
            );

            /**
             * @brief Construction with Quaternion for orientation and vector for position
             */
            DualQuaternion(
                Quaternion& p_orientation,
                Translation& p_position
            );

            /**
             * @brief Construction with Transformation Matrix
             */
            DualQuaternion(
                Transform& p_transform
            );

            DualQuaternion operator*(const DualQuaternion& p_other) const;

            /**
             * @brief returns real part of dual quaternion
             */
            const Quaternion& getRealPart() const;

            /**
             * @brief returns dual part of dual quaternion
             */
            const Quaternion& getDualPart() const;
            
        private:
            Quaternion m_realPart;
            Quaternion m_dualPart;
    };
}