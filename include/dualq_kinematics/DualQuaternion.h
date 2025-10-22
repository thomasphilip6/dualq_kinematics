#pragma once

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
            static constexpr Scalar c_tolerance = 1e-6;

            /**
             * @brief Construction with two Quaternions, one real and one dual part
             */
            DualQuaternion(
                const Quaternion& p_realPart,
                const Quaternion& p_dualPart
            );

            /**
             * @brief Construction with AxisAngle for orientation and vector for position (first rotation then translation)
             */
            DualQuaternion(
                const AngleAxis& p_angleAxis,
                const Translation& p_translation
            );

            /**
             * @brief Construction with AxisAngle for orientation and vector for position (first translation then rotation)
             */
            DualQuaternion(
                const Translation& p_translation,
                const AngleAxis& p_angleAxis      
            );

            /**
             * @brief Construction with screw axis (rotation Axis & position)
             */
            DualQuaternion(
                const Translation& p_rotationAxis,
                const Translation& p_position
            );

            /**
             * @brief Construction with Quaternion for orientation and vector for position, rotation then translation
             */
            DualQuaternion(
                const Quaternion& p_orientation,
                const Translation& p_translation
            );

            /**
             * @brief Construction with Quaternion for orientation and vector for position, translation then rotation
             */
            DualQuaternion(
                const Translation& p_translation,
                const Quaternion& p_orientation
            );

            /**
             * @brief Construction with Transformation Matrix
             */
            DualQuaternion(
                const Transform& p_transform
            );

            /**
             * @brief Returns the dual quaternion multiplication of the object and another instance
             */
            DualQuaternion operator*(const DualQuaternion& p_other) const;

            /**
             * @brief Returns the multiplication of the dual Quaternion with a scalar value
             */
            DualQuaternion operator*(const Scalar p_scalar) const;

            /**
             * @brief Returns true if object and the other instance are the same
             */
            bool isApprox(const DualQuaternion& p_other, const Scalar p_tolerance) const;

            void print() const;

            /**
             * @brief Returns the conjugate of the dual quaternion
             */
            DualQuaternion conjugate() const;

            /**
             * @brief  Returns true if dual quaternion is a unit one
             */
            bool isUnit(const Scalar p_tolerance) const;

            /**
             * @brief Inverses the dual quaternion and returns true if success
             */
            bool invert(const Scalar p_tolerance);

            /**
             * @brief Returns the inverse of the dual quaternion if it exists, not optimized because of copies
             */
            std::optional<DualQuaternion> inverse(const Scalar p_tolerance) const;

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
            bool m_isUnit = false;
    };
}

#include "dualq_kinematics/DualQuaternion.tpp"