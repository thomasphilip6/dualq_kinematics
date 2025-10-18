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

            /**
             * @brief Construction with two Quaternions, one real and one dual part
             */
            DualQuaternion(
                const Quaternion& p_realPart,
                const Quaternion& p_dualPart
            );

            /**
             * @brief Construction with AxisAngle for orientation and vector for position
             */
            DualQuaternion(
                AngleAxis& p_angleAxis,
                Translation& p_translation
            );

            /**
             * @brief Construction with screw axis (rotation Axis & position)
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

            /**
             * @brief Returns the dual quaternion multiplication of the object and another instance
             */
            DualQuaternion operator*(const DualQuaternion& p_other) const;

            /**
             * @brief Returns true if object and the other instance are the same
             */
            bool compare(const DualQuaternion& p_other, const Scalar p_tolerance) const;

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
            bool invert();

            /**
             * @brief Returns the inverse of the dual quaternion if it exists, not optimized because of copies
             */
            std::optional<DualQuaternion> inverse() const;

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