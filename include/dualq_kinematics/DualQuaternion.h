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
     * @class DualQuaternion
     * @brief Provides the mathematical object "Dual Quaternion" and its operations
     */
    class DualQuaternion 
    {
        public:

            using Quaternion = Eigen::Quaternion<Scalar>;
            using AngleAxis = Eigen::AngleAxis<Scalar>;
            using Translation = Eigen::Translation<Scalar, 3> ;
            using Transform = Eigen::Transform<Scalar,3, Eigen::Isometry>;
            using RotationMatrix = Eigen::Matrix<Scalar, 3, 3>;
            static constexpr Scalar c_tolerance = 1e-6;

            /**
             * @brief Construction with two Quaternions, one real and one dual part
             * @param p_realPart a quaternion that is the real part of the dual quaternion
             * @param p_dualPart a quaternion that is the dual part of the dual quaternion
             */
            DualQuaternion(
                const Quaternion& p_realPart,
                const Quaternion& p_dualPart
            );

            /**
             * @brief Construction with AxisAngle for orientation and vector for position (first rotation then translation)
             * @param p_angleAxis defines the orientation of the dualq
             * @param p_dualPart defines the translation of the dualq
             */
            DualQuaternion(
                const AngleAxis& p_angleAxis,
                const Translation& p_translation
            );

            /**
             * @brief Construction with AxisAngle for orientation and vector for position (first translation then rotation)
             * @param p_translation defines the translation of the dualq
             * @param p_angleAxis defines the orientation of the dualq
             */
            DualQuaternion(
                const Translation& p_translation,
                const AngleAxis& p_angleAxis      
            );

            /**
             * @brief Construction with screw axis (rotation Axis & position)
             * @param p_rotationAxis defines the screw axis carried by the dualq
             * @param p_position defines a point on the screw axis carried by the dualq 
             */
            DualQuaternion(
                const Translation& p_rotationAxis,
                const Translation& p_position
            );

            /**
             * @brief Construction with Quaternion for orientation and vector for position, rotation then translation
             * @param p_orientation defines the orientation of the dualq
             * @param p_translation defines the translation of the dualq
             */
            DualQuaternion(
                const Quaternion& p_orientation,
                const Translation& p_translation
            );

            /**
             * @brief Construction with Quaternion for orientation and vector for position, translation then rotation
             * @param p_orientation defines the orientation of the dualq
             * @param p_translation defines the translation of the dualq
             */
            DualQuaternion(
                const Translation& p_translation,
                const Quaternion& p_orientation
            );

            /**
             * @brief Construction with Transformation Matrix
             * @param p_transform the transform carried by the dualq
             */
            DualQuaternion(
                const Transform& p_transform
            );

            /**
             * @brief Returns the dual quaternion multiplication of the object and another instance
             * @param p_other the dualq that multiplies the instance
             * @return the result of the multiplication
             */
            DualQuaternion operator*(const DualQuaternion& p_other) const;

            /**
             * @brief Returns the multiplication of the dual Quaternion with a scalar value
             * @param p_scalar the scalar that multiplies the instance
             * @return the result of the multiplication
             */
            DualQuaternion operator*(const Scalar p_scalar) const noexcept;

            /**
             * @brief Returns true if object and the other instance are the same
             * @param p_other the dualq to be compared to the instance
             * @param p_tolerance the tolerance of the comparison
             * @return True if comparison is within tolerance
             */
            bool isApprox(const DualQuaternion& p_other, const Scalar p_tolerance) const;

            void print() const;

            /**
             * @brief Returns the conjugate of the dual quaternion
             */
            DualQuaternion conjugate() const;

            /**
             * @brief  Returns true if dual quaternion is a unit one
             * @param p_tolerance tolerance used
             * @return True if instance is a unit dual quaternion
             */
            bool isUnit(const Scalar p_tolerance) const;

            /**
             * @brief Inverses the dual quaternion and returns true if success
             * @param p_tolerance tolerance used
             * @return True if inversion was a success 
             */
            bool invert(const Scalar p_tolerance);

            /**
             * @brief Returns the inverse of the dual quaternion if it exists, not optimized because of copies
             * @param p_tolerance tolerance used
             * @return The inverted dualq if inversion was a successs
             */
            std::optional<DualQuaternion> inverse(const Scalar p_tolerance) const;

            /**
             * @brief Returns real part of dual quaternion
             * @return The real part of the dualq
             */
            const Quaternion& getRealPart() const;

            /**
             * @brief Returns dual part of dual quaternion
             * @return The dual part of the dualq
             */
            const Quaternion& getDualPart() const;

            /**
             * @brief Returns transation part of dual quaternion
             */
            Translation getTranslation(bool p_rotationFirst) const;

            /**
             * @brief Returns rotation part of dual quaternion as a quaternion
             */
            Quaternion getRotation() const;

            /**
             * @brief Returnsrotation part of dual quaternion as a rotation matrix
             */
            RotationMatrix getRotationMatrix() const;

            /**
             * @brief Returns the transformation matrix represented by the dual quaternion (only works with rotation first)
             */
            Transform getTransform(bool p_rotationFirst=true) const;

            /**
             * @brief Returns the exponential of a quaternion
             */
            static Quaternion quaternionExp(const Eigen::Quaternion<Scalar>& p_quaternion);

            /**
             * @brief Returns the exponential of a dual quaternion
             */
            DualQuaternion dqExp() const;
            
        private:

            // Real Part of the dual quaternion
            Quaternion m_realPart;

            // Dual Part of the dual quaternion
            Quaternion m_dualPart;
            
            
    };
}

#include "dualq_kinematics/DualQuaternion.tpp"