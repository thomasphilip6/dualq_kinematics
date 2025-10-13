//#include "dualq_kinematics/DualQuaternion.h"

namespace dualq_kinematics
{

template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(Quaternion& p_realPart, Quaternion& p_dualPart)
{
    m_realPart = p_realPart;
    m_dualPart = p_dualPart;
}

template<typename Scalar>
DualQuaternion<Scalar> DualQuaternion<Scalar>::operator*(const DualQuaternion<Scalar>& p_other) const
{
    const Quaternion l_realPart(m_realPart * p_other.m_realPart);
    const Quaternion l_dualPartOne(m_realPart * p_other.m_dualPart);
    const Quaternion l_dualPartTwo(m_dualPart * p_other.m_realPart);
    const Quaternion l_dualPart(
        l_dualPartOne.w()+l_dualPartTwo.w(),
        l_dualPartOne.x()+l_dualPartTwo.x(),
        l_dualPartOne.y()+l_dualPartTwo.y(),
        l_dualPartOne.z()+l_dualPartTwo.z()
    );
    return DualQuaternion<Scalar>(l_realPart, l_dualPart);
}

template<typename Scalar>
const typename DualQuaternion<Scalar>::Quaternion& DualQuaternion<Scalar>::getRealPart() const
{
    return m_realPart;
}

template<typename Scalar>
const typename DualQuaternion<Scalar>::Quaternion& DualQuaternion<Scalar>::getDualPart() const
{
    return m_dualPart;
}

}