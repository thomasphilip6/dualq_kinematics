#include "DualQuaternion.h"

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
        m_dualPartOne.w()+m_dualPartTwo.w(),
        m_dualPartOne.x()+m_dualPartTwo.x(),
        m_dualPartOne.y()+m_dualPartTwo.y(),
        m_dualPartOne.z()+m_dualPartTwo.z(),
    );
    return DualQuaternion<Scalar>(l_realPart, l_dualPart);
}

template<typename Scalar>
const DualQuaternion<Scalar>::getRealPart() const
{
    return m_realPart;
}

template<typename Scalar>
const DualQuaternion<Scalar>::getDualPart() const
{
    return m_dualPart;
}

}