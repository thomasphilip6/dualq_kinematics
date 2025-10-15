//#include "dualq_kinematics/DualQuaternion.h"

namespace dualq_kinematics
{

//Construction by real and dual part
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Quaternion& p_realPart, const Quaternion& p_dualPart)
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
DualQuaternion<Scalar> DualQuaternion<Scalar>::conjugate() const
{
    return DualQuaternion<Scalar>(m_realPart.conjugate(),m_dualPart.conjugate());
}

template<typename Scalar>
bool DualQuaternion<Scalar>::isUnit(const Scalar p_tolerance) const
{
    DualQuaternion<Scalar> l_dualQuaternion = *this * this->conjugate();
    Quaternion l_realExpected(1.0, 0.0, 0.0, 0.0);
    Quaternion l_dualExpected(0.0, 0.0, 0.0, 0.0);

    bool l_realOK = (l_dualQuaternion.m_realPart.coeffs() - l_realExpected.coeffs()).norm() < p_tolerance;
    bool l_dualOK = (l_dualQuaternion.m_dualPart.coeffs() - l_dualExpected.coeffs()).norm() < p_tolerance;

    return l_realOK && l_dualOK;
}

template<typename Scalar>
void DualQuaternion<Scalar>::inverse()
{
    //todo deal with realPart = 0 then no inverse exist
    if (m_isUnit==true)
    {
        m_realPart = this->conjugate().m_realPart;
        m_dualPart = this->conjugate().m_dualPart;
    }
    else
    { 
        m_realPart = m_realPart.inverse();
        m_dualPart = m_realPart*m_dualPart*m_realPart;
        m_dualPart.w() = -m_dualPart.w();
        m_dualPart.x() = -m_dualPart.x();
        m_dualPart.y() = -m_dualPart.y();
        m_dualPart.z() = -m_dualPart.z();
    }
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