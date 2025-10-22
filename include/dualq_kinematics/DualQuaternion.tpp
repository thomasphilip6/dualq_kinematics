//#include "dualq_kinematics/DualQuaternion.h"

namespace dualq_kinematics
{

//Construction by real and dual part
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Quaternion& p_realPart, const Quaternion& p_dualPart)
{
    m_realPart = p_realPart;
    m_dualPart = p_dualPart;
    m_isUnit = isUnit(c_tolerance);
}

//Construction by rotation (angle axis) and then translation
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const AngleAxis& p_angleAxis, const Translation& p_translation)
{
    m_realPart = Quaternion(p_angleAxis);
    m_dualPart = Quaternion(0.0, p_translation.x(), p_translation.y(), p_translation.z()) * m_realPart;
    m_dualPart.coeffs() = m_dualPart.coeffs() * 0.5;
    m_isUnit = isUnit(c_tolerance);//todo throw an error if m_isUnit is false
}

//Construction by translation then rotation (angle axis) 
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Translation& p_translation, const AngleAxis& p_angleAxis)
{
    m_realPart = Quaternion(p_angleAxis);
    m_dualPart = m_realPart * Quaternion(0.0, p_translation.x(), p_translation.y(), p_translation.z());
    m_dualPart.coeffs() = m_dualPart.coeffs() * 0.5;
    m_isUnit = isUnit(c_tolerance);//todo throw an error if m_isUnit is false
}

//Construction for screw axis
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Translation& p_rotationAxis, const Translation& p_position)
{
    //todo make sure screw axis is a unit vector
    m_realPart = Quaternion(0.0, p_rotationAxis.x(), p_rotationAxis.y(), p_rotationAxis.z());
    m_dualPart = Quaternion(
        0.0,
        p_position.y()*p_rotationAxis.z() - p_position.z()*p_rotationAxis.y(),
        p_position.z()*p_rotationAxis.x() - p_position.x()*p_rotationAxis.z(),
        p_position.x()*p_rotationAxis.y() - p_position.y()*p_rotationAxis.x()
    );
    m_isUnit = isUnit(c_tolerance);//todo throw an error if m_isUnit is false
}

//Construction by quaternion and vector for translation, rotation then translation
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Quaternion& p_orientation, const Translation& p_translation)
{
    m_realPart = p_orientation;
    m_dualPart = Quaternion(0.0, p_translation.x(), p_translation.y(), p_translation.z()) * m_realPart;
    m_dualPart.coeffs() = m_dualPart.coeffs() * 0.5;
    m_isUnit = isUnit(c_tolerance);//todo throw an error if m_isUnit is false
}

//Construction by quaternion and vector for translation, translation then rotation
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Translation& p_translation, const Quaternion& p_orientation)
{
    m_realPart = p_orientation;
    m_dualPart = m_realPart * Quaternion(0.0, p_translation.x(), p_translation.y(), p_translation.z());
    m_dualPart.coeffs() = m_dualPart.coeffs() * 0.5;
    m_isUnit = isUnit(c_tolerance);//todo throw an error if m_isUnit is false
}


//Construction by transformation matrix, assuming SRT, RT in this case: first rotation then translation
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Transform& p_transform)
{
    m_realPart = Quaternion(p_transform.rotation());
    m_dualPart = Quaternion(0.0, p_transform.translation().x(), p_transform.translation().y(), p_transform.translation().z()) * m_realPart;
    m_dualPart.coeffs() = m_dualPart.coeffs() * 0.5;
    m_isUnit = isUnit(c_tolerance);//todo throw an error if m_isUnit is false
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
DualQuaternion<Scalar> DualQuaternion<Scalar>::operator*(const Scalar p_scalar) const
{
    DualQuaternion l_dualqCopy = *this;
    l_dualqCopy.m_realPart.coeffs() = l_dualqCopy.m_realPart.coeffs() * p_scalar;
    l_dualqCopy.m_dualPart.coeffs() = l_dualqCopy.m_dualPart.coeffs() * p_scalar;
    return l_dualqCopy;
}

template<typename Scalar>
bool DualQuaternion<Scalar>::isApprox(const DualQuaternion& p_other, const Scalar p_tolerance) const
{
    return m_realPart.isApprox(p_other.m_realPart, p_tolerance) && m_dualPart.isApprox(p_other.m_dualPart, p_tolerance) ;
}

template<typename Scalar>
void DualQuaternion<Scalar>::print() const
{
    const std::array<Scalar, 8> l_allCoeff = {
        m_realPart.w(),m_realPart.x(),m_realPart.y(),m_realPart.z(),
        m_dualPart.w(),m_dualPart.x(),m_dualPart.y(),m_dualPart.z()
    };
    std::cout << "[ ";
    for (size_t i = 0; i < 8; i++)
    {
        std::cout << l_allCoeff.at(i);
        if(i != 7){
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
    
}

template<typename Scalar>
DualQuaternion<Scalar> DualQuaternion<Scalar>::conjugate() const
{
    return DualQuaternion<Scalar>(m_realPart.conjugate(),m_dualPart.conjugate());
}

template<typename Scalar>
bool DualQuaternion<Scalar>::isUnit(const Scalar p_tolerance) const
{
    const Scalar l_firstCondition = m_realPart.w()*m_realPart.w() + m_realPart.x()*m_realPart.x() + m_realPart.y()*m_realPart.y() + m_realPart.z()*m_realPart.z();
    const Scalar l_secondCondition = m_realPart.w()*m_dualPart.w() + m_realPart.x()*m_dualPart.x() + m_realPart.y()*m_dualPart.y() + m_realPart.z()*m_dualPart.z();

    bool l_firstOK = std::abs(1.0 - l_firstCondition) < p_tolerance;
    bool l_secondOK = std::abs(0.0 - l_secondCondition) < p_tolerance;

    return l_firstOK && l_secondOK;
}

template<typename Scalar>
bool DualQuaternion<Scalar>::invert(const Scalar p_tolerance)
{
    //check if real part is all zeroes
    if(m_realPart.isApprox(Quaternion(0.0,0.0,0.0,0.0),p_tolerance))
    {
        return false;
    }    
    if (m_isUnit==true)
    {
        m_realPart = this->conjugate().m_realPart;
        m_dualPart = this->conjugate().m_dualPart;
        return true;
    }
    else
    { 
        m_realPart = m_realPart.inverse();
        m_dualPart = m_realPart*m_dualPart*m_realPart;
        m_dualPart.w() = -m_dualPart.w();
        m_dualPart.x() = -m_dualPart.x();
        m_dualPart.y() = -m_dualPart.y();
        m_dualPart.z() = -m_dualPart.z();
        return true;
    }
}

template<typename Scalar>
std::optional<DualQuaternion<Scalar>> DualQuaternion<Scalar>::inverse(const Scalar p_tolerance) const
{
    DualQuaternion<Scalar> l_dualQuaternionCopy = *this;
    const bool l_result = l_dualQuaternionCopy.invert(p_tolerance);
    if (l_result)
    {
        return l_dualQuaternionCopy;
    }
    return std::nullopt;

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