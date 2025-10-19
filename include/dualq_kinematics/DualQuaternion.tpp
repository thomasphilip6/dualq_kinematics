//#include "dualq_kinematics/DualQuaternion.h"

namespace dualq_kinematics
{

//Construction by real and dual part
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Quaternion& p_realPart, const Quaternion& p_dualPart)
{
    m_realPart = p_realPart;
    m_dualPart = p_dualPart;
    m_isUnit = isUnit(1e-6);
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
bool DualQuaternion<Scalar>::isApprox(const DualQuaternion& p_other, const Scalar p_tolerance) const
{
    return m_realPart.isApprox(p_other.m_realPart, p_tolerance) && m_dualPart.isApprox(p_other.m_dualPart, p_tolerance) ;
}

template<typename Scalar>
void DualQuaternion<Scalar>::print() const
{
    const std::array<Scalar, 8> l_allCoeff = {
        m_realPart.w(),m_realPart.x(),m_realPart.y(),m_realPart.z(),
        m_dualPart.w(),m_dualPart.x(),m_dualPart.y(),m_realPart.z()
    };
    std::cout << "Dual Quaternion is ";
    for (size_t i = 0; i < 8; i++)
    {
        std::cout << l_allCoeff.at(i) << ", ";
    }
    std::cout << " " << std::endl;
    
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
    // const std::array<Scalar, 4> l_realPartCoeff = {m_realPart.w(),m_realPart.x(),m_realPart.y(),m_realPart.z()};
    // uint8_t l_zeroCounter=0;
    // for (auto &&l_coeff : l_realPartCoeff)
    // {
    //     if(l_coeff==0.0)
    //     {
    //         ++l_zeroCounter;
    //     }
    // }
    // if(l_zeroCounter == 4){
    //     return false;
    // }
    
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