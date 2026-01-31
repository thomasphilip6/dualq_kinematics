namespace dualq_kinematics
{

//Construction by real and dual part
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Quaternion& p_realPart, const Quaternion& p_dualPart)
{
    m_realPart = p_realPart;
    m_dualPart = p_dualPart;
}

//Construction by rotation (angle axis) and then translation
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const AngleAxis& p_angleAxis, const Translation& p_translation)
{
    m_realPart = Quaternion(p_angleAxis);
    m_dualPart = Quaternion(0.0, p_translation.x(), p_translation.y(), p_translation.z()) * m_realPart;
    m_dualPart.coeffs() = m_dualPart.coeffs() * 0.5;
}

//Construction by translation then rotation (angle axis) 
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Translation& p_translation, const AngleAxis& p_angleAxis)
{
    m_realPart = Quaternion(p_angleAxis);
    m_dualPart = m_realPart * Quaternion(0.0, p_translation.x(), p_translation.y(), p_translation.z());
    m_dualPart.coeffs() = m_dualPart.coeffs() * 0.5;
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
}

//Construction by quaternion and vector for translation, rotation then translation
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Quaternion& p_orientation, const Translation& p_translation)
{
    m_realPart = p_orientation;
    m_dualPart = Quaternion(0.0, p_translation.x(), p_translation.y(), p_translation.z()) * m_realPart;
    m_dualPart.coeffs() = m_dualPart.coeffs() * 0.5;
}

//Construction by quaternion and vector for translation, translation then rotation
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Translation& p_translation, const Quaternion& p_orientation)
{
    m_realPart = p_orientation;
    m_dualPart = m_realPart * Quaternion(0.0, p_translation.x(), p_translation.y(), p_translation.z());
    m_dualPart.coeffs() = m_dualPart.coeffs() * 0.5;
}


//Construction by transformation matrix, assuming SRT, RT in this case: first rotation then translation
template<typename Scalar>
DualQuaternion<Scalar>::DualQuaternion(const Transform& p_transform)
{
    m_realPart = Quaternion(p_transform.rotation());
    m_dualPart = Quaternion(0.0, p_transform.translation().x(), p_transform.translation().y(), p_transform.translation().z()) * m_realPart;
    m_dualPart.coeffs() = m_dualPart.coeffs() * 0.5;
}



template<typename Scalar>
DualQuaternion<Scalar> DualQuaternion<Scalar>::operator*(const DualQuaternion<Scalar>& p_other) const
{
    return DualQuaternion<Scalar>(m_realPart * p_other.m_realPart, (m_realPart * p_other.m_dualPart) + (m_dualPart * p_other.m_realPart));
}

template<typename Scalar>
DualQuaternion<Scalar> DualQuaternion<Scalar>::operator*(const Scalar p_scalar) const noexcept
{
    Eigen::Matrix<Scalar,4,1> l_real = m_realPart.coeffs() * p_scalar;
    Eigen::Matrix<Scalar,4,1> l_dual = m_dualPart.coeffs() * p_scalar;

    return DualQuaternion<Scalar>(
        Eigen::Quaternion<Scalar>(l_real),
        Eigen::Quaternion<Scalar>(l_dual)
    );
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
DualQuaternion<Scalar> DualQuaternion<Scalar>::thirdConjugate() const
{
    return DualQuaternion<Scalar>(m_realPart.conjugate(), (-1.0)*m_dualPart.conjugate());
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
    if (isUnit(p_tolerance)==true)
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

template<typename Scalar>
typename DualQuaternion<Scalar>::Translation DualQuaternion<Scalar>::getTranslation(bool p_rotationFirst) const
{
    if(p_rotationFirst)
    {
        const Quaternion l_trans = m_dualPart*(m_realPart.conjugate());
        return Translation(l_trans.x()*2,l_trans.y()*2,l_trans.z()*2);
    }
    const Quaternion l_translation = m_dualPart.conjugate() * m_realPart;
    return Translation(-l_translation.x()*2, -l_translation.y()*2, -l_translation.z()*2);
}

template<typename Scalar>
typename DualQuaternion<Scalar>::Quaternion DualQuaternion<Scalar>::getRotation() const
{
    return m_realPart;
}

template<typename Scalar>
typename DualQuaternion<Scalar>::RotationMatrix DualQuaternion<Scalar>::getRotationMatrix() const
{
    return m_realPart.toRotationMatrix();
}

template<typename Scalar>
typename DualQuaternion<Scalar>::Transform DualQuaternion<Scalar>::getTransform(bool p_rotationFirst) const
{
    return getTranslation(p_rotationFirst) * Transform(getRotationMatrix());
}

template<typename Scalar>
typename DualQuaternion<Scalar>::Quaternion DualQuaternion<Scalar>::quaternionExp(const Eigen::Quaternion<Scalar>& p_quaternion)
{
    const Scalar l_alpha = p_quaternion.vec().norm();
    Scalar l_factor = 0;
    if (l_alpha > 0.001)
    {
        l_factor = sin(l_alpha) / l_alpha;
    }
    else 
    {
        //Avoiding division by zero by using Taylor Serie of (sin a) / a
        l_factor = 1.0 - pow(l_alpha, 2.0)/6.0 + pow(l_alpha, 4.0)/120.0;
    }
    return Eigen::Quaternion<Scalar>(exp(p_quaternion.w())*cos(l_alpha), p_quaternion.x()*l_factor, p_quaternion.y()*l_factor, p_quaternion.z()*l_factor);
}

template<typename Scalar>
Scalar DualQuaternion<Scalar>::quatMulScalarPart(const Eigen::Quaternion<Scalar>& p_quaternion1, const Eigen::Quaternion<Scalar>& p_quaternion2)
{
    return (p_quaternion1.x()*p_quaternion2.x() + p_quaternion1.y()*p_quaternion2.y() + p_quaternion1.z()*p_quaternion2.z());
}

template<typename Scalar>
typename DualQuaternion<Scalar>::DualQuaternion DualQuaternion<Scalar>::dqExp() const
{
    const Scalar l_psi = m_realPart.vec().norm();
    const Quaternion l_realPartExp = quaternionExp(m_realPart);
    const Scalar l_gamma = m_realPart.x()*m_dualPart.x() + m_realPart.y()*m_dualPart.y() + m_realPart.z()*m_dualPart.z();

    Scalar l_A = 0;
    Scalar l_B = 0;
    if (l_psi > 0.001)
    {
        l_A = sin(l_psi) / l_psi;
        l_B = (cos(l_psi) - l_A) / pow(l_psi, 2.0); 
    }
    else 
    {
        //Avoiding division by zero using Taylor Series
        l_A = 1.0 - pow(l_psi, 2.0)/6.0 + pow(l_psi, 4.0)/120.0;
        l_B =  -1.0/3.0 + pow(l_psi, 2.0)/30.0 - pow(l_psi, 4.0)/840.0; 
    }

    const Quaternion l_dualPartExp(
        exp(m_realPart.w())*(-l_A)*l_gamma + m_dualPart.w()*l_realPartExp.w(),
        exp(m_realPart.w())*(l_A*m_dualPart.x() + l_B*m_realPart.x()*l_gamma) + m_dualPart.w()*l_realPartExp.x(),
        exp(m_realPart.w())*(l_A*m_dualPart.y() + l_B*m_realPart.y()*l_gamma) + m_dualPart.w()*l_realPartExp.y(),
        exp(m_realPart.w())*(l_A*m_dualPart.z() + l_B*m_realPart.z()*l_gamma) + m_dualPart.w()*l_realPartExp.z()
    );
    return DualQuaternion(l_realPartExp, l_dualPartExp);
}

template<typename Scalar>
typename DualQuaternion<Scalar>::Quaternion DualQuaternion<Scalar>::getIntersectionOfLines(DualQuaternion& p_line2)
{
    //because p_line2.getRealPart().w() = 0
    Quaternion l_result = (p_line2.getRealPart() * p_line2.getDualPart()) + (p_line2.getRealPart() * quatMulScalarPart(this->getRealPart()*this->getDualPart(), p_line2.getRealPart()));
    l_result.w() = 0;
    return l_result;
}

template<typename Scalar>
void DualQuaternion<Scalar>::transformVector(Quaternion& p_vector) const
{
    p_vector = this->getRealPart() * p_vector * this->getRealPart().conjugate();
    auto l_translation = this->getTranslation(true);
    p_vector.x() = p_vector.x() + l_translation.x();
    p_vector.y() = p_vector.y() + l_translation.y();
    p_vector.z() = p_vector.z() + l_translation.z();
}

}