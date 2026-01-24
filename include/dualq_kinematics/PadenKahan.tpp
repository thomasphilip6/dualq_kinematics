namespace dualq_kinematics
{

template<typename Scalar>
FirstPadenKahanProblem<Scalar>::FirstPadenKahanProblem()
{

}

template<typename Scalar>
FirstPadenKahanProblem<Scalar>::FirstPadenKahanProblem(Quaternion& p_pointOnLine, Quaternion& p_axis, Quaternion& p_startPoint, Quaternion& p_endPoint)
{
    compute(p_pointOnLine, p_axis, p_startPoint, p_endPoint);
}

template<typename Scalar>
void FirstPadenKahanProblem<Scalar>::compute(Quaternion& p_pointOnLine, Quaternion& p_axis, Quaternion& p_startPoint, Quaternion& p_endPoint)
{
    const Quaternion l_x(0.0, p_startPoint.x()-p_pointOnLine.x(), p_startPoint.y()-p_pointOnLine.y(), p_startPoint.z()-p_pointOnLine.z());
    const Quaternion l_y(0.0, p_endPoint.x()-p_pointOnLine.x(), p_endPoint.y()-p_pointOnLine.y(), p_endPoint.z()-p_pointOnLine.z());

    const Quaternion l_xProjected(
        0.0,
        l_x.x() - DualQuaternion::quatMulScalarPart(p_axis, l_x)*p_axis.x(),
        l_x.y() - DualQuaternion::quatMulScalarPart(p_axis, l_x)*p_axis.y(),
        l_x.z() - DualQuaternion::quatMulScalarPart(p_axis, l_x)*p_axis.z()
    );

    const Quaternion l_yProjected(
        0.0,
        l_y.x() - DualQuaternion::quatMulScalarPart(p_axis, l_y)*p_axis.x(),
        l_y.y() - DualQuaternion::quatMulScalarPart(p_axis, l_y)*p_axis.y(),
        l_y.z() - DualQuaternion::quatMulScalarPart(p_axis, l_y)*p_axis.z()
    );

    //Checking the conditions for finite solution
    if(
        compareFloatNum(l_yProjected.norm(),l_xProjected.norm(), c_tolerance) && 
        !compareFloatNum(0.0, l_xProjected.norm(), c_tolerance) && 
        !(l_yProjected.isApprox(l_xProjected, c_tolerance) && 
        compareFloatNum(DualQuaternion::quatMulScalarPart(p_axis, l_y), DualQuaternion::quatMulScalarPart(p_axis, l_x), c_tolerance))
    )
    {
        m_resultAngle_rad = std::atan2(
            DualQuaternion::quatMulScalarPart(p_axis, l_xProjected*l_yProjected),
            DualQuaternion::quatMulScalarPart(l_xProjected, l_yProjected)
        );
    }  
}

template<typename Scalar>
const typename std::optional< Scalar >& FirstPadenKahanProblem<Scalar>::getResult() const
{
    return m_resultAngle_rad;
}

template<typename Scalar>
bool FirstPadenKahanProblem<Scalar>::compareFloatNum(Scalar p_a, Scalar p_b, Scalar p_tolerance)
{
    if (std::abs(p_a - p_b) < p_tolerance) 
    {
        return true;
    }
    else 
    {
        return false;
    }
}

// ------------------ Second Paden Kahan Subproblem --------------------------- //

template<typename Scalar>
SecondPadenKahanProblem<Scalar>::SecondPadenKahanProblem()
{

}

template<typename Scalar>
SecondPadenKahanProblem<Scalar>::SecondPadenKahanProblem(Quaternion& p_pointOnLines, Quaternion& p_axis1, Quaternion& p_axis2, Quaternion& p_startPoint, Quaternion& p_endPoint)
{
    compute(p_pointOnLines, p_axis1, p_axis2, p_startPoint, p_endPoint);
}

template<typename Scalar>
void SecondPadenKahanProblem<Scalar>::compute(Quaternion& p_pointOnLines, Quaternion& p_axis1, Quaternion& p_axis2, Quaternion& p_startPoint, Quaternion& p_endPoint)
{
    Quaternion l_x(0.0, p_startPoint.x()-p_pointOnLines.x(), p_startPoint.y()-p_pointOnLines.y(), p_startPoint.z()-p_pointOnLines.z());
    Quaternion l_y(0.0, p_endPoint.x()-p_pointOnLines.x(), p_endPoint.y()-p_pointOnLines.y(), p_endPoint.z()-p_pointOnLines.z());

    std::vector<Quaternion> l_intersections = computeIntersection(p_axis1, p_axis2, l_x, l_y);
    if (l_intersections.size() != 0)
    {

        for (size_t i=0; i < l_intersections.size(); i++)
        {
            Quaternion l_c(0.0, l_intersections.at(i).x() + p_pointOnLines.x(), l_intersections.at(i).y() + p_pointOnLines.y(), l_intersections.at(i).z() + p_pointOnLines.z());
            Quaternion l_axis1Minus(0.0, -p_axis1.x(), -p_axis1.y(), -p_axis1.z());

            m_secondRotations.push_back( dualq_kinematics::FirstPadenKahanProblem(p_pointOnLines, p_axis2, p_startPoint, l_c));//sigma 2
            m_firstRotations.push_back( dualq_kinematics::FirstPadenKahanProblem(p_pointOnLines, p_axis1, l_c, p_endPoint));//sigma 1

            if(m_firstRotations.at(i).getResult().has_value() && m_secondRotations.at(i).getResult().has_value())
            {
                m_resultsAngle1_rad.push_back(m_firstRotations.at(i).getResult().value());
                m_resultsAngle2_rad.push_back(m_secondRotations.at(i).getResult().value());
            }
        }
        
    }
}

template<typename Scalar>
const typename std::vector<Scalar>&  SecondPadenKahanProblem<Scalar>::getAngle1Result() const
{
    return m_resultsAngle1_rad;
}

template<typename Scalar>
const typename std::vector<Scalar>&  SecondPadenKahanProblem<Scalar>::getAngle2Result() const
{
    return m_resultsAngle2_rad;
}

template<typename Scalar>
typename std::vector<Eigen::Quaternion<Scalar>> SecondPadenKahanProblem<Scalar>::computeIntersection(Quaternion& p_axis1, Quaternion& p_axis2, Quaternion& p_x, Quaternion& p_y)
{
    std::vector<Quaternion> l_results;
    const Scalar l_l2XScalar = DualQuaternion::quatMulScalarPart(p_axis2, p_x);
    const Scalar l_l1YScalar = DualQuaternion::quatMulScalarPart(p_axis1, p_y);

    const Quaternion l_linesProduct = p_axis1 * p_axis2;
    const Scalar l_twoLinesScalar = -l_linesProduct.w();

    const Scalar l_alpha = 
        (l_twoLinesScalar * l_l2XScalar - l_l1YScalar) / ( std::pow(l_twoLinesScalar, 2) - 1);
    
    const Scalar l_beta = 
        (l_twoLinesScalar * l_l1YScalar - l_l2XScalar) / ( std::pow(l_twoLinesScalar, 2) - 1);
    
    const Scalar l_gammaSquared = 
        (p_x.squaredNorm() - std::pow(l_alpha, 2) - std::pow(l_beta, 2) -2*l_alpha*l_beta*l_twoLinesScalar) / (l_linesProduct.squaredNorm());

    Scalar l_gamma = std::sqrt(l_gammaSquared);
    if(l_gamma < 0)
    {
        return l_results;
    }

    const Quaternion l_z(
        0.0,
        l_alpha*p_axis1.x() + l_beta*p_axis2.x() + l_gamma*l_linesProduct.x(),
        l_alpha*p_axis1.y() + l_beta*p_axis2.y() + l_gamma*l_linesProduct.y(),
        l_alpha*p_axis1.z() + l_beta*p_axis2.z() + l_gamma*l_linesProduct.z()

    );
    l_results.push_back(l_z);

    if (l_gamma != 0)
    {
        l_gamma = - l_gamma;
        const Quaternion l_z2(
            0.0,
            l_alpha*p_axis1.x() + l_beta*p_axis2.x() + l_gamma*l_linesProduct.x(),
            l_alpha*p_axis1.y() + l_beta*p_axis2.y() + l_gamma*l_linesProduct.y(),
            l_alpha*p_axis1.z() + l_beta*p_axis2.z() + l_gamma*l_linesProduct.z()
    
        );
        l_results.push_back(l_z2);

    }

    return l_results;
}

template<typename Scalar>
Scalar SecondPadenKahanProblem<Scalar>::squaredNormOfQuatVectPart(Quaternion& p_quat)
{
    return std::pow(p_quat.x(), 2) + std::pow(p_quat.y(), 2) + std::pow(p_quat.z(), 2);
}

}