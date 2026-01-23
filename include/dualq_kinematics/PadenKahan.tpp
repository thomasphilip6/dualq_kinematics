namespace dualq_kinematics
{

template<typename Scalar>
FirstPadenKahanProblem<Scalar>::FirstPadenKahanProblem(Vector3& p_pointOnLine, Quaternion& p_axis, Vector3& p_startPoint, Vector3& p_endPoint)
{
    const Quaternion l_x(0.0, p_startPoint(0)-p_pointOnLine(0), p_startPoint(1)-p_pointOnLine(1), p_startPoint(2)-p_pointOnLine(2));
    const Quaternion l_y(0.0, p_endPoint(0)-p_pointOnLine(0), p_endPoint(1)-p_pointOnLine(1), p_endPoint(2)-p_pointOnLine(2));

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
    if(compareFloatNum(l_yProjected.norm(),l_xProjected.norm(), c_tolerance) && !compareFloatNum(0.0, l_xProjected.norm(), c_tolerance) && !(l_yProjected.isApprox(l_xProjected, c_tolerance)))
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

template<typename Scalar>
SecondPadenKahanProblem<Scalar>::SecondPadenKahanProblem(Vector3& p_pointOnLines, Quaternion& p_axis1, Quaternion& p_axis2, Vector3& p_startPoint, Vector3& p_endPoint)
{
    m_firstRotations.emplace();
    m_secondRotations.emplace();

    Quaternion l_x(0.0, p_startPoint(0)-p_pointOnLines(0), p_startPoint(1)-p_pointOnLines(1), p_startPoint(2)-p_pointOnLines(2));
    Quaternion l_y(0.0, p_endPoint(0)-p_pointOnLines(0), p_endPoint(1)-p_pointOnLines(1), p_endPoint(2)-p_pointOnLines(2));

    std::vector<Quaternion> l_intersections = computeIntersection(p_axis1, p_axis2, l_x, l_y);
    if (l_intersections.size() != 0)
    {

        for (size_t i=0; i < l_intersections.size(); i++)
        {
            //todo think about keepind l_intersection as a Quaternion
            Vector3 l_intersectionVector;
            l_intersectionVector << l_intersections.at(i).x() , l_intersections.at(i).y(), l_intersections.at(i).z();
            Quaternion l_axis1Minus(0.0, -p_axis1.x(), -p_axis1.y(), -p_axis1.z());
            m_firstRotations.value().push_back(dualq_kinematics::FirstPadenKahanProblem(p_pointOnLines, p_axis2, p_startPoint, l_intersectionVector));
            m_secondRotations.value().push_back(dualq_kinematics::FirstPadenKahanProblem(p_pointOnLines, l_axis1Minus, p_endPoint, l_intersectionVector));

            if(m_firstRotations.value().at(i).getResult().has_value() && m_secondRotations.value().at(i).getResult().has_value())
            {
                m_resultsAngle1_rad.push_back(m_firstRotations.value().at(i).getResult().value());
                m_resultsAngle2_rad.push_back(m_secondRotations.value().at(i).getResult().value());
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
    const Scalar l_l1XScalar = DualQuaternion::quatMulScalarPart(p_axis1, p_x);
    const Scalar l_l2YScalar = DualQuaternion::quatMulScalarPart(p_axis2, p_y);
    const Scalar l_l1YScalar = DualQuaternion::quatMulScalarPart(p_axis1, p_y);

    const Quaternion l_linesProduct = p_axis1 * p_axis2;
    const Scalar l_twoLinesScalar = l_linesProduct.x();

    const Scalar l_alpha = 
        (l_twoLinesScalar * l_l2XScalar - l_l1XScalar) / ( std::pow(l_twoLinesScalar, 2) - 1);
    
    const Scalar l_beta = 
        (l_twoLinesScalar * l_l1YScalar - l_l2YScalar) / ( std::pow(l_twoLinesScalar, 2) - 1);
    
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