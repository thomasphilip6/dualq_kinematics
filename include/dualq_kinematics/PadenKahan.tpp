namespace dualq_kinematics
{

template<typename Scalar>
FirstPadenKahanProblem<Scalar>::FirstPadenKahanProblem()
{
    m_singularityFlag = false;
}

template<typename Scalar>
FirstPadenKahanProblem<Scalar>::FirstPadenKahanProblem(
    const Quaternion& p_pointOnLine, 
    const Quaternion& p_axis, 
    const Quaternion& p_startPoint, 
    const Quaternion& p_endPoint,
    const double& p_minValue_rad,
    const double& p_maxValue_rad
)
{
    m_singularityFlag = false;
    compute(p_pointOnLine, p_axis, p_startPoint, p_endPoint, p_minValue_rad, p_maxValue_rad);
}

template<typename Scalar>
inline void FirstPadenKahanProblem<Scalar>::compute(
    const Quaternion& p_pointOnLine, 
    const Quaternion& p_axis, 
    const Quaternion& p_startPoint, 
    const Quaternion& p_endPoint,
    const double& p_minValue_rad,
    const double& p_maxValue_rad
) noexcept
{
    const Quaternion l_x(0.0, p_startPoint.x()-p_pointOnLine.x(), p_startPoint.y()-p_pointOnLine.y(), p_startPoint.z()-p_pointOnLine.z());
    const Quaternion l_y(0.0, p_endPoint.x()-p_pointOnLine.x(), p_endPoint.y()-p_pointOnLine.y(), p_endPoint.z()-p_pointOnLine.z());

    const Scalar l_axisXScalarProduct =  DualQuaternion::quatMulScalarPart(p_axis, l_x);
    const Scalar l_axisYScalarProduct = DualQuaternion::quatMulScalarPart(p_axis, l_y);

    const Quaternion l_xProjected(
        0.0,
        l_x.x() - l_axisXScalarProduct*p_axis.x(),
        l_x.y() - l_axisXScalarProduct*p_axis.y(),
        l_x.z() - l_axisXScalarProduct*p_axis.z()
    );

    const Quaternion l_yProjected(
        0.0,
        l_y.x() - l_axisYScalarProduct*p_axis.x(),
        l_y.y() - l_axisYScalarProduct*p_axis.y(),
        l_y.z() - l_axisYScalarProduct*p_axis.z()
    );

    computeFromProjectedPoints(p_axis, l_xProjected, l_yProjected, p_minValue_rad, p_maxValue_rad);

}

template<typename Scalar>
inline void FirstPadenKahanProblem<Scalar>::compute(const Quaternion& p_pointOnLine, const Quaternion& p_axis, const Quaternion& p_startPoint, const Quaternion& p_endPoint, Scalar& p_result) noexcept
{
    const Quaternion l_x(0.0, p_startPoint.x()-p_pointOnLine.x(), p_startPoint.y()-p_pointOnLine.y(), p_startPoint.z()-p_pointOnLine.z());
    const Quaternion l_y(0.0, p_endPoint.x()-p_pointOnLine.x(), p_endPoint.y()-p_pointOnLine.y(), p_endPoint.z()-p_pointOnLine.z());

    const Scalar l_axisXScalarProduct =  DualQuaternion::quatMulScalarPart(p_axis, l_x);
    const Scalar l_axisYScalarProduct = DualQuaternion::quatMulScalarPart(p_axis, l_y);

    const Quaternion l_xProjected(
        0.0,
        l_x.x() - l_axisXScalarProduct*p_axis.x(),
        l_x.y() - l_axisXScalarProduct*p_axis.y(),
        l_x.z() - l_axisXScalarProduct*p_axis.z()
    );

    const Quaternion l_yProjected(
        0.0,
        l_y.x() - l_axisYScalarProduct*p_axis.x(),
        l_y.y() - l_axisYScalarProduct*p_axis.y(),
        l_y.z() - l_axisYScalarProduct*p_axis.z()
    );


    if(
        likely(compareFloatNum(l_yProjected.norm(),l_xProjected.norm(), c_tolerance) && 
        !compareFloatNum(0.0, l_xProjected.norm(), c_tolerance) && 
        !(l_yProjected.isApprox(l_xProjected, c_tolerance)))  
    )
    {
        p_result = std::atan2(
            DualQuaternion::quatMulScalarPart(p_axis, l_xProjected*l_yProjected),
            DualQuaternion::quatMulScalarPart(l_xProjected, l_yProjected)
        );
        return;
    }  

    p_result = std::nan("") ;
}

template<typename Scalar>
inline void FirstPadenKahanProblem<Scalar>::computeFromProjectedPoints(
    const Quaternion& p_axis, 
    const Quaternion& p_xProjected, 
    const Quaternion& p_yProjected,
    const double& p_minValue_rad,
    const double& p_maxValue_rad
) noexcept
{
    m_singularityFlag = false;
    //Checking the conditions for finite solution
    m_resultAngle_rad.reset();
    if(
        likely(compareFloatNum(p_yProjected.norm(),p_xProjected.norm(), c_tolerance) && 
        !compareFloatNum(0.0, p_xProjected.norm(), c_tolerance))  
    )
    {
        m_resultAngle_rad = std::atan2(
            DualQuaternion::quatMulScalarPart(p_axis, p_xProjected*p_yProjected),
            DualQuaternion::quatMulScalarPart(p_xProjected, p_yProjected)
        );
        if(m_resultAngle_rad > p_maxValue_rad || m_resultAngle_rad < p_minValue_rad)
        {
            m_resultAngle_rad.reset();
        }
    }  
    else 
    {
        //Singularity occured
        m_singularityFlag = true;   
    }
}

template<typename Scalar>
const typename std::optional< Scalar >& FirstPadenKahanProblem<Scalar>::getResult() const
{
    return m_resultAngle_rad;
}

template<typename Scalar>
const bool& FirstPadenKahanProblem<Scalar>::getSingularityStatus() const
{
    return m_singularityFlag;
}

template<typename Scalar>
bool FirstPadenKahanProblem<Scalar>::compareFloatNum(const Scalar p_a, const Scalar p_b, const Scalar p_tolerance) noexcept
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
    m_resultsAngle1_rad.reserve(2);
    m_resultsAngle2_rad.reserve(2);
    m_firstRotations.reserve(2);
    m_secondRotations.reserve(2);
}

template<typename Scalar>
SecondPadenKahanProblem<Scalar>::SecondPadenKahanProblem(
    const Quaternion& p_pointOnLines, 
    const Quaternion& p_axis1, 
    const Quaternion& p_axis2, 
    const Quaternion& p_startPoint, 
    const Quaternion& p_endPoint,
    const double& p_minValue1_rad,
    const double& p_maxValue1_rad,
    const double& p_minValue2_rad,
    const double& p_maxValue2_rad
)
{
    m_resultsAngle1_rad.reserve(2);
    m_resultsAngle2_rad.reserve(2);
    m_firstRotations.reserve(2);
    m_secondRotations.reserve(2);
    compute(p_pointOnLines, p_axis1, p_axis2, p_startPoint, p_endPoint, p_minValue1_rad, p_maxValue1_rad, p_minValue2_rad, p_maxValue2_rad);
}

template<typename Scalar>
bool SecondPadenKahanProblem<Scalar>::compute(
    const Quaternion& p_pointOnLines, 
    const Quaternion& p_axis1, 
    const Quaternion& p_axis2, 
    const Quaternion& p_startPoint, 
    const Quaternion& p_endPoint,
    const double& p_minValue1_rad,
    const double& p_maxValue1_rad,
    const double& p_minValue2_rad,
    const double& p_maxValue2_rad
) noexcept
{
    m_resultsAngle1_rad.clear();
    m_resultsAngle2_rad.clear();
    m_firstRotations.clear();
    m_secondRotations.clear();
    

    Quaternion l_x(0.0, p_startPoint.x()-p_pointOnLines.x(), p_startPoint.y()-p_pointOnLines.y(), p_startPoint.z()-p_pointOnLines.z());
    Quaternion l_y(0.0, p_endPoint.x()-p_pointOnLines.x(), p_endPoint.y()-p_pointOnLines.y(), p_endPoint.z()-p_pointOnLines.z());

    std::vector<Quaternion> l_intersections;
    l_intersections.reserve(2);
    computeIntersection(p_axis1, p_axis2, l_x, l_y, l_intersections);

    //If the two circles do not intersect computations fail and it is a sign for shoulder singularity
    if(l_intersections.size() == 0)
    {
        return false;
    }

    for (size_t i=0; i < l_intersections.size(); i++)
    {
        const Quaternion l_c(0.0, l_intersections.at(i).x() + p_pointOnLines.x(), l_intersections.at(i).y() + p_pointOnLines.y(), l_intersections.at(i).z() + p_pointOnLines.z());

        m_secondRotations.push_back( dualq_kinematics::FirstPadenKahanProblem(p_pointOnLines, p_axis2, p_startPoint, l_c, p_minValue2_rad, p_maxValue2_rad));//sigma 2
        m_firstRotations.push_back( dualq_kinematics::FirstPadenKahanProblem(p_pointOnLines, p_axis1, l_c, p_endPoint, p_minValue1_rad, p_maxValue1_rad));//sigma 1

        //Check if one of the problems encountered a singularity
        if(m_secondRotations.at(i).getSingularityStatus() || m_firstRotations.at(i).getSingularityStatus())
        {
            return false;
        }

        //if either of subproblems 1 could not be solved, the whole solution is invalid
        //likewise if out of bounds
        if(
            m_firstRotations.at(i).getResult().has_value() 
            && m_secondRotations.at(i).getResult().has_value()
        )
        {
            m_resultsAngle1_rad.push_back(m_firstRotations.at(i).getResult().value());
            m_resultsAngle2_rad.push_back(m_secondRotations.at(i).getResult().value());
        }
    }   
    
    return true;
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
inline void  SecondPadenKahanProblem<Scalar>::computeIntersection(const Quaternion& p_axis1, const Quaternion& p_axis2, Quaternion& p_x, Quaternion& p_y, std::vector<Eigen::Quaternion<Scalar>>& p_intersections) noexcept
{
    p_intersections.clear();
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

    if(l_gammaSquared < 0)
    {
        return;
    }

    Scalar l_gamma = std::sqrt(l_gammaSquared);

    const Quaternion l_z(
        0.0,
        l_alpha*p_axis1.x() + l_beta*p_axis2.x() + l_gamma*l_linesProduct.x(),
        l_alpha*p_axis1.y() + l_beta*p_axis2.y() + l_gamma*l_linesProduct.y(),
        l_alpha*p_axis1.z() + l_beta*p_axis2.z() + l_gamma*l_linesProduct.z()

    );
    p_intersections.push_back(l_z);

    if (!FirstPadenKahanProblem::compareFloatNum(0.0, l_gamma, c_tolerance))
    {
        l_gamma = - l_gamma;
        const Quaternion l_z2(
            0.0,
            l_alpha*p_axis1.x() + l_beta*p_axis2.x() + l_gamma*l_linesProduct.x(),
            l_alpha*p_axis1.y() + l_beta*p_axis2.y() + l_gamma*l_linesProduct.y(),
            l_alpha*p_axis1.z() + l_beta*p_axis2.z() + l_gamma*l_linesProduct.z()
    
        );
        p_intersections.push_back(l_z2);

    }

    return;
}

template<typename Scalar>
Scalar SecondPadenKahanProblem<Scalar>::squaredNormOfQuatVectPart(Quaternion& p_quat)
{
    return std::pow(p_quat.x(), 2) + std::pow(p_quat.y(), 2) + std::pow(p_quat.z(), 2);
}

// ------------------ Extended second Paden Kahan Subproblem --------------------------- //

template<typename Scalar>
SecondPadenKahanProblemExt<Scalar>::SecondPadenKahanProblemExt()
{

}

template<typename Scalar>
SecondPadenKahanProblemExt<Scalar>::SecondPadenKahanProblemExt(const Quaternion& p_pointOnLine1, const Quaternion& p_pointOnLine2,  const Quaternion& p_axis1, const Quaternion& p_axis2, const Quaternion& p_startPoint, const Quaternion& p_endPoint)
{
    compute(p_pointOnLine1, p_pointOnLine2, p_axis1, p_axis2, p_startPoint, p_endPoint);
}

template<typename Scalar>
void SecondPadenKahanProblemExt<Scalar>::compute(const Quaternion& p_pointOnLine1, const Quaternion& p_pointOnLine2, const Quaternion& p_axis1, const Quaternion& p_axis2, const Quaternion& p_startPoint, const Quaternion& p_endPoint)
{
    Quaternion l_x = p_startPoint - p_pointOnLine2;
    Quaternion l_y = p_endPoint - p_pointOnLine1;

    std::vector<Quaternion> l_z2;
    l_z2.reserve(2);
    SecondPadenKahanProblem<Scalar>::computeIntersection(p_axis1, p_axis2, l_x, l_y, l_z2);

    if (l_z2.size() != 0)
    {

        for (size_t i=0; i < l_z2.size(); i++)
        {
            Quaternion l_c(0.0, l_z2.at(i).x() + p_pointOnLine2.x(), l_z2.at(i).y() + p_pointOnLine2.y(), l_z2.at(i).z() + p_pointOnLine2.z());

            //todo change magic -M_PI +M_PI here
            m_secondRotations.push_back( dualq_kinematics::FirstPadenKahanProblem(p_pointOnLine2, p_axis2, p_startPoint, l_c, -M_PI, M_PI));//sigma 2
            m_firstRotations.push_back( dualq_kinematics::FirstPadenKahanProblem(p_pointOnLine1, p_axis1, l_c, p_endPoint, -M_PI, M_PI));//sigma 1

            if(m_firstRotations.at(i).getResult().has_value() && m_secondRotations.at(i).getResult().has_value())
            {
                m_resultsAngle1_rad.push_back(m_firstRotations.at(i).getResult().value());
                m_resultsAngle2_rad.push_back(m_secondRotations.at(i).getResult().value());
            }
        }
        
    }

}

template<typename Scalar>
const typename std::vector<Scalar>&  SecondPadenKahanProblemExt<Scalar>::getAngle1Result() const
{
    return m_resultsAngle1_rad;
}

template<typename Scalar>
const typename std::vector<Scalar>&  SecondPadenKahanProblemExt<Scalar>::getAngle2Result() const
{
    return m_resultsAngle2_rad;
}

// ------------------ Third Paden Kahan Subproblem --------------------------- //

template<typename Scalar>
ThirdPadenKahanProblem<Scalar>::ThirdPadenKahanProblem()
{
    m_results.reserve(2);
}

template<typename Scalar>
ThirdPadenKahanProblem<Scalar>::ThirdPadenKahanProblem(
    const Quaternion& p_pointOnLine, 
    const Quaternion& p_axis, 
    const Quaternion& p_startPoint, 
    const Quaternion& p_endPoint, 
    const Scalar p_distanceToEnd,
    const double& p_minValue_rad,
    const double& p_maxValue_rad
)
{
    m_results.reserve(2);
    compute(p_pointOnLine, p_axis, p_startPoint, p_endPoint, p_distanceToEnd, p_minValue_rad, p_maxValue_rad);
}

template<typename Scalar>
void ThirdPadenKahanProblem<Scalar>::compute(
    const Quaternion& p_pointOnLine, 
    const Quaternion& p_axis, 
    const Quaternion& p_startPoint, 
    const Quaternion& p_endPoint, 
    const Scalar p_distanceToEnd,
    const double& p_minValue_rad,
    const double& p_maxValue_rad
) noexcept
{
    m_results.clear();

    const Quaternion l_x(0.0, p_startPoint.x()-p_pointOnLine.x(), p_startPoint.y()-p_pointOnLine.y(), p_startPoint.z()-p_pointOnLine.z());
    const Quaternion l_y(0.0, p_endPoint.x()-p_pointOnLine.x(), p_endPoint.y()-p_pointOnLine.y(), p_endPoint.z()-p_pointOnLine.z());

    const Scalar l_axisXScalarProduct =  DualQuaternion::quatMulScalarPart(p_axis, l_x);
    const Scalar l_axisYScalarProduct = DualQuaternion::quatMulScalarPart(p_axis, l_y);

    const Quaternion l_xProjected(
        0.0,
        l_x.x() - l_axisXScalarProduct*p_axis.x(),
        l_x.y() - l_axisXScalarProduct*p_axis.y(),
        l_x.z() - l_axisXScalarProduct*p_axis.z()
    );

    const Quaternion l_yProjected(
        0.0,
        l_y.x() - l_axisYScalarProduct*p_axis.x(),
        l_y.y() - l_axisYScalarProduct*p_axis.y(),
        l_y.z() - l_axisYScalarProduct*p_axis.z()
    );

    const Quaternion l_startMinusEnd(0.0, p_startPoint.x()-p_endPoint.x(), p_startPoint.y()-p_endPoint.y(), p_startPoint.z()-p_endPoint.z());
    const Scalar l_deltaProjSquared = std::pow(p_distanceToEnd, 2) - std::pow( DualQuaternion::quatMulScalarPart(p_axis, l_startMinusEnd), 2);

    
    Scalar l_theta0 = std::atan2(
        DualQuaternion::quatMulScalarPart(p_axis, l_xProjected*l_yProjected),
        DualQuaternion::quatMulScalarPart(l_xProjected, l_yProjected)
    );

    const Scalar l_cosineLaw = (l_xProjected.squaredNorm() + l_yProjected.squaredNorm() - l_deltaProjSquared ) / (2*l_xProjected.norm()*l_yProjected.norm());
    if(unlikely(l_cosineLaw >1 || l_cosineLaw < -1))
    {
        //It means that there is no solution
        return;
    }

    const Scalar l_theta1 = std::acos(l_cosineLaw);

    if(l_theta1 != 0)
    {
        m_results.push_back( l_theta0 + l_theta1);
        m_results.push_back( l_theta0 - l_theta1);
    }
    else
    {
        m_results.push_back(l_theta0);
    }

    //ensure result is primary value
    for (auto &&l_angle : m_results)
    {
        if(l_angle > M_PI )
        {
            l_angle = 2*M_PI - l_angle;
        }
        else if(l_angle < -M_PI)
        {
            l_angle = 2*M_PI + l_angle;
        }
    }

    //Check if solution is in bound
    for (auto it = m_results.begin(); it != m_results.end();)
    {
        if ((*it) > p_maxValue_rad || (*it) < p_minValue_rad)
            it = m_results.erase(it);
        else
            ++it;
    }
    
}

template<typename Scalar>
const typename std::vector<Scalar>& ThirdPadenKahanProblem<Scalar>::getResults() const
{
    return m_results;
}

}