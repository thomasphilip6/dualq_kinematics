namespace dualq_kinematics
{
    
template<typename Scalar>
FrankaKinSolver<Scalar>::FrankaKinSolver(const ScrewCoordinates& p_screwCoordinates)
{
    m_screwCoordinates = p_screwCoordinates;

    for (size_t i = 0; i < m_screwCoordinates.value().getJointsNames().size(); i++)
    {
        m_screwCoordinatesDualQ.push_back(DualQuaternion( m_screwCoordinates.value().getScrewAxes().at(i), m_screwCoordinates.value().getPositions().at(i) ));
    }

    //Check that all dual quaternions are unit dual quaternions
    // for (auto &&l_dq : m_screwCoordinatesDualQ)
    // {
    //     try
    //     {
    //         const bool l_isUnit = l_dq.isUnit(c_tolerance);
    //         if(!l_isUnit)
    //         {
    //             throw(l_isUnit);
    //         }
    //     }
    //     //todo change to a parameter to an error for better error handling
    //     catch(const bool& e)
    //     {
    //         //do something
    //     }
        
    // }

    m_tip2BaseInit = DualQuaternion(m_screwCoordinates.value().getTip2BaseInit());

}

template<typename Scalar>
FrankaKinSolver<Scalar>::FrankaKinSolver(const std::vector<Vector3> p_screwAxes, const std::vector<Vector3> p_positions, const Eigen::Isometry3d p_tip2BaseInit)
{
    m_screwAxes = p_screwAxes;
    m_positionsOnScrew = p_positions;
    for (size_t i = 0; i < m_screwAxes.size(); i++)
    {
        m_screwCoordinatesDualQ.push_back(DualQuaternion(m_screwAxes.at(i), m_positionsOnScrew.at(i)));
    }
    
    m_tip2BaseInit = DualQuaternion(p_tip2BaseInit);
    m_tip2BaseInitTf = p_tip2BaseInit;
}

template<typename Scalar>
void FrankaKinSolver<Scalar>::computeTipFK(std::vector<double>& p_jointValues_rad, Eigen::Isometry3d& p_tip2BaseComputed) const
{
    DualQuaternion l_tip2BaseDQComputed = (m_screwCoordinatesDualQ.at(0) * (p_jointValues_rad.at(0) * 0.5)).dqExp();
    for (size_t i = 1; i < p_jointValues_rad.size(); i++)
    {
        l_tip2BaseDQComputed = l_tip2BaseDQComputed * (m_screwCoordinatesDualQ.at(i)* (p_jointValues_rad.at(i) * 0.5)).dqExp();
    }
    l_tip2BaseDQComputed = l_tip2BaseDQComputed * m_tip2BaseInit.value();
    p_tip2BaseComputed = l_tip2BaseDQComputed.getTransform();    
}

template<typename Scalar>
void FrankaKinSolver<Scalar>::computeWristPosition(const Eigen::Isometry3d& p_tip2BaseWanted, const Scalar p_q7, Vector3& p_wrist) const
{
    Scalar l_a7;
    // r7 = eeWanted.translation - df * eeWanted.z()
    Vector3 l_r7;
    if(m_screwCoordinates.has_value())
    {
        l_a7 = m_screwCoordinates.value().getPositions().at(6)(0);
        l_r7 = p_tip2BaseWanted.translation() - (m_screwCoordinates.value().getPositions().at(6)(2) - m_screwCoordinates.value().getTip2BaseInit().translation().z())*p_tip2BaseWanted.rotation().col(2);
    }
    else
    {
        l_a7 = m_positionsOnScrew.at(6)(0);
        Scalar l_df = m_positionsOnScrew.at(6)(2) - m_tip2BaseInitTf.translation().z();
        l_r7 = p_tip2BaseWanted.translation() - l_df*p_tip2BaseWanted.rotation().col(2);
    }

    Quaternion l_eeY(0.0, p_tip2BaseWanted.rotation().col(1)(0), p_tip2BaseWanted.rotation().col(1)(1), p_tip2BaseWanted.rotation().col(1)(2));
    Quaternion l_s7(0.0, p_tip2BaseWanted.rotation().col(2)(0), p_tip2BaseWanted.rotation().col(2)(1), p_tip2BaseWanted.rotation().col(2)(2));
    Quaternion l_s6;
    if (p_q7 != 0.0)
    {
        l_s6 = DualQuaternion::quaternionExp(-p_q7*l_s7)*l_eeY;
        l_s6.normalize();
    }
    else
    {
        l_s6 = l_eeY;
    }
    Quaternion l_product = l_s6 * l_s7;
    p_wrist(0) = l_r7(0) - l_a7*(l_product.x());
    p_wrist(1) = l_r7(1) - l_a7*(l_product.y());
    p_wrist(2) = l_r7(2) - l_a7*(l_product.z());

}

//todo use the fact that e(-twist) = (e(twist))^-1
template<typename Scalar>
typename std::vector<std::vector<Scalar>> FrankaKinSolver<Scalar>::compute6DOFIK(const Eigen::Isometry3d& p_tip2BaseWanted, const Scalar p_q7) const
{
    std::vector<std::vector<Scalar>> l_solutions;

    // First compute right-end side l_g = l_eeWanted * l_ee0^-1 * l_g7^-1
    DualQuaternion l_g = DualQuaternion(p_tip2BaseWanted) * m_tip2BaseInit.inverse() * (m_screwCoordinatesDualQ.at(6)* (p_q7 * 0.5)).dqExp().inverse();
    
    // Inverse Kinematic chain to get rid of spherical joint
    l_g.invert();

    // ------------------- Solve for q4 ---------------- //
    const Quaternion l_shoulderQuat(0.0, m_screwCoordinates.value().getPositions().at(0)(0), m_screwCoordinates.value().getPositions().at(0)(1), m_screwCoordinates.value().getPositions().at(0)(2));
    Quaternion l_shoulderTransformed = l_g.getTransformedVector(l_shoulderQuat);
    
    Vector3 l_wrist;
    computeWristPosition(p_tip2BaseWanted, p_q7, l_wrist);
    const Quaternion l_wristQuat(0.0, l_wrist(0), l_wrist(1), l_wrist(2));
    const Scalar l_delta = (l_shoulderTransformed - l_wristQuat).norm();
    const ThirdPadenKahan l_q4ThirdPKProbem( 
        Quaternion(0.0, m_screwCoordinates.value().getPositions().at(3)(0), m_screwCoordinates.value().getPositions().at(3)(1), m_screwCoordinates.value().getPositions().at(3)(2)), 
        m_screwCoordinatesDualQ.at(3).getRealPart(), 
        l_shoulderQuat, 
        l_wristQuat, 
        l_delta
    );

    for (size_t i = 0; i < l_q4ThirdPKProbem.getResults(); i++)
    {
        std::vector<Scalar> l_solutionSet;
        l_solutionSet.resize(7);
        l_solutionSet.at(3) = l_q4ThirdPKProbem.getResults().at(i) * (-1); // * (-1) as kinematic chain was inverted
        l_solutions.push_back(l_solutionSet);
    }

    // ------------------- Solve for q5, q6 ---------------- //

    // Remember that l_shoulderQuatCopy  = l_g^-1 * l_shoulder

    for(size_t i = 0; i < l_q4ThirdPKProbem.getResults().size(); i++)
    {
        const SecondPadenKahan l_q5Q6SecondPKProblem(
            Quaternion(0.0, m_screwCoordinates.value().getPositions().at(5)(0), m_screwCoordinates.value().getPositions().at(5)(1), m_screwCoordinates.value().getPositions().at(5)(2)),
            m_screwCoordinatesDualQ.at(5).getRealPart(),
            m_screwCoordinatesDualQ.at(4).getRealPart(),
            ((m_screwCoordinatesDualQ.at(3)* (l_solutions.at(i).at(3) * -0.5)).dqExp()).getTransformedVector(l_shoulderQuat),
            l_shoulderTransformed
        );

        if(l_q5Q6SecondPKProblem.getAngle1Result().size() == 1)
        {
            //add to current solution set 
            l_solutions.at(i).at(5) = -l_q5Q6SecondPKProblem.getAngle1Result().at(0); // * (-1) as kinematic chain was inverted
            l_solutions.at(i).at(4) = -l_q5Q6SecondPKProblem.getAngle2Result().at(0); // * (-1) as kinematic chain was inverted
        }

        if(l_q5Q6SecondPKProblem.getAngle1Result().size() > 1)
        {
            for(size_t j = 1; j < l_q5Q6SecondPKProblem.getAngle1Result().size(); j++)
            {
                std::vector<Scalar> l_solutionSet = l_solutions.at(i);
                l_solutionSet.at(5) = -l_q5Q6SecondPKProblem.getAngle1Result().at(j); // * (-1) as kinematic chain was inverted
                l_solutionSet.at(6) = -l_q5Q6SecondPKProblem.getAngle2Result().at(j); // * (-1) as kinematic chain was inverted
                l_solutions.push_back(l_solutionSet); 
            }
        }
    }

    // ------------------- Solve for q2, q3 ---------------- //

    //todo something more elaborate to get this point
    const Quaternion l_pointOnFirstScrewOnly = (0.0, m_screwCoordinates.value().getPositions().at(0)(0), m_screwCoordinates.value().getPositions().at(0)(1), m_screwCoordinates.value().getPositions().at(0)(2) - 0.3);

    for(size_t i = 0; i < l_solutions.size(); i++)
    {
        const DualQuaternion l_rightHandSide = 
            ( ((m_screwCoordinatesDualQ.at(3)* (l_solutions.at(i).at(3) * -0.5)).dqExp()).invert() ) *
            ( ((m_screwCoordinatesDualQ.at(4)* (l_solutions.at(i).at(4) * -0.5)).dqExp()).invert() ) *
            ( ((m_screwCoordinatesDualQ.at(5)* (l_solutions.at(i).at(5) * -0.5)).dqExp()).invert() ) *
            l_g;

        const Quaternion l_endPoint = l_rightHandSide.getTransformedVector(l_pointOnFirstScrewOnly);

        const SecondPadenKahan l_q2Q3SecondPKProblem(
            Quaternion(0.0, m_screwCoordinates.value().getPositions().at(1)(0), m_screwCoordinates.value().getPositions().at(1)(1), m_screwCoordinates.value().getPositions().at(1)(2)),
            m_screwCoordinatesDualQ.at(2).getRealPart(),
            m_screwCoordinatesDualQ.at(1).getRealPart(),
            l_pointOnFirstScrewOnly,
            l_endPoint
        );

        if(l_q2Q3SecondPKProblem.getAngle1Result().size() == 1)
        {
            //add to current solution set 
            l_solutions.at(i).at(2) = -l_q2Q3SecondPKProblem.getAngle1Result().at(0); // * (-1) as kinematic chain was inverted
            l_solutions.at(i).at(1) = -l_q2Q3SecondPKProblem.getAngle2Result().at(0); // * (-1) as kinematic chain was inverted
        }

        if(l_q2Q3SecondPKProblem.getAngle1Result().size() > 1)
        {
            for(size_t j = 1; j < l_q2Q3SecondPKProblem.getAngle1Result().size(); j++)
            {
                std::vector<Scalar> l_solutionSet = l_solutions.at(i);
                l_solutionSet.at(2) = -l_q2Q3SecondPKProblem.getAngle1Result().at(j); // * (-1) as kinematic chain was inverted
                l_solutionSet.at(1) = -l_q2Q3SecondPKProblem.getAngle2Result().at(j); // * (-1) as kinematic chain was inverted
                l_solutions.push_back(l_solutionSet); 
            }
        }
    }

    // ------------------- Solve for q1 ---------------- //

    //todo something more elaborate to get this point
    const Quaternion l_pointNotOnFirstScrew = (0.0, m_screwCoordinates.value().getPositions().at(0)(0), m_screwCoordinates.value().getPositions().at(0)(1)- 0.3, m_screwCoordinates.value().getPositions().at(0)(2) );
    
    for (size_t i = 0; i < l_solutions.size(); i++)
    {
        const DualQuaternion l_rightHandSide = 
            ( ((m_screwCoordinatesDualQ.at(1)* (l_solutions.at(i).at(1) * -0.5)).dqExp()).invert() ) *
            ( ((m_screwCoordinatesDualQ.at(2)* (l_solutions.at(i).at(2) * -0.5)).dqExp()).invert() ) *
            ( ((m_screwCoordinatesDualQ.at(3)* (l_solutions.at(i).at(3) * -0.5)).dqExp()).invert() ) *
            ( ((m_screwCoordinatesDualQ.at(4)* (l_solutions.at(i).at(4) * -0.5)).dqExp()).invert() ) *
            ( ((m_screwCoordinatesDualQ.at(5)* (l_solutions.at(i).at(5) * -0.5)).dqExp()).invert() ) *
            l_g;
        
        const Quaternion l_endPoint = l_rightHandSide.getTransformedVector(l_pointNotOnFirstScrew);

        const FirstPadenKahan l_q1Problem(
            Quaternion(0.0, m_screwCoordinates.value().getPositions().at(0)(0), m_screwCoordinates.value().getPositions().at(0)(1), m_screwCoordinates.value().getPositions().at(0)(2)),
            m_screwCoordinatesDualQ.at(0).getRealPart(),
            l_pointNotOnFirstScrew,
            l_endPoint
        );

        if(l_q1Problem.getResult().has_value())
        {
            l_solutions.at(i).at(0) = l_q1Problem.getResult().value();
    
        }
    }

    return l_solutions; 

}

} // namespace dualq_kinematics