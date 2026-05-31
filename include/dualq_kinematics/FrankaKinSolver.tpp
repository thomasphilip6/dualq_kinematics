namespace dualq_kinematics
{
    
template<typename Scalar>
FrankaKinSolver<Scalar>::FrankaKinSolver(const ScrewCoordinates* p_screwCoordinatesPtr)
{
    m_screwCoordinatesPtr = std::make_shared<ScrewCoordinates>(*p_screwCoordinatesPtr);

    for (size_t i = 0; i < m_screwCoordinatesPtr->getJointsNames().size(); i++)
    {
        m_screwCoordinatesDualQ.push_back(DualQuaternion( m_screwCoordinatesPtr->getScrewAxes().at(i), m_screwCoordinatesPtr->getPositions().at(i) ));
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

    m_tip2BaseInitPtr = std::make_shared<DualQuaternion>(DualQuaternion(m_screwCoordinatesPtr->getTip2BaseInit()));
    m_emergencyQ1_rad = M_PI / 2.0;
    m_emergencyQ5_rad = 0.0; 
}

template<typename Scalar>
bool FrankaKinSolver<Scalar>::setEmergencyQ1(const Scalar& p_q1Value_rad)
{
    if(
        p_q1Value_rad < m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(0)).max_position_ && 
        p_q1Value_rad > m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(0)).min_position_ 
    )
    {
        m_emergencyQ1_rad = p_q1Value_rad;
        return true;
    }
    return false;
}

template<typename Scalar>
bool FrankaKinSolver<Scalar>::setEmergencyQ5(const Scalar& p_q5Value_rad)
{
    if(
        p_q5Value_rad < m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(4)).max_position_ && 
        p_q5Value_rad > m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(4)).min_position_ 
    )
    {
        m_emergencyQ5_rad = p_q5Value_rad;
        return true;
    }
    return false;
}

template<typename Scalar>
void FrankaKinSolver<Scalar>::computeTipFK(const std::vector<double>& p_jointValues_rad, Eigen::Isometry3d& p_tip2BaseComputed) const noexcept
{
    DualQuaternion l_tip2BaseDQComputed = (m_screwCoordinatesDualQ.at(0) * (p_jointValues_rad.at(0) * 0.5)).dqExp();
    for (size_t i = 1; i < p_jointValues_rad.size(); i++)
    {
        l_tip2BaseDQComputed = l_tip2BaseDQComputed * (m_screwCoordinatesDualQ.at(i)* (p_jointValues_rad.at(i) * 0.5)).dqExp();
    }
    l_tip2BaseDQComputed = l_tip2BaseDQComputed * *m_tip2BaseInitPtr;
    p_tip2BaseComputed = l_tip2BaseDQComputed.getTransform();    
}


template<typename Scalar>
void FrankaKinSolver<Scalar>::computeElbowPosition(const Eigen::Isometry3d& p_tip2BaseWanted, const Scalar& p_q5, const Scalar& p_q6, const Scalar& p_q7, Quaternion& p_elbow) const noexcept
{

    // r7 = eeWanted.translation - df * eeWanted.z()
    Vector3 l_r7;
    Vector3 l_wrist;

    const Scalar l_a7 = m_screwCoordinatesPtr->getPositions().at(6)(0);
    const Scalar l_a5 = m_screwCoordinatesPtr->getPositions().at(3)(0);

    //l_d5 is the distance in z (robot at rest) between the elbow and the wrist
    const Scalar l_d5 = m_screwCoordinatesPtr->getPositions().at(5)(2) -  m_screwCoordinatesPtr->getPositions().at(3)(2);
    l_r7 = p_tip2BaseWanted.translation() - (m_screwCoordinatesPtr->getPositions().at(6)(2) - m_screwCoordinatesPtr->getTip2BaseInit().translation().z())*p_tip2BaseWanted.rotation().col(2);

    const Quaternion l_eeY(0.0, p_tip2BaseWanted.rotation().col(1)(0), p_tip2BaseWanted.rotation().col(1)(1), p_tip2BaseWanted.rotation().col(1)(2));
    const Quaternion l_s7(0.0, p_tip2BaseWanted.rotation().col(2)(0), p_tip2BaseWanted.rotation().col(2)(1), p_tip2BaseWanted.rotation().col(2)(2));
    Quaternion l_s6;
    
    l_s6 = DualQuaternion::quaternionExp(-p_q7*l_s7)*l_eeY;
    l_s6.normalize();

    //Vector product as product of two pure quaternion returns vector product for the vector part of the quaternion
    Quaternion l_vectorProduct = l_s6 * l_s7;
    l_wrist(0) = l_r7(0) - l_a7*(l_vectorProduct.x());
    l_wrist(1) = l_r7(1) - l_a7*(l_vectorProduct.y());
    l_wrist(2) = l_r7(2) - l_a7*(l_vectorProduct.z());

    Quaternion l_s5 = DualQuaternion::quaternionExp(-p_q6*l_s6)*l_s7;
    l_s5 = -1.0 * l_s5; // as s5 and s7 are (0,0,-1) and (0,0,1) at rest
    l_s5.normalize();

    Quaternion l_s4 = DualQuaternion::quaternionExp(-p_q5*l_s5)*l_s6;
    l_s4.normalize();

    l_vectorProduct = l_s4 * l_s5;
    Quaternion l_r4;
    l_r4.w() = 0.0;
    l_r4.x() = l_wrist(0) - l_d5 * l_s5.x() - l_a5 * l_vectorProduct.x(); 
    l_r4.y() = l_wrist(1) - l_d5 * l_s5.y() - l_a5 * l_vectorProduct.y(); 
    l_r4.z() = l_wrist(2) - l_d5 * l_s5.z() - l_a5 * l_vectorProduct.z(); 

    p_elbow = l_r4;
}

template<typename Scalar>
void FrankaKinSolver<Scalar>::computeWristPosition(const Eigen::Isometry3d& p_tip2BaseWanted, const Scalar& p_q7, Vector3& p_wrist) const noexcept
{
    // r7 = eeWanted.translation - df * eeWanted.z()

    const Scalar l_a7 = m_screwCoordinatesPtr->getPositions().at(6)(0);
    const Vector3 l_r7 = p_tip2BaseWanted.translation() - (m_screwCoordinatesPtr->getPositions().at(6)(2) - m_screwCoordinatesPtr->getTip2BaseInit().translation().z())*p_tip2BaseWanted.rotation().col(2);

    const Quaternion l_eeY(0.0, p_tip2BaseWanted.rotation().col(1)(0), p_tip2BaseWanted.rotation().col(1)(1), p_tip2BaseWanted.rotation().col(1)(2));
    const Quaternion l_s7(0.0, p_tip2BaseWanted.rotation().col(2)(0), p_tip2BaseWanted.rotation().col(2)(1), p_tip2BaseWanted.rotation().col(2)(2));
 
    Quaternion l_s6 = DualQuaternion::quaternionExp(-p_q7*l_s7)*l_eeY;
    l_s6.normalize();
    
    const Quaternion l_product = l_s6 * l_s7;
    p_wrist(0) = l_r7(0) - l_a7*(l_product.x());
    p_wrist(1) = l_r7(1) - l_a7*(l_product.y());
    p_wrist(2) = l_r7(2) - l_a7*(l_product.z());

}

template<typename Scalar>
void FrankaKinSolver<Scalar>::compute6DOFIK(const Eigen::Isometry3d& p_tip2BaseWanted, const Scalar& p_q7, std::vector<std::vector<Scalar>>& p_solutions) const noexcept
{
    p_solutions.clear();
    // First compute right-end side l_g = l_eeWanted * l_ee0^-1 * l_g7^-1
    //todo add error management (dualq inverse, paden kahan problems not returning)
    DualQuaternion l_g = DualQuaternion(p_tip2BaseWanted) * m_tip2BaseInitPtr->inverse(c_tolerance).value() * ((m_screwCoordinatesDualQ.at(6)* (-p_q7 * 0.5)).dqExp());

    // Inverse Kinematic chain to get rid of spherical joint
    l_g.invert(c_tolerance);

    // For q2q3 solve later
    const Quaternion l_pointOnFirstScrewOnly = 
        Quaternion(0.0, m_screwCoordinatesPtr->getPositions().at(0)(0), m_screwCoordinatesPtr->getPositions().at(0)(1), m_screwCoordinatesPtr->getPositions().at(0)(2)) - 0.3*m_screwCoordinatesDualQ.at(0).getRealPart();

    Quaternion l_pointOnLinesQ2Q3(0.0, m_screwCoordinatesPtr->getPositions().at(1)(0), m_screwCoordinatesPtr->getPositions().at(1)(1), m_screwCoordinatesPtr->getPositions().at(1)(2));
     
    //For q1 later
    const Quaternion l_pointNotOnFirstScrew = 
        Quaternion(0.0, m_screwCoordinatesPtr->getPositions().at(0)(0), m_screwCoordinatesPtr->getPositions().at(0)(1), m_screwCoordinatesPtr->getPositions().at(0)(2)) - 0.3*m_screwCoordinatesDualQ.at(1).getRealPart();
    
    const Quaternion l_pointOnLineQ1(0.0, m_screwCoordinatesPtr->getPositions().at(0)(0), m_screwCoordinatesPtr->getPositions().at(0)(1), m_screwCoordinatesPtr->getPositions().at(0)(2));    
    const Quaternion l_pointOnLinesQ5Q6(0.0, m_screwCoordinatesPtr->getPositions().at(5)(0), m_screwCoordinatesPtr->getPositions().at(5)(1), m_screwCoordinatesPtr->getPositions().at(5)(2));

    // ------------------- Solve for q4 ---------------- //
    const Quaternion l_shoulderQuat(0.0, m_screwCoordinatesPtr->getPositions().at(0)(0), m_screwCoordinatesPtr->getPositions().at(0)(1), m_screwCoordinatesPtr->getPositions().at(0)(2));
    Quaternion l_shoulderTransformed = l_g.getTransformedVector(l_shoulderQuat);
    
    Vector3 l_wrist;
    computeWristPosition(p_tip2BaseWanted, p_q7, l_wrist);
    const Quaternion l_wristQuat(0.0, l_wrist(0), l_wrist(1), l_wrist(2));
    Quaternion l_wristInTipFrame = l_g.getTransformedVector(l_wristQuat);

    const Scalar l_delta = (l_shoulderTransformed - l_wristInTipFrame).norm();
    Quaternion l_pointOnlineQ4(0.0, m_screwCoordinatesPtr->getPositions().at(3)(0), m_screwCoordinatesPtr->getPositions().at(3)(1), m_screwCoordinatesPtr->getPositions().at(3)(2));
    const ThirdPadenKahan l_q4ThirdPKProbem(
        l_pointOnlineQ4,
        m_screwCoordinatesDualQ.at(3).getRealPart(), 
        l_shoulderQuat, 
        l_wristInTipFrame, 
        l_delta,
        -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(3)).max_position_,
        -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(3)).min_position_
    );

    SecondPadenKahan l_q5Q6SecondPKProblem;

    for (size_t i = 0; i < l_q4ThirdPKProbem.getResults().size(); i++)
    {
        //Shoulder Sing : l_shoulderSingularityFree if false means that the subproblems causes l_pointOnFirstScrewOnly to be both on screw 1 & 3, in this case the usual cancelation of screw 1 doesn't work
        //Elbow Sing : l_elbowSingularityFree if false means that the subproblems causes screw 5 to be aligned with spherical shoulder, then the cancelation of screws to get q5 and q6 doesn't work
        bool l_elbowSingularityFree;
        std::vector<Scalar> l_solutionSet;
        l_solutionSet.resize(7);
        l_solutionSet.at(3) = l_q4ThirdPKProbem.getResults().at(i) * (-1); // * (-1) as kinematic chain was inverted
        
        const DualQuaternion l_g4 = ((m_screwCoordinatesDualQ.at(3)* (l_solutionSet.at(3) * 0.5)).dqExp());

        // ------------------- Solve for q5, q6 ---------------- //

        l_elbowSingularityFree = l_q5Q6SecondPKProblem.compute(
            l_pointOnLinesQ5Q6,
            m_screwCoordinatesDualQ.at(5).getRealPart(),
            m_screwCoordinatesDualQ.at(4).getRealPart(),
            l_g4.inverse(c_tolerance).value().getTransformedVector(l_shoulderQuat),
            l_shoulderTransformed,
            -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(5)).max_position_,
            -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(5)).min_position_,
            -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(4)).max_position_,
            -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(4)).min_position_
        );

        size_t l_q5Q6SecondPKSolNb = l_q5Q6SecondPKProblem.getAngle1Result().size();
        FirstPadenKahan l_q6Problem;

        if(!l_elbowSingularityFree)
        {
            //q1 or q3 will compensate for this
            l_solutionSet.at(4) = m_emergencyQ5_rad;
            const DualQuaternion l_g5Inverted = ((m_screwCoordinatesDualQ.at(3)* (l_solutionSet.at(3) * 0.5)).dqExp());

            l_q6Problem.compute(
                l_pointOnLinesQ5Q6,
                m_screwCoordinatesDualQ.at(5).getRealPart(),
                (l_g5Inverted * (l_g4).inverse(c_tolerance).value()).getTransformedVector(l_shoulderQuat),
                l_shoulderTransformed,
                -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(5)).max_position_,
                -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(5)).min_position_
            );

            if(l_q6Problem.getResult().has_value())
            {
                l_q5Q6SecondPKSolNb = 1;
            }
            else
            {
                l_q5Q6SecondPKSolNb = 0;
            }

        }

        SecondPadenKahan l_q2Q3SecondPKProblem;
        
        for(size_t j = 0; j < l_q5Q6SecondPKSolNb; j++)
        {
            //Singularity is detected if during the next subproblem the circles don't intersect (or they do because of tiny floating point values, then it is caught in the PK1 because the projection of the point is too close to zero)
            bool l_shoulderSingularityFree;
            
            if(l_elbowSingularityFree)
            {
                l_solutionSet.at(5) = -l_q5Q6SecondPKProblem.getAngle1Result().at(j); // * (-1) as kinematic chain was inverted
                l_solutionSet.at(4) = -l_q5Q6SecondPKProblem.getAngle2Result().at(j); // * (-1) as kinematic chain was inverted
            }
            else
            {
                l_solutionSet.at(5) = -l_q6Problem.getResult().value(); // * (-1) as kinematic chain was inverted
            }

            const DualQuaternion l_g5 = ((m_screwCoordinatesDualQ.at(4)* (l_solutionSet.at(4) * 0.5)).dqExp());
            const DualQuaternion l_g6 = ((m_screwCoordinatesDualQ.at(5)* (l_solutionSet.at(5) * 0.5)).dqExp());

            // ------------------- Solve for q2, q3 ---------------- //

            DualQuaternion l_rightHandSide = l_g4 * l_g5 * l_g6 * l_g;

            Quaternion l_endPoint = l_rightHandSide.getTransformedVector(l_pointOnFirstScrewOnly);
            
            l_shoulderSingularityFree = l_q2Q3SecondPKProblem.compute(
                l_pointOnLinesQ2Q3,
                m_screwCoordinatesDualQ.at(2).getRealPart(),
                m_screwCoordinatesDualQ.at(1).getRealPart(),
                l_pointOnFirstScrewOnly,
                l_endPoint,
                -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(2)).max_position_,
                -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(2)).min_position_,
                -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(1)).max_position_,
                -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(1)).min_position_
            );

            if(!l_shoulderSingularityFree)
            {
                //Fix q1 at its emergency value and q3 will compensate for it in the following PK2 problem
                l_solutionSet.at(0) = m_emergencyQ1_rad;

                const DualQuaternion l_g1Inverted = (m_screwCoordinatesDualQ.at(0)* (-l_solutionSet.at(0) * 0.5)).dqExp();
                l_endPoint = l_rightHandSide.getTransformedVector(l_pointNotOnFirstScrew);

                l_q2Q3SecondPKProblem.compute(
                    l_pointOnLinesQ2Q3,
                    m_screwCoordinatesDualQ.at(2).getRealPart(),
                    m_screwCoordinatesDualQ.at(1).getRealPart(),
                    l_g1Inverted.getTransformedVector(l_pointNotOnFirstScrew),
                    l_endPoint,
                    -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(2)).max_position_,
                    -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(2)).min_position_,
                    -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(1)).max_position_,
                    -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(1)).min_position_
                );

                for(size_t k = 0; k < l_q2Q3SecondPKProblem.getAngle1Result().size(); k++)
                {
                
                    l_solutionSet.at(2) = -l_q2Q3SecondPKProblem.getAngle1Result().at(k); // * (-1) as kinematic chain was inverted
                    l_solutionSet.at(1) = -l_q2Q3SecondPKProblem.getAngle2Result().at(k); // * (-1) as kinematic chain was inverted
                    p_solutions.push_back(l_solutionSet);
                }
                
                continue;
            }

            FirstPadenKahan l_q1Problem;
            for(size_t k = 0; k < l_q2Q3SecondPKProblem.getAngle1Result().size(); k++)
            {
            
                l_solutionSet.at(2) = -l_q2Q3SecondPKProblem.getAngle1Result().at(k); // * (-1) as kinematic chain was inverted
                l_solutionSet.at(1) = -l_q2Q3SecondPKProblem.getAngle2Result().at(k); // * (-1) as kinematic chain was inverted
                const DualQuaternion l_g2 = ((m_screwCoordinatesDualQ.at(1)* (l_solutionSet.at(1) * 0.5)).dqExp());
                const DualQuaternion l_g3 = ((m_screwCoordinatesDualQ.at(2)* (l_solutionSet.at(2) * 0.5)).dqExp());

                // ------------------- Solve for q1 ---------------- //

                l_rightHandSide = l_g2 * l_g3 * l_g4 * l_g5 * l_g6 * l_g;
            
                const Quaternion l_endPointQ1 = l_rightHandSide.getTransformedVector(l_pointNotOnFirstScrew);
                l_q1Problem.compute(
                    l_pointOnLineQ1,
                    m_screwCoordinatesDualQ.at(0).getRealPart(),
                    l_pointNotOnFirstScrew,
                    l_endPointQ1,
                    -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(0)).max_position_,
                    -m_screwCoordinatesPtr->m_robotModelPtr->getVariableBounds(m_screwCoordinatesPtr->getJointsNames().at(0)).min_position_
                );

                if(l_q1Problem.getResult().has_value())
                {
                    l_solutionSet.at(0) = -l_q1Problem.getResult().value();
                    p_solutions.push_back(l_solutionSet);
                }
            }

        }

    }

    //fill q7 for every set of solutions
    for (auto &&l_solution : p_solutions)
    {
        l_solution.at(6) = p_q7;
    }
    
    return; 

}

} // namespace dualq_kinematics