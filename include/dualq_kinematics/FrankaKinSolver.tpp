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
    for (auto &&l_dq : m_screwCoordinatesDualQ)
    {
        try
        {
            const bool l_isUnit = l_dq.isUnit(c_tolerance);
            if(!l_isUnit)
            {
                throw(l_isUnit);
            }
        }
        //todo change to a parameter to an error for better error handling
        catch(const bool& e)
        {
            //do something
        }
        
    }

    m_tip2BaseInit.value() = DualQuaternion(m_screwCoordinates.value().getTip2BaseInit());

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
    Scalar l_a7 = m_screwCoordinates.value().getPositions().at(6)(0);

    // r7 = eeWanted.translation - df * eeWanted.z()
    Vector3 l_r7 = p_tip2BaseWanted.translation() - (m_screwCoordinates.value().getTip2BaseInit().translation().z() - m_screwCoordinates.value().getPositions().at(6)(2))*p_tip2BaseWanted.rotation().col(2);

    Quaternion l_eeY(0.0, p_tip2BaseWanted.rotation().col(1)(0), p_tip2BaseWanted.rotation().col(1)(1), p_tip2BaseWanted.rotation().col(1)(2));
    Quaternion l_s7(0.0, p_tip2BaseWanted.rotation().col(2)(0), p_tip2BaseWanted.rotation().col(2)(1), p_tip2BaseWanted.rotation().col(2)(2));
    Quaternion l_s6;
    if (p_q7 != 0.0)
    {
        l_s6 = DualQuaternion::quaternionExp(-p_q7*l_s7)*l_eeY;
    }
    else
    {
        l_s6 = l_eeY;
    }
    Quaternion l_product = l_s6 * l_s7;
    p_wrist(0) = l_r7(0) - l_a7*l_product.x();
    p_wrist(1) = l_r7(1) - l_a7*l_product.y();
    p_wrist(2) = l_r7(2) - l_a7*l_product.z();

}

} // namespace dualq_kinematics