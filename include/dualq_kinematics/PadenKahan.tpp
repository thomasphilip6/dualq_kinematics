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

}