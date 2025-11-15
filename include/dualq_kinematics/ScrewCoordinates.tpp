namespace dualq_kinematics
{

template<typename Scalar>
ScrewCoordinates<Scalar>::ScrewCoordinates(const moveit::core::RobotModel& robot_model)
{
    const urdf::ModelInterfaceSharedPtr l_urdf = robot_model.getURDF();
    const size_t l_jointsNumber = l_urdf->getRoot()->child_links.size();
    //todo compare size of chilc_links and joints
    std::map<std::string, urdf::JointSharedPtr> l_jointMap = l_urdf->joints_;
    for (auto it = l_jointMap.begin(); it != l_jointMap.end(); ++it)
    {
        m_joints.push_back(it->first);
    }

    std::vector<urdf::Pose> l_joints2Parent;
    l_joints2Parent.reserve(l_jointsNumber);
    for (size_t i = 0; i < l_jointsNumber; i++)
    {
        l_joints2Parent.at(i) = l_urdf->getJoint(m_joints.at(i))->parent_to_joint_origin_transform;
    }
    
}

template<typename Scalar>
const typename std::vector< typename ScrewCoordinates<Scalar>::Translation >& ScrewCoordinates<Scalar>::getScrewAxes() const
{
    return m_screwAxes;
}

template<typename Scalar>
const  std::vector< typename ScrewCoordinates<Scalar>::Translation >& ScrewCoordinates<Scalar>::getPositions() const
{
    return m_positions;
}

template<typename Scalar>
const std::vector<std::string>& ScrewCoordinates<Scalar>::getJointsNames() const
{
    return m_joints;
}

template<typename Scalar>
const typename ScrewCoordinates<Scalar>::Transform& ScrewCoordinates<Scalar>::getLastJnt2EE() const
{
    return m_ee;
}

template<typename Scalar>
void ScrewCoordinates<Scalar>::transformToScrewCoordinates(std::vector<urdf::Pose>& p_jnt2ParentPoses)
{
    std::vector<Transform> l_jnt2ParentTransforms;
    l_jnt2ParentTransforms.reserve(p_jnt2ParentPoses.size());
    for (size_t i = 0; i < p_jnt2ParentPoses.size(); i++)
    {
        auto const l_jnt2ParentQuat = [](std::vector<urdf::Pose>& p_jnt2ParentPoses, size_t i){
            Quaternion l_quaternion;
            p_jnt2ParentPoses.at(i).rotation.getQuaternion(l_quaternion.x(), l_quaternion.y(), l_quaternion.z(), l_quaternion.w());
            return l_quaternion;
        };
        const Translation l_jnt2ParentTrans(p_jnt2ParentPoses.at(i).position.x(), p_jnt2ParentPoses.at(i).position.y(), p_jnt2ParentPoses.at(i).position.z());
        
        l_jnt2ParentTransforms.at(i) = [](const Quaternion l_jnt2ParentQuat, const Translation l_jnt2ParentTrans){
            Transform l_transform = Transform::Identity();
            l_transform.linear() = l_jnt2ParentQuat.toRotationMatrix();
            l_transform.translation() = l_jnt2ParentTrans;
            return l_transform;
        };
    }
    
    std::vector<Transform> l_jnt2Base;
    l_jnt2ParentTransforms.reserve(p_jnt2ParentPoses.size());
}

};