namespace dualq_kinematics
{

template<typename Scalar>
ScrewCoordinates<Scalar>::ScrewCoordinates(const moveit::core::RobotModel& robot_model)
{
    const urdf::ModelInterfaceSharedPtr l_urdf = robot_model.getURDF();
    std::map<std::string, urdf::JointSharedPtr> l_jointMap = retrieveKinChainJoints(robot_model);
    const size_t l_jointsNumber = l_jointMap.size();
    for (auto it = l_jointMap.begin(); it != l_jointMap.end(); ++it)
    {
        m_joints.push_back(it->first);
    }

    std::vector<urdf::Pose> l_joints2Parent;
    l_joints2Parent.resize(l_jointsNumber);
    for (size_t i = 0; i < l_jointsNumber; i++)
    {
        l_joints2Parent.at(i) = l_urdf->getJoint(m_joints.at(i))->parent_to_joint_origin_transform;
    }
    m_screwAxes.resize(l_jointsNumber);
    m_positions.resize(l_jointsNumber);
    //transformToScrewCoordinates(l_joints2Parent);
    
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
std::map<std::string, urdf::JointSharedPtr> ScrewCoordinates<Scalar>::retrieveKinChainJoints(const moveit::core::RobotModel& p_robotModel){
    const auto& l_initialJoints = p_robotModel.getURDF()->joints_;
    std::map<std::string, urdf::JointSharedPtr> l_finalJoints;

    std::vector<std::string> l_eeJoints;
    for (size_t j=0; j < p_robotModel.getEndEffectors().size(); j++)
    {
        for (size_t i = 0; i < p_robotModel.getEndEffectors().at(j)->getJointModels().size(); i++)
        {
            l_eeJoints.push_back(p_robotModel.getEndEffectors().at(j)->getJointModels().at(i)->getName());
        }
    }
    std::unordered_set<std::string> l_eeSet(l_eeJoints.begin(), l_eeJoints.end());

    for (auto it = l_initialJoints.begin(); it != l_initialJoints.end(); ++it)
    {
        bool l_isValid = true;
        switch (it->second->type)
        {
        case urdf::Joint::FIXED:
            l_isValid = false;
            break;
        case urdf::Joint::FLOATING:
            l_isValid = false;
            break;
        case urdf::Joint::UNKNOWN:
            l_isValid = false;
            break;
        default:
            l_isValid = true;
            break;
        }

        if(l_isValid && (l_eeSet.count(it->first) == 0))
        {
            l_finalJoints.emplace(it->first, it->second);
        }
    }

    return l_finalJoints;
}

template<typename Scalar>
void ScrewCoordinates<Scalar>::transformToScrewCoordinates(std::vector<urdf::Pose>& p_jnt2ParentPoses)
{
    std::vector<Transform> l_jnt2ParentTransforms;
    l_jnt2ParentTransforms.resize(p_jnt2ParentPoses.size());
    for (size_t i = 0; i < p_jnt2ParentPoses.size(); i++)
    {
        auto const l_jnt2ParentQuat = [&]{
            Quaternion l_quaternion;
            p_jnt2ParentPoses.at(i).rotation.getQuaternion(l_quaternion.x(), l_quaternion.y(), l_quaternion.z(), l_quaternion.w());
            return l_quaternion;
        }();

        Translation l_jnt2ParentTrans(p_jnt2ParentPoses.at(i).position.x, p_jnt2ParentPoses.at(i).position.y, p_jnt2ParentPoses.at(i).position.z);
        Transform l_transform = Transform::Identity();
        l_transform.linear() = l_jnt2ParentQuat.toRotationMatrix();
        l_transform.translation() = l_jnt2ParentTrans.vector();
        l_jnt2ParentTransforms.at(i) = l_transform;
    }
    
    std::vector<Transform> l_jnt2Base;
    l_jnt2Base.resize(p_jnt2ParentPoses.size());
    for (size_t i = 0; i < p_jnt2ParentPoses.size(); i++)
    {
        Transform l_transform = Transform::Identity();
        for (size_t j = 0; j <= i; j++)
        {
            l_transform = l_transform * l_jnt2ParentTransforms.at(j);
        }
        l_jnt2Base.at(i) = l_transform;  

        //Screw Axis is z vector in Base frame
        m_screwAxes.at(i) = Eigen::Translation<Scalar,3>( l_transform.rotation().col(2) );

        //Position on the screw axis is for example the translation from Base to Joint i
        m_positions.at(i) = Eigen::Translation<Scalar,3>( l_transform.translation() );
    }
    
}

}