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

};