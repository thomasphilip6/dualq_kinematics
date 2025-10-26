namespace dualq_kinematics
{

template<typename Scalar>
ScrewCoordinates<Scalar>::ScrewCoordinates(const moveit::core::RobotModel& robot_model)
{
    const urdf::ModelInterfaceSharedPtr l_urdf = robot_model.getURDF();
}

};