import unittest
import launch_testing

import launch
import launch_ros
import launch_testing.actions
import rclpy

from moveit_configs_utils import MoveItConfigsBuilder

def generate_test_description():
    #todo change panda to a launch argument
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    return (
        launch.LaunchDescription([
            
            #Node under test
            launch_ros.actions.Node(
                package="dualq_kinematics",
                executable="dualq_kinematics_ScrewCoordinatesTest",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic
                ],
            ),

            #Launch tests 0.5s later
            launch.actions.TimerAction(
                period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            
        ]),{}
    )

# Post-shutdown tests
@launch_testing.post_shutdown_test()
class ScrewCoordinatesShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)