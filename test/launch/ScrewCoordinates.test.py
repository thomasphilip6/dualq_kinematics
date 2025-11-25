import unittest
import launch_testing
import pytest

import launch
import launch_ros
import launch_testing.actions
from launch_testing.util import KeepAliveProc
import rclpy

from moveit_configs_utils import MoveItConfigsBuilder

@pytest.mark.rostest
def generate_test_description():
    #todo change panda to a launch argument
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    screw_coordinates_node =  launch_ros.actions.Node(
                package="dualq_kinematics",
                executable="dualq_kinematics_ScrewCoordinatesTest",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic
                ],
    )

    return (
        launch.LaunchDescription([
            
            #Node under test
            screw_coordinates_node,
            KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),

            #Launch tests 0.5s later
            # launch.actions.TimerAction(
            #      period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            
        ]),
        {"screw_coordinates_node": screw_coordinates_node},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, screw_coordinates_node):
        proc_info.assertWaitForShutdown(process=screw_coordinates_node, timeout=4000.0)

# Post-shutdown tests
@launch_testing.post_shutdown_test()
class ScrewCoordinatesShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)