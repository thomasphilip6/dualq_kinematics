import unittest
import launch_testing
import pytest

import launch
import launch_ros
import launch_testing.actions
from launch_testing.util import KeepAliveProc
import rclpy

from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

@pytest.mark.rostest
def generate_test_description():
    #todo change panda to a launch argument
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()

    franka_kin_solver_node =  launch_ros.actions.Node(
                package="dualq_kinematics",
                executable="dualq_kinematics_FrankaKinSolverTest",

                #Uncomment following line to debug with gdb, gdbserver is used as this is an integration test
                prefix=['gdbserver localhost:3000'],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,

                ],
                output="screen",
    )

    return (
        launch.LaunchDescription([
            
            #Node under test
            franka_kin_solver_node,
            KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),

            #Launch tests 0.5s later
            # launch.actions.TimerAction(
            #      period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            
        ]),
        {"franka_kin_solver_node": franka_kin_solver_node},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, franka_kin_solver_node):
        proc_info.assertWaitForShutdown(process=franka_kin_solver_node, timeout=40000.0)

# Post-shutdown tests
@launch_testing.post_shutdown_test()
class FrankaKinSolverShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)