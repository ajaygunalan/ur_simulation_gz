#!/usr/bin/env python
# Copyright 2023, FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import logging
import os
import pytest
import sys
import time
import unittest


import rclpy
from rclpy.node import Node
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ActionInterface,
)


# Increased timeout since sim_time could be slower... TODO: Make this more properly
TIMEOUT_EXECUTE_TRAJECTORY = 60

ROBOT_JOINTS = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# TODO: Add tf_prefix parametrization
# TODO: Currently, launching multiple simulations from this makes the old simulation not stop. This
# might change, once the gz launch system migration is done using gzserver and such....
# @launch_testing.parametrize(
# "ur_type",
# ["ur3", "ur3e", "ur5", "ur5e", "ur7e", "ur10", "ur10e", "ur12e", "ur16e", "ur15", "ur20", "ur30"],
# )
@pytest.mark.launch_test
def generate_test_description():
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "launch_rviz": "false",
            "gazebo_gui": "false",
            "start_joint_controller": "true",
        }.items(),
    )
    return LaunchDescription([ReadyToTest(), simulator])


class GazeboTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("ur_gazebo_test")
        time.sleep(1)
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):
        self._follow_joint_trajectory = ActionInterface(
            self.node,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )
        # TODO: Replace this timeout with a proper check whether the robot is initialized
        time.sleep(5)

    def test_trajectory(self):
        """Test robot movement."""
        # Construct test trajectory
        test_trajectory = [
            (Duration(sec=6, nanosec=0), [0.0 for j in ROBOT_JOINTS]),
            (Duration(sec=9, nanosec=0), [-0.5 for j in ROBOT_JOINTS]),
            (Duration(sec=12, nanosec=0), [-1.0 for j in ROBOT_JOINTS]),
        ]

        trajectory = JointTrajectory(
            # joint_names=[tf_prefix + joint for joint in ROBOT_JOINTS],
            joint_names=ROBOT_JOINTS,
            points=[
                JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
                for (test_time, test_pos) in test_trajectory
            ],
        )

        # Sending trajectory goal
        logging.info("Sending simple goal")
        goal_handle = self._follow_joint_trajectory.send_goal(trajectory=trajectory)
        self.assertTrue(goal_handle.accepted)

        # Verify execution
        result = self._follow_joint_trajectory.get_result(goal_handle, TIMEOUT_EXECUTE_TRAJECTORY)
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)
