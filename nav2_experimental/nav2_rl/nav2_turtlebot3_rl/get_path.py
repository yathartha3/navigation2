# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import NavigateToPose
import nav2_msgs
from geometry_msgs.msg import PoseStamped

class RLPathClient(Node):
    '''
    This class is designed to use the navigation stack to publish the global path.
    Functions implemented will send the current pose, and goal pose to the global planner
    using action interface, and get the path so that Reinforcemnet Learning algorithms
    can be used to follow the path.

    It uses ComputePathToPose action.

    This is a part of experimental package.
    '''
    def __init__(self):
        super().__init__('rl_path_client')
        self.action_client_ = ActionClient(self, NavigateToPose, 'NavigateToPose')
        
    def send_goal(self, goal_pose):
        goal_msg = nav2_msgs.action.NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.action_client_.wait_for_server()
        self.action_client_.send_goal_async(goal_msg)

def test(args=None):
    rclpy.init(args=args)

    action_client = RLPathClient()

    test_goal = PoseStamped()
    test_goal.pose.position.x = 0.0
    test_goal.pose.position.y = 2.0
    test_goal.pose.position.z = 0.0
    
    action_client.send_goal(test_goal)


if __name__ == '__main__':
    test()
