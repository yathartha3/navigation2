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

from time import sleep
from threading import Thread

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from math import pi, atan2, sin, cos
import math
import random
import parameters

from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetEntityState, SetEntityState


class TurtlebotEnv():
    def __init__(self):
        self.node_ = rclpy.create_node('turtlebot3_env_rl')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node_)
        self.node_.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.act = 0
        self.done = False
        self.actions = [0.0, 0.0]  # Linear and Angular velocities

        self.collision = False
        self.collision_tol = 0.125
        self.laser_scan_range = [0] * 360
        self.states_input = [3.5] * 8  # Eight laser scans
        self.zero_div_tol = 0.01
        self.range_min = 0.0
        self.states = []

        self.current_pose = Pose()
        self.goal_pose = Pose()

        self.pub_cmd_vel = self.node_.create_publisher(Twist, 'cmd_vel', 1)
        self.sub_scan = self.node_.create_subscription(LaserScan, 'scan', self.scan_callback,
                                                       qos_profile_sensor_data)

        self.reset_simulation = self.node_.create_client(Empty, 'reset_simulation')
        self.reset_world = self.node_.create_client(Empty, 'reset_world')
        self.unpause_proxy = self.node_.create_client(Empty, 'unpause_physics')
        self.pause_proxy = self.node_.create_client(Empty, 'pause_physics')
        self.get_entity_state = self.node_.create_client(GetEntityState, 'get_entity_state')
        self.set_entity_state = self.node_.create_client(SetEntityState, 'set_entity_state')
        self.scan_msg_received = False
        self.t = Thread(target=self.executor.spin)
        self.t.start()
        self.time_factor = 10.0
        self.time_to_sample = 1.0

    def cleanup(self):
        self.t.join()

    # Rate object is not yet available in rclpy. Thus, we created this method to calculate the
    # difference between simulation time and system time
    def get_time_factor(self):
        sim_time_start = self.node_._clock.now()
        sleep(self.time_to_sample)
        sim_time_end = self.node_._clock.now()
        sim_time_dif = (sim_time_end.nanoseconds - sim_time_start.nanoseconds) / 1e9
        return sim_time_dif / self.time_to_sample

    def get_reward(self):
        # Total reward is based on goal distance and heading. However if we care only about getting near goal radius,
        # heading reward is not necessary. In case we want to orient to a given goal pose, the heading will be useful.
        reward = 0.0
        goal_dist = self.distance_to_goal()
        # Goal is reached if robot is within 0.5 meters of the goal
        if goal_dist > 0.5:
            distance_reward = -(goal_dist*goal_dist)
        else:
            distance_reward = 1000
            self.done = True
            print("Goal Reached")

        heading_reward = -0.5*self.get_heading()**2  # heading is not so important so x0.5

        reward += distance_reward
        reward += heading_reward

        if self.collision:    # this should not be in get_reward(). checked in scan callback
            reward += -500
            self.done = True
        return reward

    def scan_callback(self, LaserScan):
        self.scan_msg_received = True
        self.laser_scan_range.clear()
        self.laser_scan_range = []
        self.range_min = LaserScan.range_min
        range_max = LaserScan.range_max
        for i in range(len(LaserScan.ranges)):
            if LaserScan.ranges[i] == float('Inf'):
                self.laser_scan_range.append(range_max)
            elif LaserScan.ranges[i] < self.range_min + self.collision_tol - 0.05:
                self.laser_scan_range.append(self.range_min + self.collision_tol)
            else:
                self.laser_scan_range.append(LaserScan.ranges[i])

        self.states_input.clear()
        self.states_input = []

        step = int(len(LaserScan.ranges) / 8)
        for i in range(8):
            step = int(len(LaserScan.ranges) / 8)
            self.states_input.append(min(self.laser_scan_range[i * step:(i + 1) * step],
                                    default=0))

    def action_space(self):
        return len(self.actions)

    def observation_space(self):
        return len(self.states)

    def step(self, action):
        vel_cmd = Twist()
        self.act = action
        vel_cmd.linear.x = float(action[0])
        vel_cmd.angular.z = float(action[1])
        self.pub_cmd_vel.publish(vel_cmd)

        # TODO: Also test without setting velocities to zero after each step
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0

        sleep(parameters.LOOP_RATE / self.time_factor)
        # Publish zeros
        self.pub_cmd_vel.publish(vel_cmd)

        if self.check_collision():
            self.collision = True
            self.done = True

        get_reward = self.get_reward()
        return self.observation(), get_reward, self.done, {}

    def observation(self):
        self.get_robot_pose()
        self.states.clear()
        # states include laser points, goal distance, and heading
        self.states = [0] * (len(self.states_input) + 2)
        self.states[:8] = self.states_input[:]

        dist = self.distance_to_goal()
        heading = self.get_heading()

        self.states[8] = dist
        self.states[9] = heading

        return self.states

    def check_collision(self):
        if min(self.states_input) < (self.range_min + self.collision_tol):
           print("Colistion proximity... " + str(min(self.laser_scan_range)))
           return True

    def stop_action(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        self.pub_cmd_vel.publish(vel_cmd)

    def set_random_robot_pose(self):
        sleep(1.0)
        while not self.set_entity_state.wait_for_service(timeout_sec=1.0):
            print('Set entity state service is not available...')
        random_pose = self.get_random_pose()
        req = SetEntityState.Request()
        req.state.name = 'turtlebot3_waffle'
        req.state.pose.position.x = random_pose.position.x
        req.state.pose.position.y = random_pose.position.y
        req.state.pose.position.z = 0.0
        req.state.pose.orientation.x = random_pose.orientation.x
        req.state.pose.orientation.y = random_pose.orientation.y
        req.state.pose.orientation.z = random_pose.orientation.z
        req.state.pose.orientation.w = random_pose.orientation.w
        future = self.set_entity_state.call_async(req)

        while not future.done() and rclpy.ok():
            sleep(0.1)
        sleep(1.0)

    def get_robot_pose(self):
        while not self.get_entity_state.wait_for_service(timeout_sec=1.0):
            print('get entity state service is not available...')
        req = GetEntityState.Request()
        req.name = 'turtlebot3_waffle'
        future = self.get_entity_state.call_async(req)

        while not future.done() and rclpy.ok():
            sleep(0.01 / self.time_factor)

        self.current_pose.position.x = future.result().state.pose.position.x
        self.current_pose.position.y = future.result().state.pose.position.y
        self.current_pose.position.z = future.result().state.pose.position.z
        self.current_pose.orientation.x = future.result().state.pose.orientation.x
        self.current_pose.orientation.y = future.result().state.pose.orientation.y
        self.current_pose.orientation.z = future.result().state.pose.orientation.z
        self.current_pose.orientation.w = future.result().state.pose.orientation.w

    def get_random_pose(self):
        random_pose = Pose()
        yaw = random.uniform(0, pi * 2)

        random_pose.position.x = random.uniform(-4, 4)
        random_pose.position.y = random.uniform(-4, 4)
        random_pose.position.z = 0.0
        random_pose.orientation.x = 0.0
        random_pose.orientation.y = 0.0
        random_pose.orientation.z = sin(yaw * 0.5)
        random_pose.orientation.w = cos(yaw * 0.5)
        return random_pose

    def get_zero_pose(self):
        zero_pose = Pose()
        # Positions and orientations are zero by default
        zero_pose.orientation.w = 1.0
        return zero_pose

    def distance_to_goal(self):
        self.get_robot_pose()
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def get_heading(self):
        goal_angle = math.atan2(self.goal_pose.position.y - self.current_pose.position.y,
                                self.goal_pose.position.x - self.current_pose.position.x)
        current_yaw = self.get_yaw(self.current_pose)
        heading = goal_angle - current_yaw

        if heading > pi:
            heading -= 2 * pi
        elif heading < -pi:
            heading += 2 * pi

        return heading

    def get_yaw(self, q):
        yaw = atan2(2.0 * (q.orientation.x * q.orientation.y + q.orientation.w * q.orientation.z),
                    q.orientation.w * q.orientation.w + q.orientation.x * q.orientation.x -
                    q.orientation.y * q.orientation.y - q.orientation.z * q.orientation.z)
        return yaw

    def reset(self):
        while not self.reset_world.wait_for_service(timeout_sec=1.0):
            print('Reset world service is not available...')
        self.reset_world.call_async(Empty.Request())

        self.time_factor = self.get_time_factor()
        #print('Time factor is: ' + "%f" % self.time_factor)

        self.scan_msg_received = False
        self.stop_action()
        while not self.reset_world.wait_for_service(timeout_sec=1.0):
            print('Reset world service is not available...')
        self.reset_world.call_async(Empty.Request())

        while not self.reset_simulation.wait_for_service(timeout_sec=1.0):
            print('Reset simulation service is not available...')
        self.reset_simulation.call_async(Empty.Request())

        self.set_random_robot_pose()
        self.get_robot_pose()
        self.goal_pose = self.get_zero_pose()

        self.laser_scan_range = [0] * 360
        self.states_input = [3.5] * 8
        while not self.scan_msg_received and rclpy.ok():
            sleep(0.1 / self.time_factor)
        self.collision = False
        self.done = False

        return self.observation()
