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

from turtlebot3_env_oa import TurtlebotEnv
from dqn import DQN

import random
import numpy as np
import tensorflow as tf
import copy
import rclpy
from rclpy.node import Node
from time import sleep
import parameters
from tensorboardX import SummaryWriter


def trainModel(env, action_size):
    logger = SummaryWriter()
    state = env.reset()     # Required to populate observation space. # TODO: fix this
    observation_space = env.observation_space()
    agent = DQN(observation_space, action_size)
    target_model_update_counter = 0
    agent.step = 0
    for episode in range(parameters.EPISODES):
        episode_reward = 0.0
        print("Episode number: " + str(episode))
        state = env.reset()
        observation_size = len(state)
        state = np.reshape(state, [1, observation_size])
        done = False
        agent.step += 1
        train_count = 0
        reward = 0.0
        game_count = 0

        while not done and rclpy.ok():
            game_count += 1
            agent.step += 1

            # Forward the DQN with states to get the action
            action = agent.get_action(state)

            # Perform the action and get next state, reward and done flag if robot collides
            next_state, reward, done = env.step(action)
            
            # Check if end of max time-steps
            if game_count == 100:
                done = True
            
            next_state = np.reshape(next_state, [1, observation_space])
            agent.save_to_memory(state, action, reward, next_state, done)
            state = copy.deepcopy(next_state)
            
            episode_reward += reward
            
            if done:
                print("Done")
                env.stop_action()
                logger.add_scalar('episode_rewards', episode_reward, episode)
                print("Episode reward = " + str(episode_reward))

        for _ in range(10):
            env.stop_action()
            agent.experience_replay()
    
        if episode % parameters.TARGET_MODEL_UPDATE_STEP == 0:
            agent.save_load_model_weights()

        agent.model.save('navigator_model.h5')

    # Episodes ended
    logger.close()


def main(args=None):
    rclpy.init(args=args)
    env = TurtlebotEnv()
    action_size = env.action_space()

    # Ctrl-C doesn't make rclpy.ok() to return false. Thus, we catch the exception with
    # `finally` to shutdown ros and terminate the background thread cleanly.
    try:
        trainModel(env, action_size)
    except KeyboardInterrupt:
        pass
    finally:
        env.stop_action()
        rclpy.shutdown()
        env.cleanup()
    return


if __name__ == "__main__":
    main()
