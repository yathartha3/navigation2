import argparse
import os
import rclpy
from gym import wrappers
from config import Config
from core.util import time_seq, load_obj
from tester import Tester
from trainer import Trainer
from turtlebot3_env_rl import TurtlebotEnv
from ddpg import DDPG


parser = argparse.ArgumentParser(description='')
parser.add_argument('--train', dest='train', action='store_true', help='train model')
parser.add_argument('--test', dest='test', action='store_true', help='test model')
parser.add_argument('--env', default='turtlebot3', type=str, help='gym environment')
parser.add_argument('--gamma', default=0.99, type=float, help='discount')
parser.add_argument('--episodes', default=2000, type=int)
parser.add_argument('--batch_size', default=128, type=int)
parser.add_argument('--epsilon', default=1.0, type=float, help='noise epsilon')
parser.add_argument('--eps_decay', default=0.001, type=float, help='epsilon decay')
parser.add_argument('--max_buff', default=1000000, type=int, help='replay buff size')
parser.add_argument('--output', default='out', type=str, help='result output dir')
parser.add_argument('--cuda', dest='cuda', action='store_true', help='use cuda')
parser.add_argument('--model_path', type=str, help='if test mode, import the model')
parser.add_argument('--load_config', type=str, help='load the config from obj file')

step_group = parser.add_argument_group('step')
step_group.add_argument('--customize_step', dest='customize_step', action='store_true', help='customize max step per episode')
step_group.add_argument('--max_steps', default=200, type=int, help='max steps per episode')

record_group = parser.add_argument_group('record')
record_group.add_argument('--record', dest='record', action='store_true', help='record the video')
record_group.add_argument('--record_ep_interval', default=20, type=int, help='record episodes interval')

checkpoint_group = parser.add_argument_group('checkpoint')
checkpoint_group.add_argument('--checkpoint', dest='checkpoint', action='store_true', help='use model checkpoint')
checkpoint_group.add_argument('--checkpoint_interval', default=500, type=int, help='checkpoint interval')

retrain_group = parser.add_argument_group('retrain')
retrain_group.add_argument('--retrain', dest='retrain', action='store_true', help='retrain model')
retrain_group.add_argument('--retrain_model', type=str, help='retrain model path')


if __name__=="__main__":
    args = parser.parse_args()

    # Manually set here
    args.train = True
    args.test = False
    # args.model_path = os.getcwd() + "/out/turtlebot3-run40"

    config = Config()
    config.env = args.env

    config.gamma = args.gamma
    config.episodes = args.episodes
    config.max_steps = args.max_steps
    config.batch_size = args.batch_size
    config.epsilon = args.epsilon
    config.eps_decay = args.eps_decay
    config.max_buff = args.max_buff
    config.output = args.output
    config.use_cuda = args.cuda
    config.checkpoint = args.checkpoint
    config.checkpoint_interval = args.checkpoint_interval

    config.learning_rate = 1e-3
    config.learning_rate_actor = 1e-4
    config.epsilon_min = 0.001
    config.epsilon = 1.0
    config.tau = 0.001
    config.seed = 7
    rclpy.init(args=None)

    # Initialize the environment
    env = TurtlebotEnv()
    env.reset()

    config.action_dim = int(env.action_space())
    config.action_lim = 0.25
    config.state_dim = int(env.observation_space())

    if args.load_config is not None:
            config = load_obj(args.load_config)

    agent = DDPG(config)

    if args.train:
        trainer = Trainer(agent, env, config,
                          record=args.record)
        trainer.train()

    elif args.retrain:
        if args.retrain_model is None:
            print('please add the retrain model path:', '--retrain_model xxxx')
            exit(0)

        ep, step = agent.load_checkpoint(args.retrain_model)
        trainer = Trainer(agent, env, config,
                          record=args.record)
        trainer.train(ep, step)


    elif args.test:
        if args.model_path is None:
            print('please add the model path:', '--model_path xxxx')
            exit(0)

        # record
        if args.record:
            os.makedirs('video', exist_ok=True)
            filepath = 'video/' + args.env + '-' + time_seq()
            env = wrappers.Monitor(env, filepath, video_callable=lambda episode_id: episode_id % 25 == 0)

        tester = Tester(agent, env,
                        model_path=args.model_path)
        tester.test(visualize=False)
        # TODO: How to handle visualize? Users might want to use Gazebo headless while training, and visualize while testing

    else:
        print('choose train or test:', '--train or --test')
