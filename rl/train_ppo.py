#!/usr/bin/env python3
"""Stable-Baselines3 PPO example; start matching simulator instances first."""
import argparse
import os
from pathlib import Path


def make_env(rank):
    def factory():
        os.environ["ROS_DOMAIN_ID"] = str(42 + rank)
        os.environ["IGN_PARTITION"] = f"patrol_rl_{rank}"
        from patrol_gym import PatrolCarEnv
        return PatrolCarEnv()
    return factory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--instances", type=int, default=1)
    parser.add_argument("--steps", type=int, default=1_000_000)
    parser.add_argument("--output", type=Path, default=Path("runs/ppo_patrol"))
    args = parser.parse_args()
    if args.instances < 1:
        parser.error("--instances must be positive")
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
    env = (DummyVecEnv([make_env(0)]) if args.instances == 1 else
           SubprocVecEnv([make_env(i) for i in range(args.instances)], start_method="spawn"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=str(args.output.parent),
                n_steps=1024, batch_size=256, learning_rate=3e-4,
                policy_kwargs={"net_arch": [256, 256]})
    try:
        model.learn(total_timesteps=args.steps, progress_bar=True)
        model.save(str(args.output))
    finally:
        env.close()


if __name__ == "__main__":
    main()
