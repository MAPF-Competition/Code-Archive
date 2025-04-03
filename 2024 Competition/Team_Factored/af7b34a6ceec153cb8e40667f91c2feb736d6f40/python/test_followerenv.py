import LoRR_MAPF.envs
import gymnasium as gym
from tqdm import tqdm

if __name__ == "__main__":
    env = gym.make("LoRR_MAPF/FollowerRandom-v0", render_mode='human')
    obs, info = env.reset()
    for _ in tqdm(range(100)):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
