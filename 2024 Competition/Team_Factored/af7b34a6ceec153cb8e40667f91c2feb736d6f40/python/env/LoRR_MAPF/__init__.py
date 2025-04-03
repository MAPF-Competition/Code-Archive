from gymnasium.envs.registration import register

register(
    id="LoRR_MAPF/GridWorld-v0",
    entry_point="LoRR_MAPF.envs:GridWorldEnv",
)
