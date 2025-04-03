from LoRR_MAPF.envs.grid_world import LoRR
from LoRR_MAPF.envs.follower_grid_world import FollowerLoRR
from gymnasium.envs.registration import register
from pathlib import Path

examples_path = Path(__file__).parent.parent / "example_problems"

register(
    id="LoRR_MAPF/FollowerRandom-v0",
    entry_point="LoRR_MAPF.envs:FollowerLoRR",
    kwargs={
        "domain_path": examples_path / "random.domain",
        "map_name": "random-32-32-20.map",
        "num_agents": 100,
        "block_all_actions": False,
        "collision_penalty": 0,
        "unassigned_reward": 10,
        "stray_penalty": 0,
    }
)
register(
    id="LoRR_MAPF/FollowerEmpty-v0",
    entry_point="LoRR_MAPF.envs:FollowerLoRR",
    kwargs={
        "domain_path": examples_path / "empty.domain",
        "map_name": "empty_8_8.map",
        "num_agents": 4,
        "block_all_actions": False,
        "collision_penalty": 0,
        "unassigned_reward": 10,
        "stray_penalty": 0,
    }
)
register(
    id="LoRR_MAPF/FollowerCity-v0",
    entry_point="LoRR_MAPF.envs:FollowerLoRR",
    kwargs={
        "domain_path": examples_path / "city.domain",
        "map_name": "Paris_1_256.map",
        "num_agents": 250,
        "block_all_actions": False,
        "collision_penalty": 0,
        "unassigned_reward": 10,
        "stray_penalty": 0,
    }
)
register(
    id="LoRR_MAPF/FollowerGame-v0",
    entry_point="LoRR_MAPF.envs:FollowerLoRR",
    kwargs={
        "domain_path": examples_path / "game.domain",
        "map_name": "brc202d.map",
        "num_agents": 500,
        "block_all_actions": False,
        "collision_penalty": 0,
        "unassigned_reward": 10,
        "stray_penalty": 0,
    }
)
register(
    id="LoRR_MAPF/FollowerSortation-v0",
    entry_point="LoRR_MAPF.envs:FollowerLoRR",
    kwargs={
        "domain_path": examples_path / "warehouse.domain",
        "map_name": "sortation_large.map",
        "num_agents": 2000,
        "block_all_actions": False,
        "collision_penalty": 0,
        "unassigned_reward": 10,
        "stray_penalty": 0,
    }
)
register(
    id="LoRR_MAPF/FollowerWarehouse-v0",
    entry_point="LoRR_MAPF.envs:FollowerLoRR",
    kwargs={
        "domain_path": examples_path / "warehouse.domain",
        "map_name": "warehouse_large.map",
        "num_agents": 5000,
        "block_all_actions": False,
        "collision_penalty": 0,
        "unassigned_reward": 10,
        "stray_penalty": 0,
    }
)