from invoke import task
import json
import os
import shutil
import yaml

def load_yaml(file_path):
    assert os.path.exists(file_path) and os.path.isfile(file_path)
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

DOMAINS = load_yaml("domains.yaml")
AGENTS = load_yaml("agents.yaml")

RUN_AGENT_PATH = "./python/pyMAPFPlanner.py"

def get_map(schema_file: str):
    assert os.path.exists(schema_file) and os.path.isfile(schema_file)
    with open(schema_file, "r") as f:
        schema = json.load(f)

    rel_map_path = schema["mapFile"]
    domain_path = os.path.dirname(schema_file)
    return os.path.join(domain_path, rel_map_path)

RUN_MSG = f"Domain name to simulate. Available names: {', '.join(DOMAINS.keys())}"
@task(help={'domain': RUN_MSG})
def run(c, domain: str, agent: str = "default", output_file: str = None):
    assert domain in DOMAINS, "Invalid domain name"
    assert agent in AGENTS, "Invalid agent name"
    if output_file is None:
        output_file = f"{domain}.json"

    print("loading agent:", agent)
    shutil.copyfile(AGENTS[agent], RUN_AGENT_PATH)

    domain_f = DOMAINS[domain]
    print(f"Running Simulation with domain {domain}: {domain_f}")
    c.run("./compile.sh", hide=True)
    c.run(f"./build/lifelong --inputFile {domain_f} -o {output_file}")
    os.remove(RUN_AGENT_PATH)

@task
def viz(c, domain, file=None):
    assert domain in DOMAINS, "Invalid domain name"
    if file is None:
        file = f"{domain}.json"

    domain_f = DOMAINS[domain]
    print(f"Visualizing plan {file} for domain {domain}")
    map_path = get_map(domain_f)
    c.run(f"python PlanViz/script/run.py --map {map_path} --plan {file} --grid --aid --tid")
