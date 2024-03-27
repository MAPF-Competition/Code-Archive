#!/usr/bin/env python3

import numpy as np

import json
import multiprocessing
import os
import sys
import subprocess

from glob import glob
from tqdm import tqdm

PROBLEMS = [
        "random.domain/random_*.json",
        "warehouse.domain/warehouse_small_*.json"
]

ROOT = os.path.dirname(os.path.dirname(__file__))
PROBLEM_FOLDER = os.path.join(ROOT, "example_problems")
EXECUTABLE = os.path.join(ROOT, "build", "lifelong")

def problem_name(path):
    return os.path.basename(path)[:-7]

def number_of_agents(path):
    return int(os.path.basename(path).split("_")[-1][:-5])

def eval_algorithm(args):
    input_path, simulation_time = args
    out = "/tmp/out-" + os.path.basename(input_path)
    cmd = [EXECUTABLE, "--inputFile", input_path, "-o", out, "--simulationTime", str(simulation_time), "--planTimeLimit", "1"]
    process = subprocess.run(cmd, stdout=subprocess.DEVNULL, check=True)
    with open(out, "r") as f:
        d = json.load(f)
        tasks_solved = d["numTaskFinished"]
        max_plan_time = max(d["plannerTimes"])
        all_valid = 1 if d["AllValid"] == "Yes" else 0

    return (input_path,
            tasks_solved,
            tasks_solved/simulation_time,
            tasks_solved/simulation_time/number_of_agents(input_path),
            max_plan_time,
            all_valid)

def print_table(table, agent_tics, title, averages=True, rounding=4):

    print("#"*(len(title)+4))
    print("# " + title + " #")
    print("#"*(len(title)+4))
    first_column_width = 20
    column_width = 12
    if averages:
        row_averages = np.nanmean(table, 1)
        column_averages = np.nanmean(table, 0)
        total_average = np.nanmean(table)
        print(" "*first_column_width + "".join(f"{t:{column_width}}" for t in agent_tics) + f"|{'avg':>{column_width}}")
        for idx, problem in enumerate(PROBLEMS):
            name = problem_name(problem)
            print(f"{name:<{first_column_width}}", end="")
            for value in table[idx]:
                if np.isnan(value):
                    print(f"{'-':>{column_width}}", end="")
                else:
                    print(f"{value:>{column_width}.{rounding}f}", end="")
            print(f"|{row_averages[idx]:>{column_width}.{rounding}f}")
        print("-"*first_column_width + "-"*(column_width*(table.shape[1]+1)+1))
        print(f"{'avg':<{first_column_width}}", end="")
        for value in column_averages:
            print(f"{value:>{column_width}.{rounding}f}", end="")
        print(f"|{total_average:>{column_width}.{rounding}f}")
    else:
        print(" "*first_column_width + "".join(f"{t:{column_width}}" for t in agent_tics))
        for idx, problem in enumerate(PROBLEMS):
            name = problem_name(problem)
            print(f"{name:<{first_column_width}}", end="")
            for value in table[idx]:
                if np.isnan(value):
                    print(f"{'-':>{column_width}}", end="")
                else:
                    print(f"{value:>{column_width}.{rounding}f}", end="")
            print()

    print()


def main(argv):
    if len(argv) > 1:
        simulation_time = int(argv[1])
    else:
        simulation_time = 100;
    instances = {}
    for p in PROBLEMS:
        name = problem_name(p)
        instances[name] = sorted(glob(os.path.join(PROBLEM_FOLDER, p)), key=number_of_agents)

    instances_flat = []
    for l in instances.values():
        for instance in l:
            instances_flat.append((instance,simulation_time))

    pool = multiprocessing.Pool()

    results = {}
    for instance,*result in tqdm(pool.imap_unordered(eval_algorithm, instances_flat), total=len(instances_flat)):
        results[instance] = result

    agent_tics = set()
    for instance,_ in instances_flat:
        agent_tics.add(number_of_agents(instance))
    agent_tics = sorted(agent_tics)

    table1 = np.zeros((len(PROBLEMS),len(agent_tics)))
    table2 = np.zeros_like(table1)
    table3 = np.zeros_like(table1)
    table4 = np.zeros_like(table1)
    table5 = np.zeros_like(table1)
    for idx, problem in enumerate(PROBLEMS):
        for jdx, nags in enumerate(agent_tics):
            instance_name = os.path.join(PROBLEM_FOLDER, problem.replace("*", str(nags)))
            table1[idx][jdx] = results.get(instance_name, (np.nan,)*5)[0]
    for idx, problem in enumerate(PROBLEMS):
        for jdx, nags in enumerate(agent_tics):
            instance_name = os.path.join(PROBLEM_FOLDER, problem.replace("*", str(nags)))
            table2[idx][jdx] = results.get(instance_name, (np.nan,)*5)[1]
    for idx, problem in enumerate(PROBLEMS):
        for jdx, nags in enumerate(agent_tics):
            instance_name = os.path.join(PROBLEM_FOLDER, problem.replace("*", str(nags)))
            table3[idx][jdx] = results.get(instance_name, (np.nan,)*5)[2]
    for idx, problem in enumerate(PROBLEMS):
        for jdx, nags in enumerate(agent_tics):
            instance_name = os.path.join(PROBLEM_FOLDER, problem.replace("*", str(nags)))
            table4[idx][jdx] = results.get(instance_name, (np.nan,)*5)[3]
    for idx, problem in enumerate(PROBLEMS):
        for jdx, nags in enumerate(agent_tics):
            instance_name = os.path.join(PROBLEM_FOLDER, problem.replace("*", str(nags)))
            table5[idx][jdx] = results.get(instance_name, (np.nan,)*5)[4]

    print_table(table1, agent_tics, "#Tasks solved", True, rounding=0)
    print_table(table2, agent_tics, "#Tasks solved / timestep", True)
    print_table(table3, agent_tics, "#Tasks solved / (timestep*agent)", True)
    print_table(table4, agent_tics, "Max. planning time (s)", True)
    print_table(table5, agent_tics, "All valid", False, rounding=0)


if __name__ == "__main__":
    main(sys.argv)

