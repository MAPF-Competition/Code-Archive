import yaml
import os
import argparse
from multiprocessing import Pool
import subprocess
from time import time
import csv
from datetime import datetime


def parse_args():
    # parse yaml config file path
    args = argparse.ArgumentParser()
    args.add_argument("--config", type=str, default="config.yaml")
    args.add_argument("--timelimit", type=int, default=5000)
    args = args.parse_args()
    return args


def run_test(args):
    test, timeout = args
    ipf = test
    test_name = ipf.split("/")[-1]
    opf = f"{test_name}"
    logf = f"{test_name}.log"
    cmd = COMMAND.format(ipf=ipf, opf=opf, logf=logf)

    start_time = time()
    solved = 0

    try:
        subprocess.run(cmd, shell=True, timeout=timeout, check=True)
        solved = 1
    except subprocess.TimeoutExpired:
        pass
    except subprocess.CalledProcessError:
        pass

    run_time = time() - start_time
    return test_name, run_time, solved


def on_error(e):
    print(f"\nAn error occurred: {e}")


t_start = time()  # Record the start time
num_total_tasks = 0
cnt_fin = 0
cnt_solved = 0


def handle_result(result):
    global cnt_fin, cnt_solved, results
    test_name, run_time, solved = result

    cnt_fin += 1
    cnt_solved += solved

    num_agents = test_name.split(".")[0].split("_")[-1]

    # load log file and get the number of task finished
    with open(f"results/latest/logs/{test_name}.log", "r") as f:
        for line in f.readlines()[::-1]:
            if "tasks has been finished by far in total" in line:
                num_tasks = int(line.split()[0])
                break
    # Timestep
    with open(f"results/latest/logs/{test_name}.log", "r") as f:
        for line in f.readlines()[::-1]:
            if "Timestep" in line:
                time_step = int(line.split()[1])
                break

    results.append(
        (
            test_name,
            num_agents,
            solved,
            run_time,
            num_tasks,
            time_step,
            round(num_tasks / time_step, 2),
        )
    )
    print(
        f"\r{(time() - t_start):.2f} sec\t"
        f"{cnt_fin}/{num_total_tasks} "
        f"({(cnt_fin / num_total_tasks * 100):.2f}%) tasks have been finished, "
        f"solved: {cnt_solved}/{num_total_tasks} ({(cnt_solved / cnt_fin * 100 if cnt_fin else 0):.2f}%)",
        end="",
    )


def run_tests_with_timeout(tests, timeout):
    global t_start, num_total_tasks, cnt_fin, cnt_solved
    t_start = time()  # Record the start time
    num_total_tasks = len(tests)
    cnt_fin = 0
    cnt_solved = 0

    try:
        with Pool() as pool:
            results = [
                pool.apply_async(
                    run_test,
                    ((test, timeout),),
                    callback=handle_result,
                    error_callback=on_error,
                )
                for test in tests
            ]
            pool.close()
            pool.join()
            return results
    except KeyboardInterrupt:
        print("\nExecution was interrupted by the user.")
        pool.terminate()
        pool.join()
        return []


def main():
    global results
    results = []

    # Create results directory and log directory
    if not os.path.exists("results"):
        os.makedirs("results")
    # Get the current date and time to include in the folder name
    now = datetime.now()
    date_time = now.strftime("%Y%m%d_%H%M%S")

    # Create a new parent folder with the current date and time
    parent_folder = f"results/{date_time}"
    os.makedirs(parent_folder, exist_ok=True)

    # Create subfolders for output and logs
    output_folder = os.path.join(parent_folder, "output")
    os.makedirs(output_folder, exist_ok=True)

    log_folder = os.path.join(parent_folder, "logs")
    os.makedirs(log_folder, exist_ok=True)

    # Create or update a symbolic link to the latest results folder
    symlink_path = "results/latest"
    if os.path.exists(symlink_path):
        os.remove(symlink_path)
    os.symlink(os.path.abspath(parent_folder), symlink_path)

    # Update the COMMAND with the new folders for output and logs
    global COMMAND
    COMMAND = f"build/lifelong --inputFile {{ipf}} -o {output_folder}/{{opf}} > {log_folder}/{{logf}}"

    # Read config file
    args = parse_args()
    with open(args.config, "r") as f:
        config = yaml.safe_load(f)
    # Run tests with multiprocessing
    run_tests_with_timeout(config["tests"], timeout=args.timelimit)
    print()

    # After running all tests, write the results to a CSV file
    csv_filename = os.path.join(parent_folder, "results.csv")
    with open(csv_filename, "w", newline="") as csvfile:
        fieldnames = [
            "test_name",
            "num_agents",
            "solved",
            "run_time",
            "num_tasks",
            "time_step",
            "throughput",
        ]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for (
            test_name,
            num_agents,
            solved,
            run_time,
            num_tasks,
            time_step,
            throughput,
        ) in results:
            writer.writerow(
                {
                    "test_name": test_name,
                    "num_agents": num_agents,
                    "solved": solved,
                    "run_time": run_time,
                    "num_tasks": num_tasks,
                    "time_step": time_step,
                    "throughput": throughput,
                }
            )


if __name__ == "__main__":
    main()
