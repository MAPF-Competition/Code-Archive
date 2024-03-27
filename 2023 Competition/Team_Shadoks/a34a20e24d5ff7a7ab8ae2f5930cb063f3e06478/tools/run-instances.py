#!/usr/bin/env python3
"""
Run several instances with real parameters
"""

import os
import multiprocessing
import json
from datetime import datetime

easy_instances = [
	# "city.domain/paris_200.json",
	"random.domain/random_20.json",
	# "warehouse.domain/sortation_large_400.json",
	# "game.domain/brc202d_200.json",
	# "warehouse.domain/warehouse_large_200.json",
]

instances = [
	# "city.domain/paris_1500.json",
	# "city.domain/paris_3000.json",
	# "random.domain/random_200.json",
	"random.domain/random_100.json",
	# "random.domain/random_400.json",
	# "warehouse.domain/sortation_large_10000.json",
	# "random.domain/random_600.json",
	# "random.domain/random_800.json",
	# "game.domain/brc202d_6500.json",
	# "warehouse.domain/warehouse_large_10000.json",
]

def run(instances, t, steps):
	"""Run the instances in sequential order."""
	for ins in instances:
		fn = ins.split('/')[1].split('.')[0]
		timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
		# command = f"valgrind --tool=callgrind ./build/lifelong --simulationTime {steps} --planTimeLimit {t} --preprocessTimeLimit 1800 --fileStoragePath assets --inputFile example_problems/{ins} --logFile errors.log --output build/{fn}.st{steps}-{timestamp}.sol.json"
		command = f"./build/lifelong --simulationTime {steps} --planTimeLimit {t} --preprocessTimeLimit 1800 --fileStoragePath assets --inputFile example_problems/{ins} --logFile errors.log --output build/{fn}.st{steps}-{timestamp}.sol.json"
		print(command)
		os.system(command)
		os.system('echo; echo "====="; echo')


def run_one(instance):
	fn = instance.split('/')[1].split('.')[0]
	os.system(f"./build/lifelong --simulationTime {steps} --planTimeLimit {t} --fileStoragePath assets --inputFile example_problems/{instance} --logFile test.txt --output build/{fn}.st{steps}.sol.json > log/log-{fn}-{steps}.txt")

def run_parallel(instances, t, steps):
	"""Run the instances in parallel."""
	pool = multiprocessing.Pool(processes=5)
	pool.map(run_one, instances)
	pool.close()
	pool.join()
	print("Computation finished")


def run_with_parameters(instances, t, steps):
	parameters_list = []
	# parameters_list.append({'alpha': .5, 'delta_alpha': 0, 'beta': 1.8, 'delta_beta': -.1, 'enqueue_bound': 60, 'delta_mu': 0})
	for v in [-.2]:
		parameters_list.append({'alpha': .5, 'delta_alpha': -.1, 'beta': 1.4, 'delta_beta': -.1, 'enqueue_bound': 50, 'delta_mu': 0, 'replan': 25})
	results = []
	for instance in instances:
		fn = instance.split('/')[1].split('.')[0]
		for parameters in parameters_list:
			# Write the parameters
			with open("build/parameters.json", "w") as file:
				file.write(json.dumps(parameters))
			datetime_string = datetime.now().strftime("%Y%m%d_%H%M%S")
			out_fn = f"build/{fn}.st{steps}-{datetime_string}.sol.json"
			# Run the planner
			os.system(f"./build/lifelong --simulationTime {steps} --planTimeLimit {t} --preprocessTimeLimit 1800 --fileStoragePath assets --inputFile example_problems/{instance} --logFile errors.log --output {out_fn}")
			os.system('echo; echo "====="; echo')
			# Get the results
			cur_result = {'parameters': parameters, 'instance': instance, 'filename': out_fn, 'steps': steps, 'tasks': 0, 'agents': 0}
			with open(out_fn, "r") as file:
				data = json.load(file)
				cur_result['tasks'] = data['numTaskFinished']
				cur_result['agents'] = data['teamSize']
				cur_result['fast_mover'] = data['AllValid']
			results.append(cur_result)
			with open("results.json", "w") as file:
				json.dump(results, file, indent=4)
			


if __name__ == "__main__":
	t = 1 # number of seconds per timestep
	steps = 100 # number of steps to compute

	# run(instances, t, steps)
	# run_parallel(instances, t, steps)
	run_with_parameters(instances, t, steps)
