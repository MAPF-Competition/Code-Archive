#!/usr/bin/env python3
"""
Run several instances with real parameters
"""

import os
import json
import math
import random
import copy
import uuid
from datetime import datetime

class Search:
	"""An instance map, possibly with barriers"""
	def __init__(self, instance, t, steps, parameters):
		"""Read the map from a file."""
		self.instance = instance
		self.time = t
		self.steps = steps
		self.parameters = parameters
		self.log_fn = "log-" + datetime.now().strftime("%Y%m%d-%H%M%S") + ".txt"

	def solution_filename(self):
		"""Make a file name for a solution."""
		fn = self.instance.split('/')[1].split('.')[0]
		timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
		unique_id = uuid.uuid4()
		return f"build/{fn}.st{steps}-{timestamp}-{unique_id}.sol.json"

	def run(self, assignment):
		"""Run the algorithm with the parameters and return the number of tasks."""
		with open("build/parameters.json", "w") as file:
			file.write(json.dumps(assignment))
		out_fn = self.solution_filename()
		# Run the planner
		os.system(f"./build/lifelong --simulationTime {self.steps} --planTimeLimit {self.time} --preprocessTimeLimit 1800 --fileStoragePath assets --inputFile example_problems/{self.instance} --logFile errors.log --output {out_fn} >> {self.log_fn}")
		# Get the results
		cur_result = {
			'instance': self.instance, 'steps': self.steps, 'timeLimit': self.time,
			'assignment': copy.deepcopy(assignment), 
			'filename': out_fn, 
			'tasks': 0, 'agents': 0
		}
		with open(out_fn, "r") as file:
			data = json.load(file)
			cur_result['tasks'] = data['numTaskFinished']
			cur_result['agents'] = data['teamSize']
			cur_result['fast_mover'] = (data['AllValid'] == "Yes")
		os.system(f"rm {out_fn}")
		return cur_result
	

	def random_assignment(self):
		"""Make a random assignment of parameters."""
		assignment = {}
		for param in self.parameters:
			key = param['key']
			n = random.randint(param['min'] / param['step'], param['max'] / param['step'])
			assignment[key] = param['step'] * n
		# print(assignment)
		return assignment
	
	def perturbate(self, assignment):
		"""Change the values of the parameters."""
		next = {}
		for param in self.parameters:
			key = param['key']
			delta = random.choice([-1, 0, 1])
			value = assignment[key] + delta * param['step']
			value = max(value, param['min'])
			value = min(value, param['max'])
			next[key] = value
		# print(next)
		return next
	
	def simulated_annealing(self, generations=1000):
		"""Simulated annealing for improving one of the criteria"""
		print(f"Searching parameters for {self.instance} with constraints {self.parameters}")
		best_sol = None
		cur_sol = None
		cur_assign = self.random_assignment()
		cur_sol = self.run(cur_assign)
		best_sol = cur_sol
		print("Starting simulated annealing...")
		for i in range(generations):
			T = 1 - i/generations
			next_assign = self.perturbate(cur_assign)
			next_sol = self.run(next_assign)
			if next_sol['tasks'] < cur_sol['tasks'] or random.random() < math.exp(-1/T):
				cur_sol = next_sol
				if cur_sol['tasks'] > best_sol['tasks']:
					best_sol = cur_sol
					print(f"++ {cur_sol['tasks']}* ({i}) {cur_sol}")
				else:
					print(f"-- {cur_sol['tasks']} ({i}) {cur_sol}")
		print(f"Best assignment: {best_sol}")
	

	def greedy(self, nb_rounds=4, depth=10):
		cur_assign = self.random_assignment()
		cur_sol = self.run(cur_assign)
		best_sol = cur_sol
		for round in range(nb_rounds):
			for param in self.parameters:
				print(f"Round #{round}, parameter {param['key']}:")
				cur_asign = copy.deepcopy(best_sol['assignment'])
				for i in range(depth):
					t = i / (depth - 1)
					cur_asign[param['key']] = (1-t) * param['min'] + t * param['max']
					cur_sol = self.run(cur_asign)
					if cur_sol['tasks'] > best_sol['tasks']:
						best_sol = cur_sol
						print(f"-- {cur_sol['tasks']}* ({i}) {cur_sol}")


				


			


if __name__ == "__main__":
	# instance = "city.domain/paris_1500.json"
	# instance = "city.domain/paris_3000.json"
	instance = "random.domain/random_200.json"
	# instance = "random.domain/random_100.json"
	# instance = "random.domain/random_400.json"
	# instance = "warehouse.domain/sortation_large_10000.json"
	# instance = "random.domain/random_600.json"
	# instance = "random.domain/random_800.json"
	# instance = "game.domain/brc202d_6500.json"
	# instance = "warehouse.domain/warehouse_large_10000.json"
	t = 2 # number of seconds per timestep
	steps = 500 # number of steps to compute
	parameters = [
		{'key': 'alpha', 'min': 0.05, 'max': 1.0, 'step': 0.05},
		{'key': 'beta', 'min': 0.5, 'max': 2.0, 'step': 0.1},
		{'key': 'enqueue_bound', 'min': 10, 'max': 200, 'step': 10}
	]

	search = Search(instance, t, steps, parameters)
	search.simulated_annealing(generations=100)
	# search.greedy(nb_rounds=4, depth=10)
