#!/usr/bin/env python3
"""
Script for reading the log and extracting useful information
"""

import json
import random
import sys
import statistics

class Log:
	"""A class for reading the logging information of a Planner."""
	def __init__(self, fn):
		self.fn = fn
		self.map = ""
		self.num_agents = 0
		self.algorithm = ""
		self.tasks = []
		self.fields = {
			'planned': 'Number of planned agents',
			'success': 'Number of succesfully planned agents',
			'times': 'Time spent in each timestep',
			'path': 'Length of the paths computed',
			'rounds': 'Number of rounds',
		}
		self.data = {key: [] for key in self.fields}
	
	def read_line(self, line):
		"""Read a line"""
		if "Planning" in line:
			self.num_agents = int(line.split()[1])
			self.map = line.split()[6]
		if "initialized" in line:
			self.algorithm = ''.join(line.rsplit(' ', 1)[0])
		if "tasks:" in line:
			self.tasks.append(int(line.split()[1][6:-1]))
		if "Time:" in line:
			self.data['times'].append(float(line.split()[2]))
		if "PlannerLazy" in self.algorithm:
			if "agents planned" in line: 
				self.data['planned'].append(int(line.split()[1]))
				self.data['path'].append(int(line.split()[6]))
		if "PlannerLong" in self.algorithm:
			if "rounds:" in line:
				self.data['rounds'].append(int(line.split()[5]))
		if "successful" in line:
			self.data['success'].append(int(line.split()[8][:-1]))
	
	def print(self):
		print(f"-- Statistics of {self.fn} --")
		print(f"Map {self.map} with {self.num_agents} agents")
		print(self.algorithm)
		for key, message in self.fields.items():
			if self.data[key]:
				print(f"{message} ({len(self.data[key])})")
				print(f"\taverage: {statistics.mean(self.data[key]):.6f}")
				print(f"\tstandard deviation: {statistics.stdev(self.data[key]):.6f}")
				print(f"\tmin: {min(self.data[key])}, max: {max(self.data[key])}")
		self.print_tasks()
		print("")

	def print_tasks(self):
		resume = []
		for i in range(1, 11):
			resume.append(self.tasks[i * len(self.tasks) // 10 - 1])
		print(f"Tasks: {resume}")
		


def read_log(fn):
	"""Read the data from the log"""
	with open(fn, 'r') as file:
		lines = [line.rstrip() for line in file]
	logs = []
	cur_log = Log(fn)
	for line in lines:
		cur_log.read_line(line)
		if "=====" in line:
			logs.append(cur_log)
			cur_log = Log(fn)
	if cur_log.num_agents > 0:
		logs.append(cur_log)
	for log in logs:
		log.print()

def resume(values):
	less_values = []
	for i in range(1, 11):
		less_values.append(values[i * len(values) // 10 - 1])
	return less_values


def print_stats(fn):
	"""Compute different statistics of a solution."""
	data = read_log(fn)	
	for solution in data:
		print(f"-- Statistics of {fn} --")
		print(f"Map {solution['map']} with {solution['agents']} agents")
		print(solution['algorithm'])
		for key, message in [
			('planned', 'Number of planned agents'), ('success', 'Number of succesfully planned agents'), 
			('times', 'Time spent in each timestep'), ('path', 'Length of the paths computed'),
			('rounds', 'Number of rounds')]:
			if solution[key]:
				print(f"{message} ({len(solution[key])})")
				print(f"\taverage: {statistics.mean(solution[key]):.6f}")
				print(f"\tstandard deviation: {statistics.stdev(solution[key]):.6f}")
				print(f"\tmin: {min(solution[key])}, max: {max(solution[key])}")
		print(f"Tasks: {resume(solution['tasks'])}")
		print("")





if __name__ == "__main__":
	# print_stats("../../log.txt")
	
	if len(sys.argv) < 2:
		print("Usage: python3 real-log.py <filename>")
	else:
		for arg in sys.argv[1:]:
			filename = arg
			read_log(filename)
			# print_stats(filename)
