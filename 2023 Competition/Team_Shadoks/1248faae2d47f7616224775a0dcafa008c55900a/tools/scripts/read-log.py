#!/usr/bin/env python3
"""
Script for reading the log and extracting useful information
"""

import json
import random
import sys
import statistics


def read_log(fn):
	"""Read the data from the log"""
	with open(fn, 'r') as file:
		lines = [line.rstrip() for line in file]
	data = []
	cur_solution = {'agents': 0, 'map': "", 'barriers': False, 'algorithm': "", 'planned': [], 'times': [], 'tasks': [], 'success': []}
	for line in lines:
		if "Planning" in line:
			cur_solution['agents'] = int(line.split()[1])
			cur_solution['map'] = line.split()[6]
		if "Barriers read" in line:
			cur_solution['barriers'] = True
		if "Planner" in line:
			cur_solution['algorithm'] = ''.join(line.rsplit(' ', 1)[0])
		if "Time:" in line:
			cur_solution['times'].append(float(line.split()[2]))
		# if "agents planned" in line:
			# cur_solution['planned'].append(int(line.split()[6]))
		if "successful" in line:
			cur_solution['success'].append(int(line.split()[8][:-1]))
		if "tasks:" in line:
			t = line.split()[1][6:-1]
			cur_solution['tasks'].append(int(t))

		if "=====" in line:
			data.append(cur_solution)
			cur_solution = {'agents': 0, 'map': "", 'barriers': False, 'algorithm': "", 'planned': [], 'times': [], 'tasks': [], 'success': []}
	if cur_solution['agents'] > 0:
		data.append(cur_solution)
	return data

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
		if solution['barriers']:
			print(f"{solution['algorithm']} with barriers")
		else:
			print(f"{solution['algorithm']} with no barriers")
		if solution['planned']:
			print(f"Number of planned agents ({len(solution['planned'])})")
			print(f"\taverage: {statistics.mean(solution['planned']):.2f}")
			print(f"\tstandard deviation: {statistics.stdev(solution['planned']):.2f}")
			print(f"\tmin: {min(solution['planned'])}, max: {max(solution['planned'])}")
		if solution['success']:
			print(f"Number of succesfully planned agents ({len(solution['success'])})")
			print(f"\taverage: {statistics.mean(solution['success']):.2f}")
			print(f"\tstandard deviation: {statistics.stdev(solution['success']):.2f}")
			print(f"\tmin: {min(solution['success'])}, max: {max(solution['success'])}")
		if solution['times']:
			print(f"Time spent in each timestep ({len(solution['times'])})")
			print(f"\taverage: {statistics.mean(solution['times']):.6f}")
			print(f"\tstandard deviation: {statistics.stdev(solution['times']):.6f}")
			print(f"\tmin: {min(solution['times'])}, max: {max(solution['times'])}")
		print(f"Tasks: {resume(solution['tasks'])}")
		print("")





if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("Usage: python3 real-log.py <filename>")
	else:
		for arg in sys.argv[1:]:
			filename = arg
			print_stats(filename)