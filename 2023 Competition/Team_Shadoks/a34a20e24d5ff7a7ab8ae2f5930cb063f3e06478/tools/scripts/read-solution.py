#!/usr/bin/env python3
"""
Script for reading a solution file and finding weak points.
"""

import json
import math
import sys
# import itertools

import networkx as nx

class Instance:
	"""An instance map, possibly with barriers"""
	def __init__(self, fn):
		"""Read the map from a file."""
		self.width = 0
		self.height = 0
		self.pixels = []
		self.barriers = {"E": [], "S": [], "W": [], "N": []}
		with open(fn) as f:
			lines = [line.rstrip() for line in f]
		self.height = int(lines[1].split()[1])
		self.width = int(lines[2].split()[1])
		for line in lines[4:]:
			for char in line:
				if char in ['@', 'T']:
					self.pixels.append(False) # an obstacle
				else:
					self.pixels.append(True) # a pixel
	
	def _read_barriers(self, fn):
		if fn is not None:
			with open(fn, 'r') as json_file:
				self.barriers = json.load(json_file)
	
	def make_graph(self):
		"""
		Build the directed graph. Note that I put the arrows in the opposite direction 
		so that we can compute distances 
		"""
		edges = []
		for loc in range(self.width*self.height):
			coord = {"x": loc % self.width, "y": loc // self.width}
			if self.pixels[loc]:
				# Add rotations
				for i in range(0, 4):
					edges.append((4*loc + i, 4*loc + (i+1) % 4))
					edges.append((4*loc + (i+1) % 4, 4*loc + i))
				# Add moves
				next = loc + 1
				if loc not in self.barriers['E'] and coord['x'] + 1 < self.width and self.pixels[next]:
					edges.append((4*next + 0, 4*loc + 0))
				next = loc + self.width
				if loc not in self.barriers['S'] and coord['y'] + 1 < self.height and self.pixels[next]:
					edges.append((4*next + 1, 4*loc + 1))
				next = loc - 1
				if loc not in self.barriers['W'] and coord['x'] - 1 >= 0 and self.pixels[next]:
					edges.append((4*next + 2, 4*loc + 2))
				next = loc - self.width
				if loc not in self.barriers['N'] and coord['y'] - 1 >= 0 and self.pixels[next]:
					edges.append((4*next + 3, 4*loc + 3))
		self.digraph = nx.DiGraph()
		self.digraph.add_edges_from(edges)

	def location(self, x, y):
		return self.width*y + x
	
	def position(self, x, y, dir):
		return 4*self.location(x, y) + dir


	def distance_map(self, end):
		"""Compute the distance map from each position (location, direction) to a given location"""
		dist = [nx.shortest_path_length(self.digraph, 4*end + i) for i in range(4)]
		distance_map = {node: min([d[node] for d in dist]) for node in self.digraph.nodes}
		return distance_map




class Solution:
	def __init__(self, fn):
		"""Read a solution file"""
		self.agent = []
		self.path = []
		self.task = []
		with open(fn, 'r') as json_file:
			data = json.load(json_file)
		self.teamSize = data["teamSize"]
		self._make_paths(data)
		self._make_tasks(data)
		print(f"-- Solution file {fn} read")

	def _make_paths(self, data):
		for a in range(self.teamSize):
			cur_path = []
			cur_pos = (data["start"][a][1], data["start"][a][0], 0)
			actions = data["actualPaths"][a].split(',')
			cur_path.append(cur_pos)
			for action in actions:
				cur_pos = self.apply_action(cur_pos, action)
				cur_path.append(cur_pos)
			self.path.append(cur_path)
	
	def _make_tasks(self, data):
		for a, agent_events in enumerate(data["events"]):
			tasks = []
			for event in agent_events:
				if event[2] == "assigned":
					cur_task = {
						"t": event[1],
						"x": data["tasks"][event[0]][2],
						"y": data["tasks"][event[0]][1],
					}
					tasks.append(cur_task)
			self.task.append(tasks)
	
	def task(self, agent, time):
		"""Return the task of an agent at a given time."""
		cur_task = self.task[agent][0]
		for task in self.task[agent]:
			if task["t"] < time:
				cur_task = task
			else:
				return cur_task
		return cur_task
	

	def apply_action(self, pos, action):
		if action == "R":
			return (pos[0], pos[1], (pos[2] + 1) % 4)
		elif action == "C":
			return (pos[0], pos[1], (pos[2] + 3) % 4)
		elif action == "F":
			if pos[2] == 0:
				return (pos[0] + 1, pos[1], pos[2])
			elif pos[2] == 1:
				return (pos[0], pos[1] + 1, pos[2])
			elif pos[2] == 2:
				return (pos[0] - 1, pos[1], pos[2])
			elif pos[2] == 3:
				return (pos[0], pos[1] - 1, pos[2])
		return pos


	def speed(self, instance, agent):
		"""For each task of the agent during the solution, make a list with its distance at each timestep."""
		profiles = []
		for task in self.task[agent]:
			cur_profile = {"agent": agent, "start": task["t"], "distance": []}
			end = instance.location(task["x"], task["y"])
			dist = instance.distance_map(end)
			for t in range(task["t"], len(self.path[agent])):
				pos = self.path[agent][t]
				d = dist[instance.position(pos[0], pos[1], pos[2])]
				cur_profile["distance"].append(d)
				if d == 0:
					break
			profiles.append(cur_profile)
		return profiles

	def analyse(self, profile):
		"""Analyse the profile of an agent"""
		bad_steps = []
		count = 0
		for i in range(len(profile["distance"]) - 1):
			if profile["distance"][i] <= profile["distance"][i+1] and profile["distance"][i] > 0 :
				bad_steps.append(profile["start"] + i)
				count += (profile["distance"][i+1] - profile["distance"][i] + 1)
		profile["distance_sum"] = count
		profile["bad_steps"] = bad_steps
		if profile["distance"][-1] == 0:
			profile["speed"] = (profile["distance"][0] - profile["distance"][-1]) / (len(profile["distance"]) - 1)
		return profile

	def problems(self, instance, agent):
		"""Detect bad actions in the path of an agent."""
		profiles = self.speed(instance, agent)
		for profile in profiles:
			bad_steps = []
			count = 0
			for i in range(len(profile["distance"]) - 1):
				if profile["distance"][i] <= profile["distance"][i+1] and profile["distance"][i] > 0 :
					bad_steps.append(profile["start"] + i)
					count += (profile["distance"][i+1] - profile["distance"][i] + 1)
			profile["distance_sum"] = count
			profile["agent"] = agent
			profile["bad_steps"] = bad_steps
			profile["speed"] = (profile["distance"][0] - profile["distance"][-1]) / len(profile["distance"])
		return profiles
		# print(f"Bad steps of a{agent} (value={count}, count= {len(bad_steps)}): " + " ".join(f"{t}({profile[t]})" for t in bad_steps))

	def ending(seld, profile, dist):
		"""Extract the end of the path, when the agent is close to its task"""
		last_index = None
		for i in range(len(profile["distance"])):
			if profile["distance"][i] == dist:
				last_index = i
		if last_index is None:
			return None
		profile["distance"] = profile["distance"][last_index:]
		return profile

def which_map(fn):
	if "random" in fn:
		return "../../example_problems/random.domain/maps/random-32-32-20.map"
	if "sortation" in fn:
		return "sortation_large.map"
	if "warehouse" in fn:
		return "warehouse_large.map"
	print("No instance known")
	return None


def detect_slow_agents(fn, n_agents):
	"""For each pair of agent and task, see how fast the agent finishes its task."""
	print("Measuring the speed for each agent. See the results in the file solution-analysis.txt")
	sol = Solution(fn)
	ins = Instance(which_map(fn))
	ins.make_graph()
	number_agents = min(n_agents, sol.teamSize)
	profiles = []
	for a in range(number_agents):
		print(f"Measuring a{a}", end="\r", flush=True)
		for profile in sol.speed(ins, a):
			if profile['distance'][-1] == 0: # ignore unfinished tasks
				profiles.append(sol.analyse(profile))
		write_to_file(fn, profiles)

def finish_line(fn, dist):
	sol = Solution(fn)
	ins = Instance(which_map(fn))
	ins.make_graph()
	number_agents = min(100, sol.teamSize)
	profiles = []
	for a in range(number_agents):
		print(f"Measuring a{a}", end="\r", flush=True)
		for profile in sol.speed(ins, a):
			subprofile = sol.ending(profile, dist)
			if subprofile is not None:
				profiles.append(sol.analyse(subprofile))
		write_to_file(profiles)
	profiles.sort(key=lambda x: x["speed"])
	for profile in profiles:
		print(f"- a{profile['agent']}: speed={profile['speed']:.3f}, start={profile['start']} distance={profile['distance_sum']} bad_steps={profile['bad_steps']}")


def write_to_file(fn, profiles):
	profiles.sort(key=lambda x: x["speed"])
	with open("solution-analysis.txt", 'w') as file:
		file.write(f"Solution file {fn}\n")
		file.write("---\n")
		for profile in sorted(profiles, key=lambda x: x["speed"]):
			file.write(f"- a{profile['agent']}: speed={profile['speed']:.3f}, start={profile['start']} distance={profile['distance_sum']} bad_steps={profile['bad_steps']}\n")
		

def poor_decisions(fn):
	"""
	Show the bad decisions made by each agent in a solution.
	These are the steps where they do not advance towards their task.
	"""
	sol = Solution(fn)
	ins = Instance(which_map(fn))
	ins.make_graph()
	number_agents = min(n_agents, sol.teamSize)
	profiles = []
	for a in range(number_agents):
		profiles.extend(sol.problems(ins, a))
	profiles.sort(key=lambda x: len(x["speed"]))
	for profile in profiles:
		print(f"- a{profile['agent']}: speed={profile['speed']}, distance={profile['distance_sum']} bad_steps={profile['bad_steps']}")
		# print(f"- a{profile['agent']}: speed={profile['speed']}, distance={profile['distance_sum']} bad_steps: " + " ".join(f"{t}({profile[t]})" for t in bad_steps))



if __name__ == "__main__":
	if len(sys.argv) != 2:
		print("Usage: python3 real-solution.py <filename>")
	fn_sol = sys.argv[1]
	# fn_sol = "sortation_large_10000.st1000-20231121_134634.sol.json"
	# fn_sol = "random_600-st400-20231118_040538.sol.json"
	# fn_sol = "../../build/" + "random_100.st500-20231128_201755.sol.json"
	detect_slow_agents(fn_sol, 100000)
	# finish_line(fn_sol, 10)

