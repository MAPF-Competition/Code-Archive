"""
Script for computing an approximation of a distance between all pairs of locations.
Indeed, we select a subset of pixels and compute distances between them. Then,
we associate every pixel to its closest selected pixel. Hence, we obtain an
approximation of the distance.
"""

import json
import random
import copy
from collections import deque
import math

import gurobipy as gp
from gurobipy import GRB

random.seed(0)

def read_map(fn):
	"""Read a file in .map format."""
	map = {"filename": fn, "width": 0, "height": 0, "obstacles": [], "location_at_index": []}
	with open(fn) as f:
		lines = [line.rstrip() for line in f]
	map["height"] = int(lines[1].split()[1])
	map["width"] = int(lines[2].split()[1])
	location = 0
	for line in lines[4:]:
		for char in line:
			if char in ['@', 'T']:
				map["obstacles"].append(True) # an obstacle
			else:
				map["obstacles"].append(False) # not an obstacle
				map["location_at_index"].append(location)
			location += 1
	return map

def location_to_coords(map, location):
	"""Convert a location (index) into coordinates"""
	return {
		"x": location % map["width"],
		"y": location // map["width"]
	}

def coords_to_location(x, y, map):
	if x < 0 or y < 0:
		return None
	if x >= map['width'] or y >= map['height']:
		return None
	return y * map['width'] + x

def select_locations(map, n):
	"""Get `n` random locations in the map"""
	locations = copy.copy(map["location_at_index"])
	random.shuffle(locations)
	return locations[:n]

def compute_distances(map, locations):
	"""Compute all the pairwise distances between the pixels in `locations`."""
	n = len(locations)
	distance = [[-1 for i in range(n)] for j in range(n)]
	for i in range(len(locations)):
		dist = distance_map(map, locations[i])
		for j in range(len(locations)):
			distance[i][j] = dist[locations[j]]
	return distance

def distance_map(map, location):
	"""Compute the distance map of `location`."""
	distance = [-1 for i in range(map["width"]*map["height"])]
	queue = deque()
	queue.append(location)
	distance[location] = 0
	while queue:
		cur_loc = queue.popleft()
		for next in neigbors(map, cur_loc):
			if distance[next] < 0:
				distance[next] = distance[cur_loc] + 1
				queue.append(next)
	return distance

def neigbors(map, location):
	p = location_to_coords(map, location)
	neighs = []
	if p['x'] + 1 < map["width"]:
		neighs.append(location + 1)
	if p['y'] + 1 < map["height"]:
		neighs.append(location + map["width"])
	if p['x'] - 1 >= 0:
		neighs.append(location - 1)
	if p['y'] - 1 >= 0:
		neighs.append(location - map["width"])
	return [x for x in neighs if not map['obstacles'][x]]


def label(map, locations):
	"""
	Label the pixels of a map given a set of locations.
	We also compute the distances from each pixel to its closest location.
	"""
	label = [-1 for i in range(map["width"]*map["height"])]
	dist = [-1 for i in range(map["width"]*map["height"])]
	queue = deque()
	for i, location in enumerate(locations):
		queue.append(location)
		label[location] = i
		dist[location] = 0
	while queue:
		cur_loc = queue.popleft()
		for next in neigbors(map, cur_loc):
			if label[next] < 0:
				label[next] = label[cur_loc]
				queue.append(next)
				dist[next] = dist[cur_loc] + 1
	# print(f"Max distance: {max(dist)}")
	max_dist = max(dist)
	worst_location = dist.index(max_dist)
	return {
		"label": label, 
		"approximation": max_dist,
		"worst_location": worst_location
	}

def simulated_annealing(map, n_sites, max_iterations):
	print(f"Simulated annealing with {max_iterations} iterations")
	# initial solution
	cur_sol = select_locations(map, n_sites)
	cur_value = label(map, cur_sol)["approximation"]
	best = {
		"solution": cur_sol,
		"value": cur_value
	}
	print(f"Values: {best['value']}", end=" ")
	# iterate to find a better solution
	for i in range(max_iterations):
		# next_sol = any_neighbor(cur_sol, map)
		if i % 3 == 0:
			next_sol = any_neighbor(cur_sol, map)
		else:
			next_sol = smart_neighbor(cur_sol, map)
		next_value = label(map, next_sol)["approximation"]
		# Calculate the change in distance
		delta_distance = next_value - cur_value
		temperature = 1 - i/max_iterations
		if delta_distance < 0 or random.random() < math.exp(-delta_distance / temperature):
			cur_sol = next_sol
			cur_value = next_value
			if cur_value < best["value"]:
				best["solution"] = copy.copy(cur_sol)
				best["value"] = cur_value
				print(f"{cur_value} ({i})", end=" ", flush=True)
				print_solution(map, cur_sol)
	print()
	return best["solution"]

def any_neighbor(sites, map):
	next = sites[:]
	n_pixels = len(map["location_at_index"])
	i = random.randrange(0, len(sites))
	j = random.randrange(0, n_pixels)
	next[i] = map["location_at_index"][j]
	return next

def smart_neighbor(sites, map):
	labeling = label(map, sites)
	i = labeling['label'][labeling['worst_location']]
	p = location_to_coords(map, sites[i])
	while True:
		dx = random.randint(-3, 3)
		dy = random.randint(-3, 3)
		next_loc = coords_to_location(p['x'] + dx, p['y'] + dy, map)
		if next_loc is not None and not map['obstacles'][next_loc]:
			break
	next = sites[:]
	next[i] = next_loc
	return next


def print_solution(map, locations):
	"""Show the locations as an instance"""
	n = len(locations)
	data = {
		"teamSize": n,
		"start": [],
		"makespan": 0,
		"plannerPaths": ["" for _ in range(n)],
		"tasks": [[[i, 0, 0]] for i in range(n)],
		"events": [[[i, 0, "assigned"]] for i in range(n)], 
		"errors": []
	}
	for loc in locations:
		p = location_to_coords(map, loc)
		data['start'].append([p['y'], p['x'], "E"])
	with open("sites.json", "w") as json_file:
		json.dump(data, json_file)

def solve(fn, n_sites=256):
	"""Given a number of sites, try to put them everywhere."""
	map = read_map(fn)
	# Show a few random solutions
	for i in range(5):
		locations = select_locations(map, n_sites)
		labeling = label(map, locations)
		print(f"Value: {labeling['approximation']}")
	locations = simulated_annealing(map, n_sites, 100000)
	labeling = label(map, locations)
	distance = compute_distances(map, locations)
	data = {
		"meta": {"filename": fn, "number_sites": n_sites, "approximation": labeling["approximation"]},
		"name": fn,
		"sites": locations,
		"label": labeling["label"],
		"distance": distance
	}
	print_solution(map, locations)
	print(f"Worst location: {labeling['worst_location']}")

	out_fn = fn.split(".")[0] + "-dist.json"
	with open(out_fn, "w") as json_file:
		json.dump(data, json_file)
	

def ilp_solver(fn, max_dist):
	"""
	Solving the covering problem.
	We search as few sites as possible such that location in the map 
	is at most at distance `max_dist` from a site.
	"""
	map = read_map(fn)
	model = gp.Model()
	model.Params.TimeLimit = 180.0
	x = {} # The i-th pixel is a site
	for loc in map["location_at_index"]:
		x[loc] = model.addVar(vtype=GRB.BINARY)
	for i, loc in enumerate(map["location_at_index"]):
		if i%100 == 0:
			print(f"Constraint loc: {i}/{len(map['location_at_index'])}", end="\r", flush=True)
		pixels = k_neighborhood(map, loc, max_dist)
		model.addConstr(sum(x[j] for j in pixels) >= 1)
	model.setObjective(sum(x[loc] for loc in map["location_at_index"]), GRB.MINIMIZE)
	model.optimize()
	# if model.Status == GRB.TIME_LIMIT:
	# 	return False
	# if model.Status != GRB.OPTIMAL:
	# 	return False
	print(f"Number of sites: {model.objVal}")
	# Get the solution
	sites = []
	for j, var in x.items():
		if var.X > .5:
			sites.append(j)
			assert not map['obstacles'][j]
	write_sites(map, sites, max_dist)
	# write_sites_details(map, sites, max_dist)


def k_neighborhood(map, location, max_distance):
	"""Get locations at a given distance from a location"""
	neighs = []
	dist = [-1 for i in range(map["width"]*map["height"])]
	queue = deque()
	queue.append(location)
	dist[location] = 0
	neighs.append(location)
	while queue:
		cur_loc = queue.popleft()
		for next in neigbors(map, cur_loc):
			if dist[next] < 0:
				dist[next] = dist[cur_loc] + 1
				if dist[next] <= max_distance:
					neighs.append(next)
					queue.append(next)
	# print(f"max_dist = {max(dist)} = param {max_distance}. Size: {len(neighs)}")
	for loc in neighs:
		assert not map['obstacles'][loc]
	return neighs


def conflict_optimizer(fn, max_dist, objective):
	"""
	Solve the minimum subset covering with conflict optimization.
	This is very slow and does not work.
	"""
	def covered_neighborhood(sites, neighborhood):
		"""Return true if the neighborhood is covered by the sites"""
		for loc in neighborhood:
			if loc in sites:
				return True
		return False
	
	def best_move(uncovered_location):
		best_exchange = {'add': None, 'remove': None, 'value': None, 'uncovered': None}
		if covered_neighborhood(sites, neighborhoods[uncovered_location]):
			return best_exchange
		for loc_add in neighborhoods[uncovered_location]:
			for loc_rem in sites:
				cur_exchange = {'add': loc_add, 'remove': loc_rem, 'value': 0, 'uncovered': []}
				cur_sites = set(l for l in sites)
				cur_sites.remove(loc_rem)
				cur_sites.add(loc_add)
				for loc, neighborhood in neighborhoods.items():
					if not covered_neighborhood(cur_sites, neighborhood) and covered_neighborhood(sites, neighborhood):
						cur_exchange['uncovered'].append(loc)
						cur_exchange['value'] += (count[loc] + 1)**2
				if best_exchange['value'] is None or cur_exchange['value'] < best_exchange['value']:
					best_exchange = cur_exchange
					if best_exchange['value'] == 0:
						return best_exchange
		return best_exchange

	map = read_map(fn)
	candidates = [loc for loc in map["location_at_index"]]
	neighborhoods = {loc: k_neighborhood(map, loc, max_dist) for loc in candidates}
	sites = set(candidates[:objective])
	queue = deque()
	count = {loc: 0 for loc in neighborhoods}
	for loc, neighborhood in neighborhoods.items():
		if not covered_neighborhood(sites, neighborhood):
			queue.append(loc)
			count[loc] += 1
	while queue:
		cur_loc = queue.popleft()
		exchange = best_move(sites, neighborhoods, cur_loc, count)
		sites.remove(exchange['remove'])
		sites.add(exchange['add'])
		for loc in exchange['uncovered']:
			queue.append(loc)
	write_sites(map, list(sites), max_dist)

def greedy(fn, max_dist):
	map = read_map(fn)
	sites = {loc: True for loc in map["location_at_index"]}
	neighborhoods = {}
	for i, loc in enumerate(sites):
		if i%1000 == 0:
			print(f"Constraint loc: {i}/{len(sites)}", end="\r", flush=True)
		neighborhoods[loc] = set(k_neighborhood(map, loc, max_dist))
	permutation = [loc for loc in sites]
	random.shuffle(permutation)
	for i, site in enumerate(permutation):
		if i%1000 == 0:
			print(f"Removing sites: {i}/{len(sites)}", end="\r", flush=True)
		necessary = False
		for loc, neighborhood in neighborhoods.items():
			if site in neighborhood and sum(1 for j in neighborhood if sites[j]) == 1:
				necessary = True
		if not necessary:
			sites[site] = False
	write_sites(map, [j for j in sites if sites[j]], max_dist)






def write_sites_details(map, sites, max_dist):
	"""Write the sites with a labeling and a distance matrix."""
	labeling = label(map, sites)
	distance = compute_distances(map, sites)
	data = {
		"meta": {"filename": map['filename'], "max_dist": max_dist, "approximation": labeling["approximation"]},
		"name": fn,
		"sites": sites,
		"label": labeling["label"],
		"distance": distance
	}
	print_solution(map, sites)
	print(f"Worst location: {labeling['worst_location']}")
	out_fn = map['filename'].split(".")[0] + "-dist.json"
	with open(out_fn, "w") as json_file:
		json.dump(data, json_file)

def write_sites(map, sites, max_dist):
	"""Write only the sites."""
	data = {
		"meta": {"filename": map['filename'], "max_dist": max_dist, "nb_sites": len(sites)},
		"sites": sites,
	}
	print_solution(map, sites)
	out_fn = map['filename'].split(".")[0] + "-dist.json"
	with open(out_fn, "w") as json_file:
		json.dump(data, json_file)


if __name__ == "__main__":
	fn = "random-32-32-20.map"
	# fn = "Paris_1_256.map"
	# fn = "brc202d.map"
	# fn = "warehouse_large.map"
	fn = "sortation_large.map"
	
	# solve(fn, 256)
	
	# ilp_solver("random-32-32-20.map", 2)
	# ilp_solver(fn, 10)
	greedy(fn, 10)
	# conflict_optimizer(fn, 5, 400)
