"""
Script for helping me setting the barriers for random.
"""

import json
import copy
import math
import random
from collections import deque

import networkx as nx

def read_map(fn):
	"""Read a file in .map format."""
	map = {"width": 0, "height": 0, "pixels": [], "removed": []}
	with open(fn) as f:
		lines = [line.rstrip() for line in f]
	map["height"] = int(lines[1].split()[1])
	map["width"] = int(lines[2].split()[1])
	for line in lines[4:]:
		for char in line:
			if char in ['@', 'T']:
				map["pixels"].append(False) # an obstacle
			else:
				map["pixels"].append(True) # a pixel
	to_remove = {
		"random-32-32-20.map": [],
		"brc202d.map": [129434,135790,136319,31095,48507,247466,66312,152942,53110,62566,76478],
		"Paris_1_256.map": [65445,964,64361,6889],
		"sortation_large.map": [],
		"warehouse_large.map": [],
		"warehouse_small.map": [],
	}
	for loc in to_remove[fn]:
		map['pixels'][loc] = False
		map['removed'].append(loc)
	remove_exceptions(map)
	remove_endpoints(map)
	# print(f"Removed points: {map['removed']}")
	return map

def make_graph(fn):
	"""Make an (undirected) graph"""
	map = read_map(fn)
	edges = []
	for loc in range(map['width']*map['height']):
		coord = location_to_coords(map, loc)
		if map['pixels'][loc]:
			next_x = loc + 1
			next_y = loc + map['width']
			if coord['x'] + 1 < map['width'] and map['pixels'][next_x]:
				edges.append((loc, next_x))
			if coord['y'] + 1 < map['height'] and map['pixels'][next_y]:
				edges.append((loc, next_y))
	G = nx.Graph()
	G.add_edges_from(edges)
	print(f"The undirected diameter is at least {diameter(G)}")
	return strong_orientation(G)

def make_digraph(fn_map, fn_barriers):
	"""Make a directed graph using the barriers"""
	map = read_map(fn_map)
	with open(fn_barriers, 'r') as json_file:
		barriers = json.load(json_file)
	edges = []
	for loc in range(map['width']*map['height']):
		coord = location_to_coords(map, loc)
		if map['pixels'][loc]:
			next = loc + 1
			if loc not in barriers['E'] and coord['x'] + 1 < map['width'] and map['pixels'][next]:
				edges.append((loc, next))
			next = loc + map['width']
			if loc not in barriers['S'] and coord['y'] + 1 < map['height'] and map['pixels'][next]:
				edges.append((loc, next))
			next = loc - 1
			if loc not in barriers['W'] and coord['x'] - 1 >= 0 and map['pixels'][next]:
				edges.append((loc, next))
			next = loc - map['width']
			if loc not in barriers['N'] and coord['y'] - 1 >= 0 and map['pixels'][next]:
				edges.append((loc, next))
	G = nx.DiGraph()
	G.add_edges_from(edges)
	assert nx.number_strongly_connected_components(G) == 1
	print(f"The graph is strongly connected. Number of nodes: {G.number_of_nodes()}")
	print(f"The directed diameter is at least {diameter(G)}")
	return G

def remove_exceptions(map):
	"""Remove the small connected components"""
	edges = []
	for loc in range(map['width']*map['height']):
		coord = location_to_coords(map, loc)
		if map['pixels'][loc]:
			next_x = loc + 1
			next_y = loc + map['width']
			if coord['x'] + 1 < map['width'] and map['pixels'][next_x]:
				edges.append((loc, next_x))
			if coord['y'] + 1 < map['height'] and map['pixels'][next_y]:
				edges.append((loc, next_y))
	G = nx.Graph()
	G.add_edges_from(edges)
	small_cc = sorted(nx.connected_components(G), key=len, reverse=True)[1:]
	for cc in small_cc:
		for loc in cc:
			map['pixels'][loc] = False
			map['removed'].append(loc)
	print("Trimming done")
	

def remove_endpoints(map):
	stability = False
	removed = []
	while not stability:
		stability = True
		for loc in range(map['width']*map['height']):
			if endpoint(map, loc):
				map['pixels'][loc] = False
				map['removed'].append(loc)
				stability = False
				removed.append(loc)
	# print(removed)

def endpoint(map, location):
	"""Is the location an endpoint (only one neighbor)"""
	if not map['pixels'][location]:
		return False
	p = location_to_coords(map, location)
	count = 0
	if p['x']+1 < map['width'] and map['pixels'][location + 1]:
		count += 1
	if p['y']+1 < map['height'] and map['pixels'][location + map['width']]:
		count += 1
	if p['x']-1 >= 0 and map['pixels'][location - 1]:
		count += 1
	if p['y']-1 >= 0 and map['pixels'][location - map['width']]:
		count += 1
	return count == 1
	
	

def strong_orientation(graph):
	"""
	Set a direction in every edge to have a strongly connected digraph.
	We build a spanning tree and then orient the rest of the edges from the deepest to the other one.
	"""
	G = nx.DiGraph()
	max_node = max(list(graph.nodes))
	depth = [None for _ in range(max_node + 1)]
	root = list(graph.nodes)[0]
	depth[root] = 0
	for u, v in nx.dfs_edges(graph, root):
		depth[v] = depth[u] + 1
		G.add_edge(u, v)
	for u, v in graph.edges():
		if depth[u] > depth[v] and not G.has_edge(v, u):
			G.add_edge(u, v)
		if depth[v] > depth[u] and not G.has_edge(u, v):
			G.add_edge(v, u)
	# print_barriers(G)
	if nx.number_strongly_connected_components(G) != 1:
		print("Error: the graph is not strongly connected: there are some narrow corridors")
		for c in sorted(nx.strongly_connected_components(G), key=len, reverse=True):
			print(f"There is a component of size {len(c)} with vertex {list(c)[0]}")
		exit()
	assert nx.number_strongly_connected_components(G) == 1
	print(f"The graph is strongly connected. Number of nodes: {G.number_of_nodes()}")
	return G


def perturbate(G):
	cycle = random_cycle(G)
	for u, v in cycle:
		assert G.has_edge(u, v)
		G.remove_edge(u, v)
		G.add_edge(v, u)
	# assert nx.number_strongly_connected_components(G) == 1
	# print(f"Strongly CC: {nx.number_strongly_connected_components(G)}")


def random_cycle(G):
	while True:
		root = random.choice(list(G.nodes()))
		current_node = root
		cycle = []
		visited = set()
		while True:
			unvisited_neighbors = [n for n in G.neighbors(current_node) if n not in visited]
			if unvisited_neighbors:
				next_node = random.choice(unvisited_neighbors)
				cycle.append((current_node, next_node))
				visited.add(next_node)
				current_node = next_node
			else:
				break
			if current_node == root:
				return cycle


def strong_orientation_r(graph, root):
	"""
	Set a direction in every edge to have a strongly connected digraph.
	We build a spanning tree and then orient the rest of the edges from the deepest to the other one.
	"""
	max_node = max(list(graph.nodes))
	depth = [None for _ in range(max_node + 1)]
	eccentricity = 0
	depth[root] = 0
	for u, v in nx.bfs_edges(graph, root):
		depth[v] = depth[u] + 1
		eccentricity = max(eccentricity, depth[v])
	print(f"Max distance from {root}: {eccentricity}")
	return eccentricity


def print_barriers(G, fn="test-barriers.json"):
	barriers = {'meta': approx_diameter(G), 'E': [],'S': [],'W': [],'N': []}
	for u, v in G.edges:
		if v == u+1:
			barriers['W'].append(v)
		elif u == v+1:
			barriers['E'].append(v)
		elif u < v:
			barriers['N'].append(v)
		elif v < u:
			barriers['S'].append(v)
	with open(fn, "w") as file:
		file.write(json.dumps(barriers))

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

def random_search(fn, n=100):
	"""Try a number of barriers and choose the best"""
	graph = make_graph(fn)
	print(diameter(graph))
	best_solution = {'graph': graph.copy(), 'value': diameter(graph)}
	for i in range(n):
		perturbate(graph)
		new_value = diameter(graph)
		print(f"diameter: {new_value}")
		if new_value < best_solution['value']:
			best_solution['graph'] = graph.copy()
			best_solution['value'] = new_value
			print_barriers(best_solution['graph'], "test-random.json")
			print(f"Record ({i})")
	print(f"Best diameter: {diameter(best_solution['graph'])}")			
	print_barriers(best_solution['graph'], "test-random.json")


def diameter(G):
	"""Compute the diameter using an exact or approximate method."""
	if G.number_of_nodes() < 2000:
		return nx.diameter(G)
	else:
		return nx.algorithms.approximation.diameter(G)
		# diams = [nx.algorithms.approximation.diameter(G) for _ in range(4)]
		# return max(diams)

def approx_diameter(G):
	G_reversed = G.reverse()
	best_pair = {'a': -1, 'b': -1, 'distance': 0}
	for i in range(4):
		source = random.choice(list(G))
		cur_pair = two_sweep_directed(G, G_reversed, source)
		# print(cur_pair)
		if cur_pair['distance'] > best_pair['distance']:
			best_pair = cur_pair
	return best_pair

def two_sweep_directed(G, G_reversed, source):
	forward_distances = nx.shortest_path_length(G, source) # compute forward distances from source
	backward_distances = nx.shortest_path_length(G_reversed, source) # compute backward distances from source
	*_, a_1 = forward_distances # take a node a_1 at the maximum distance from the source in G
	# print(f"dist({source}, {a_1}) = {forward_distances[a_1]}")
	*_, a_2 = backward_distances # take a node a_2 at the maximum distance from the source in G_reversed
	# print(f"dist({a_2}, {source}) = {backward_distances[a_2]}")
	# do the same in the other direction
	backward_distances = nx.shortest_path_length(G_reversed, a_1)
	*_, a_12 = backward_distances
	a_12_dist = backward_distances[a_12]
	forward_distances = nx.shortest_path_length(G, a_2)
	*_, a_21 = forward_distances
	a_21_dist = forward_distances[a_21]
	# return the most distant pair
	if a_12_dist > a_21_dist:
		return {'a': a_1, 'b': a_12, 'distance': a_12_dist}
	else:
		return {'a': a_2, 'b': a_21, 'distance': a_21_dist}

def simulated_annealing(fn, generations=1000, start=None):
	"""
	Solve the maximum independent set problem with simulated annealing.
	"""
	if start is None:
		graph = make_graph(fn)
	else:
		graph = make_digraph(fn, start)
	cur_sol = {'graph': graph.copy(), 'value': diameter(graph)}
	best_sol = copy.deepcopy(cur_sol)
	print_barriers(best_sol['graph'])
	print("Simulated annealing: ", end="", flush=True)
	for i in range(generations):
		T = 1 - i/generations
		neighbor = cur_sol['graph'].copy()
		perturbate(neighbor)
		next_sol = {'graph': neighbor, 'value': diameter(neighbor)}
		if next_sol['value'] < cur_sol['value'] or random.random() < math.exp(-1/T):
			cur_sol = next_sol
			if cur_sol['value'] < best_sol['value']:
				best_sol = cur_sol
				print(f"{next_sol['value']}* ({i})", end=' ', flush=True)
				print_barriers(best_sol['graph'], "autosave-sa.json")
			# else:
				# print(next_sol['value'], end=' ', flush=True)
	print(f"Best diameter: {best_sol['value']}")


def get_cycle(path_ab, path_ba):
	"""Get the cycle defined by a path from a-b and a path b-a."""
	# detect if both cycles are the same
	if path_ba[::-1] == path_ab:
		return []
	# one half of the cycle
	start_ab = 0
	while path_ab[start_ab+1] == path_ba[-(start_ab+1) - 1]:
		start_ab += 1
	end_ab = len(path_ab) - 1
	while path_ab[end_ab-1] == path_ba[len(path_ab) - (end_ab-1) - 1]:
		end_ab -= 1
	# other half of the cycle
	start_ba = 0
	while path_ba[start_ba+1] == path_ab[-(start_ba+1) - 1]:
		start_ba += 1
	end_ba = len(path_ba) - 1
	while path_ba[end_ba-1] == path_ab[len(path_ba) - (end_ba-1) - 1]:
		end_ba -= 1
	# we build the cycle
	cycle = path_ab[start_ab:end_ab] + path_ba[start_ba:end_ba]
	# for i in range(start_ab, end_ab):
	# 	cycle.append(path_ab[i])
	# for i in range(start_ba, end_ba):
	# 	cycle.append(path_ba[i])
	return cycle

def reverse_cycle(G, cycle):
	for i in range(len(cycle)):
		u, v = cycle[i], cycle[(i + 1) % len(cycle)]
		assert G.has_edge(u, v)
		G.remove_edge(u, v)
		G.add_edge(v, u)

def shorten_largest_path(graph):
	"""Get a large shortest path (possibly the largest) and reverse a cycle to make a shortcut."""
	pair = approx_diameter(graph)
	path_ab = nx.shortest_path(graph, pair['a'], pair['b'])
	path_ba = find_paths(graph, path_ab, True)
	assert len(path_ab) >= len(path_ba)
	if len(path_ab) == len(path_ba):
		# print("Both paths have the same length")
		return
	cycle = get_cycle(path_ab, path_ba)
	reverse_cycle(graph, cycle)

def shorten_largest_path_smarter(graph):
	"""Get a large shortest path (possibly the largest) and reverse a cycle to make a shortcut."""
	pair = approx_diameter(graph)
	path_ab = nx.shortest_path(graph, pair['a'], pair['b'])
	paths = find_paths(graph, path_ab, False)
	best_move = {'value': None, 'cycle': []}
	for path_ba in paths:
		cycle = get_cycle(path_ab, path_ba)
		if not cycle:
			continue
		H = copy.deepcopy(graph)
		reverse_cycle(H, cycle)
		cur_diam = diameter(H)
		if best_move['value'] is None or cur_diam < best_move['value']:
			best_move['value'] = cur_diam
			best_move['cycle'] = cycle
	reverse_cycle(graph, best_move['cycle'])


def find_paths(G, path, shortest=True):
	# Reverse the path in the graph
	H = copy.deepcopy(G)
	for i in range(len(path) - 1):
		u, v = path[i], path[i + 1]
		assert H.has_edge(u, v)
		H.remove_edge(u, v)
		H.add_edge(v, u)
	if shortest:
		path_ba = nx.shortest_path(H, path[-1], path[0])
		return path_ba
	# Look for paths
	paths = []
	for i in range(10):
		for u, v, d in H.edges(data=True):
			d['weight'] = random.uniform(0.5, 1)
		path_ba = nx.shortest_path(H, source=path[-1], target=path[0], weight='weight') 
		paths.append(path_ba)
	return paths


def greedy(fn, n=100, start=None):
	"""
	Take the largest path and reverse a cycle to make it shorter.
	Note that this can also increase the diameter of the digraph.
	"""
	best_diameter = None
	if start is None:
		graph = make_graph(fn)
	else:
		graph = make_digraph(fn, start)
	print_barriers(graph)
	print(f"Initial diameter >= {diameter(graph)}")
	print("Greedy: ", end='')
	for i in range(n):
		shorten_largest_path(graph)
		cur_diameter = diameter(graph)
		if best_diameter is None or cur_diameter < best_diameter:
			best_diameter = cur_diameter
			print(f"{cur_diameter}* ({i}) ", end="", flush=True)
			print_barriers(graph, "autosave-greedy.json")
		else:
			print(f"{cur_diameter} ", end="", flush=True)
	print(f"Best diameter: {best_diameter}")


def greedy_smarter(fn, n=100, start=None):
	"""
	Greedy algorithm for reducing the diameter of the strong orientation.
	At each iteration, we search the largest distance and try to reverse different paths to reduce the diameter
	"""
	best_diameter = None
	if start is None:
		graph = make_graph(fn)
	else:
		graph = make_digraph(fn, start)
	print_barriers(graph)
	print(f"Initial diameter >= {diameter(graph)}")
	print("Greedy smarter: ", end='')
	for i in range(n):
		shorten_largest_path_smarter(graph)
		cur_diameter = diameter(graph)
		# print(f"Diameter >= {cur_diameter}")
		if best_diameter is None or cur_diameter < best_diameter:
			best_diameter = cur_diameter
			print(f"{cur_diameter}* ({i}) ", end="", flush=True)
			print_barriers(graph, "autosave-greedy.json")
		else:
			print(f"{cur_diameter} ", end="", flush=True)
	print(f"Best diameter: {best_diameter}")



if __name__ == "__main__":
	# fn = "random-32-32-20.map"
	# fn = "brc202d.map"
	# fn = "Paris_1_256.map"
	# fn = "sortation_large.map"
	fn = "warehouse_small.map"
	# random_search(fn, 10)
	# simulated_annealing(fn, 1000, 'autosave-sa.json')
	# greedy(fn, 100)
	greedy_smarter(fn, 10)
