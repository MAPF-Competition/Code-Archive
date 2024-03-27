"""
Script for helping me setting the barriers for random.
"""

import json
import copy
import math
import random
from collections import deque

import networkx as nx

# random.seed(0) # for debugging

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

def make_graph(fn, arcs):
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
	return strong_orientation(G, arcs)


def strong_orientation(graph, arcs):
	G = nx.DiGraph()
	max_node = max(list(graph.nodes))
	depth = [None for _ in range(max_node + 1)]
	root = list(graph.nodes)[0]
	# depth[root] = 0
	stack = [(None, root)]
	while stack:
		prev, u  = stack.pop()
		if depth[u] is None:
			if prev is None:
				depth[u] = 0
			else:
				depth[u] = depth[prev] + 1
				G.add_edge(u, prev)
			previous_in_arcs = []
			previous_others = []
			for v in nx.neighbors(graph, u):
				if depth[v] is None:
					if (v, u) in arcs:
						previous_in_arcs.append((u, v))
					elif (u, v) not in arcs:
						previous_others.append((u, v))
			stack.extend(previous_others)
			stack.extend(previous_in_arcs)
	# Add rest of the arcs (not sure about this)
	for arc in arcs:
		if not G.has_edge(*arc):
			G.add_edge(*arc)
	# Complete the arcs
	for u, v in graph.edges():
		if depth[u] > depth[v] and not G.has_edge(u, v) and not G.has_edge(v, u):
			G.add_edge(v, u)
		if depth[v] > depth[u] and not G.has_edge(v, u) and not G.has_edge(u, v):
			G.add_edge(u, v)
	for arc in arcs:
		assert G.has_edge(*arc)
	print_barriers(G)
	if nx.number_strongly_connected_components(G) != 1:
		print("Error: the graph is not strongly connected: there are some narrow corridors")
		for c in sorted(nx.strongly_connected_components(G), key=len, reverse=True):
			print(f"There is a component of size {len(c)} with vertex {list(c)[0]}")
		# exit()
	# assert nx.number_strongly_connected_components(G) == 1
	print(f"The graph is strongly connected. Number of nodes: {G.number_of_nodes()}")
	return G

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


def set_weights(digraph, arcs):
	nx.set_edge_attributes(digraph, values = 1, name = 'weight')
	for u, v in arcs:
		digraph[u][v]['weight'] = 1000000


def valid_path(path, arcs):
	"""Return True if the path does not intersect the fixed arcs."""
	for arc in path:
		if arc in arcs:
			return False
	for i in range(len(path)):
		arc = (path[i], path[(i+1) % len(path)])
		if arc in arcs:
			return False
	return True

def valid_orientation(digraph, arcs):
	for arc in arcs:
		if not digraph.has_edge(*arc):
			return False
	return True


def perturbate(G, arcs):
	while True:
		cycle = random_cycle(G)
		if valid_path(cycle, arcs):
			break
	for u, v in cycle:
		assert G.has_edge(u, v)
		assert G[u][v]['weight'] == 1
		G.remove_edge(u, v)
		G.add_edge(v, u, weight=1)
	# set_weights(G, arcs)
	# assert nx.number_strongly_connected_components(G) == 1
	# print(f"Strongly CC: {nx.number_strongly_connected_components(G)}")


def random_cycle(G):
	H = copy.deepcopy(G)
	start = random.choice(list(G.nodes()))
	end = random.choice(list(G.neighbors(start)))
	for u, v, d in H.edges(data=True):
		d['weight'] *= random.uniform(0.5, 1)
	path = nx.shortest_path(H, source=end, target=start, weight='weight') 
	cycle = [(path[i], path[(i+1) % len(path)]) for i in range(len(path))]
	return cycle


def print_barriers(G, fn="test-barriers.json"):
	barriers = {'value': diameter(G), 'E': [],'S': [],'W': [],'N': []}
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


def value(G, arcs):
	"""Number of arcs in arcs that are not well oriented in the graph."""
	count = 0
	for u, v in arcs:
		if not G.has_edge(u, v):
			count += 1
	return count

# def simulated_annealing(fn, arcs, generations=1000, start=None):
# 	"""
# 	Solve the maximum independent set problem with simulated annealing.
# 	"""
# 	if start is None:
# 		graph = make_graph(fn)
# 	else:
# 		graph = make_digraph(fn, start)
# 	cur_sol = {'graph': graph.copy(), 'value': value(graph, arcs)}
# 	best_sol = copy.deepcopy(cur_sol)
# 	print_barriers(best_sol['graph'], arcs)
# 	print(f"Simulated annealing: {best_sol['value']}", end=" ", flush=True)
# 	for i in range(generations):
# 		T = 1 - i/generations
# 		neighbor = cur_sol['graph'].copy()
# 		perturbate(neighbor)
# 		next_sol = {'graph': neighbor, 'value': value(neighbor, arcs)}
# 		if next_sol['value'] < cur_sol['value'] or random.random() < math.exp(-1/T):
# 			cur_sol = next_sol
# 			if cur_sol['value'] < best_sol['value']:
# 				best_sol = cur_sol
# 				print(f"{next_sol['value']}* ({i})", end=' ', flush=True)
# 				print_barriers(best_sol['graph'], arcs, "autosave-partial-sa.json")
# 			# else:
# 				# print(next_sol['value'], end=' ', flush=True)
# 	print(f"Best value: {best_sol['value']}")


# def greedy(fn, arcs, start=None):
# 	if start is None:
# 		graph = make_graph(fn)
# 		G = strong_orientation(graph)
# 	else:
# 		G = make_digraph(fn, start)
# 	print_barriers(G, arcs)
# 	fixed = {arc: False for arc in arcs}
# 	for arc in fixed:
# 		u, v = arc
# 		if G.has_edge(v, u):
# 			if nx.has_path(G, source=u, target=v):
# 				path = nx.shortest_path(G, source=u, target=v)
# 				reverse_cycle(G, path)
# 				G.remove_edge(u, v)
# 				fixed[arc] = True
# 				print_barriers(G, arcs)
# 	# assert sum(1 for arc in fixed if fixed[arc]) == len(fixed)
# 	for arc in fixed:
# 		if fixed[arc]:
# 			G.add_edge(*arc)
# 	print_barriers(G, arcs, "autosave-partial-test.json")
# 	print(f"Strongly CC: {nx.number_strongly_connected_components(G)}")
# 	print(f"Value of the graph: {value(G, arcs)}")
# 	return G



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

def shorten_largest_path(graph, arcs):
	"""Get a large shortest path (possibly the largest) and reverse a cycle to make a shortcut."""
	pair = approx_diameter(graph)
	path_ab = nx.shortest_path(graph, pair['a'], pair['b'], weight='weight')
	if not valid_path(path_ab, arcs):
		print("The only path passes by the fixed arcs")
		return
	paths = find_paths(graph, path_ab, False)
	best_move = {'value': None, 'cycle': []}
	for path_ba in paths:
		assert valid_path(path_ba, arcs)
		cycle = get_cycle(path_ab, path_ba)
		if not cycle:
			continue
		assert valid_path(cycle, arcs)
		H = copy.deepcopy(graph)
		reverse_cycle(H, cycle)
		cur_diam = diameter(H)
		if best_move['value'] is None or cur_diam < best_move['value']:
			best_move['value'] = cur_diam
			best_move['cycle'] = cycle
	reverse_cycle(graph, best_move['cycle'])
	assert valid_orientation(graph, arcs)


def reverse_cycle(G, cycle):
	for i in range(len(cycle)):
		u, v = cycle[i], cycle[(i + 1) % len(cycle)]
		assert G.has_edge(u, v)
		G.remove_edge(u, v)
		G.add_edge(v, u)

def find_paths(G, path, shortest=True):
	# Reverse the path in the graph
	H = copy.deepcopy(G)
	for i in range(len(path) - 1):
		u, v = path[i], path[i + 1]
		assert H.has_edge(u, v)
		H.remove_edge(u, v)
		H.add_edge(v, u, weight=1)
	if shortest:
		path_ba = nx.shortest_path(H, path[-1], path[0], weight='weight')
		return path_ba
	# Look for paths
	paths = []
	for i in range(10):
		for u, v, d in H.edges(data=True):
			if d['weight'] <= 1:
				d['weight'] = random.uniform(0.1, 1)
		path_ba = nx.shortest_path(H, source=path[-1], target=path[0], weight='weight')
		paths.append(path_ba)
	return paths

def make_arcs(fn):
	"""Make some arcs that will be fixed."""
	arcs = []
	if fn == "warehouse_large.map":
		# Add east rows
		for offset in range(5007, 64007, 3000):
			arcs.extend([(offset + j, offset + j + 1) for j in range(484)])
		# Add south columns
		for offset in range(3511, 3990, 8):
			arcs.extend([(offset + j, offset + j + 500) for j in range(0, 61500, 500)])
		# Add west rows
		for offset in range(6507, 64007, 3000):
			arcs.extend([(offset + j + 1, offset + j) for j in range(484)])
		# Add north columns
		for offset in range(3515, 3990, 8):
			arcs.extend([(offset + j + 500, offset + j) for j in range(0, 61500, 500)])
	elif fn == "warehouse_small.map":
		# Add east rows
		for offset in range(577, 1375, 57*6):
			arcs.extend([(offset + j, offset + j + 1) for j in range(40)])
		# Add south columns
		for offset in range(410, 445, 8):
			arcs.extend([(offset + j, offset + j + 57) for j in range(0, 1026, 57)])
		# Add west rows
		for offset in range(748, 1375, 57*6):
			arcs.extend([(offset + j + 1, offset + j) for j in range(40)])
		# Add north columns
		for offset in range(414, 445, 8):
			arcs.extend([(offset + j + 57, offset + j) for j in range(0, 1026, 57)])
	return arcs

def greedy_smarter(fn, arcs, n=100, start=None):
	"""
	Greedy algorithm for reducing the diameter of the strong orientation.
	At each iteration, we search the largest distance and try to reverse different paths to reduce the diameter
	"""
	best_diameter = None
	if start is None:
		graph = make_graph(fn, arcs)
	else:
		graph = make_digraph(fn, start)
	assert valid_orientation(graph, arcs)
	set_weights(graph, arcs)
	print_barriers(graph)
	
	print(f"Initial diameter >= {diameter(graph)}")
	print("Greedy smarter: ", end='')
	for i in range(n):
		shorten_largest_path(graph, arcs)
		cur_diameter = diameter(graph)
		# print(f"Diameter >= {cur_diameter}")
		if best_diameter is None or cur_diameter < best_diameter:
			best_diameter = cur_diameter
			print(f"{cur_diameter}* ({i}) ", end="", flush=True)
			print_barriers(graph, "autosave-greedy.json")
		else:
			print(f"{cur_diameter} ", end="", flush=True)
	print(f"Best diameter: {best_diameter}")


def simulated_annealing(fn, arcs, generations=1000, start=None):
	if start is None:
		graph = make_graph(fn, arcs)
	else:
		graph = make_digraph(fn, start)
	# assert valid_orientation(graph, arcs)
	set_weights(graph, arcs)

	cur_sol = {'graph': graph.copy(), 'value': diameter(graph)}
	best_sol = copy.deepcopy(cur_sol)
	print_barriers(best_sol['graph'], "autosave-sa.json")
	print("Simulated annealing: ", end="", flush=True)
	for i in range(generations):
		T = 1 - i/generations
		neighbor = cur_sol['graph'].copy()
		perturbate(neighbor, arcs)
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



if __name__ == "__main__":
	# fn = "random-32-32-20.map"
	# fn = "brc202d.map"
	# fn = "Paris_1_256.map"
	# fn = "sortation_large.map"
	fn = "warehouse_large.map"
	# fn = "warehouse_small.map"
	arcs = make_arcs(fn)	
	# greedy_smarter(fn, arcs, 10, "autosave-sa.json")
	simulated_annealing(fn, arcs, 1000, "autosave-sa.json")