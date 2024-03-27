"""
Script for improving the barriers of a map.
I only want this for the map random-32-32-20.map
"""

import json
import random
import itertools
from collections import deque
import copy
import math

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

def make_graph(map):
	"""Make an (undirected) graph"""
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


def write_barriers(G, fn="test-barriers.json"):
	barriers = {'diameter': -1, 'E': [],'S': [],'W': [],'N': []}
	for u, v in G.edges:
		if v == u+1:
			barriers['W'].append(v)
		elif u == v+1:
			barriers['E'].append(v)
		elif u < v:
			barriers['N'].append(v)
		elif v < u:
			barriers['S'].append(v)
	if nx.is_strongly_connected(G):
		barriers['diameter'] = diameter(G)
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


def diameter(G):
	"""Compute the diameter using an exact or approximate method."""
	if G.number_of_nodes() < 2000:
		return nx.diameter(G)
	else:
		return nx.algorithms.approximation.diameter(G)


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


def reverse_arc(digraph, u, v):
	"""Reverse an arc in a digraph. You are responsible for keeping the digraph strongly connected."""
	assert digraph.has_edge(u, v)
	digraph.remove_edge(u, v)
	digraph.add_edge(v, u)


def reverse_path_between(digraph, u, v):
	"""
	Reverse a path (the shortest path between two nodes) in a digraph. 
	You are responsible for keeping the digraph strongly connected.
	"""
	path = nx.shortest_path(digraph, source=u, target=v)
	for i in range(len(path) - 1):
		reverse_arc(digraph, path[i], path[i+1])

def reverse_path(digraph, path):
	for i in range(len(path) - 1):
		reverse_arc(digraph, path[i], path[i+1])

def rotate_distance(digraph, source, target):
	"""Compute the real distance, taking turns into account"""
	dist = {node: None for node in digraph.nodes}
	prev = {node: None for node in digraph.nodes}
	dist[target] = 0
	queue = deque()
	queue.append(target)
	while queue:
		cur_loc = queue.popleft()
		if cur_loc == source:
			return dist[source]
		for next_loc in digraph.predecessors(cur_loc):
			next_dist = 0
			if dist[cur_loc] == 0:
				next_dist = 1
			else:
				if next_loc - cur_loc == cur_loc - prev[cur_loc]:
					next_dist = dist[cur_loc] + 1
				else:
					next_dist = dist[cur_loc] + 2
			if dist[next_loc] is None or next_dist < dist[next_loc]:
				dist[next_loc] = next_dist
				prev[next_loc] = cur_loc
				queue.append(next_loc)
	assert False
	return None


def distant_neighbors(digraph):
	"""Get the pairs of neigbors that are at distance greater than 5"""
	pairs = []
	for e in digraph.edges:
		delta = detour_distance(digraph, *e)
		if delta > 15:
			pairs.append({'edge': e, 'dist': delta})
	pairs.sort(key=lambda x: x['dist'], reverse=True)
	return pairs

def detour_distance(digraph, u, v):
	"""
	Find the difference between the minimum distance between
	two nodes and the distance in the digraph
	"""
	graph = digraph.to_undirected()
	graph.remove_edge(u, v)
	d1 = nx.shortest_path_length(digraph, source=v, target=u)
	d2 = nx.shortest_path_length(graph, source=v, target=u)
	return d1 - d2


def confluence_nodes(digraph):
	"""Nodes with two neigbors coming from opposite directions."""
	nodes = []
	for node in digraph.nodes:
		in_nodes = list(digraph.predecessors(node))
		if len(in_nodes) == 3:
			nodes.append(node)
		elif len(in_nodes) == 2 and abs(node - in_nodes[0]) == abs(node - in_nodes[1]):
			nodes.append(node)
	nodes.sort()
	return nodes

def diameter_pair(digraph):
	"""Pair of nodes having the largest distance"""
	G_reversed = digraph.reverse()
	best_pair = {'a': -1, 'b': -1, 'distance': 0}
	for _ in range(4):
		source = random.choice(list(digraph))
		cur_pair = two_sweep_directed(digraph, G_reversed, source)
		if cur_pair['distance'] > best_pair['distance']:
			best_pair = cur_pair
	return best_pair

def eval_orientation(digraph):
	"""Evaluate a strong orientation"""
	if nx.number_strongly_connected_components(digraph) != 1:
		print("Error: the graph is not strongly connected: there are some narrow corridors")
		for c in sorted(nx.strongly_connected_components(digraph), key=len, reverse=True):
			print(f"There is a component of size {len(c)} with vertex {list(c)[0]}")
	# assert nx.number_strongly_connected_components(digraph) == 1
	dn = distant_neighbors(digraph)
	print(f"There are {len(dn)} distant neighbors: {dn}")
	cn = confluence_nodes(digraph)
	print(f"There are {len(cn)} confluence nodes: {cn}")
	print(f"There are {len(dn)} distant neighbors and {len(cn)} confluence nodes ")
	# print("max distance: ", diameter_pair(digraph))


#####################
# Simulated annealing
#####################


# def simulated_annealing(fn, generations=1000):
# 	"""Simulated annealing for improving one of the criteria"""
# 	def value(G):
# 		# return len(confluence_nodes(G))
# 		return nx.diameter(G)
# 	digraph = make_digraph(fn, "test-barriers.json")
# 	# assert valid_orientation(graph, arcs)
# 	cur_sol = {'graph': digraph.copy(), 'value': value(digraph)}
# 	best_sol = copy.deepcopy(cur_sol)
# 	write_barriers(best_sol['graph'], "autosave-sa.json")
# 	print("Simulated annealing: ", end="", flush=True)
# 	for i in range(generations):
# 		T = 1 - i/generations
# 		neighbor = cur_sol['graph'].copy()
# 		perturbate(neighbor)
# 		next_sol = {'graph': neighbor, 'value': value(neighbor)}
# 		if next_sol['value'] < cur_sol['value'] or random.random() < math.exp(-1/T):
# 			cur_sol = next_sol
# 			# write_barriers(next_sol['graph'], "autosave-sa-bad.json")
# 			print(f"{next_sol['value']} ({i})", end=' ', flush=True)
# 			if cur_sol['value'] < best_sol['value']:
# 				best_sol = cur_sol
# 				print(f"{next_sol['value']}* ({i})", end=' ', flush=True)
# 				write_barriers(best_sol['graph'], "autosave-sa.json")
# 			# else:
# 				# print(next_sol['value'], end=' ', flush=True)
# 	print(f"Best diameter: {best_sol['value']}")



# def perturbate(G):
# 	cycle = random_cycle(G)
# 	for u, v in cycle:
# 		reverse_arc(G, u, v)

# def random_cycle(G):
# 	H = copy.deepcopy(G)
# 	start = random.choice(list(G.nodes()))
# 	end = random.choice(list(G.neighbors(start)))
# 	for u, v, d in H.edges(data=True):
# 		d['weight'] = random.uniform(0.1, 1)
# 	path = nx.shortest_path(H, source=end, target=start, weight='weight') 
# 	cycle = [(path[i], path[(i+1) % len(path)]) for i in range(len(path))]
# 	return cycle


# ########
# # Greedy
# ########

# def greedy_smarter(fn, start="test-barriers.json", n=100):
# 	"""
# 	Greedy algorithm for reducing the diameter of the strong orientation.
# 	At each iteration, we search the largest distance and try to reverse different paths to reduce the diameter
# 	"""
# 	best_diameter = None
# 	digraph = make_digraph(fn, start)
# 	print(f"Initial diameter >= {diameter(digraph)}")
# 	print("Greedy smarter: ", end='')
# 	for i in range(n):
# 		shorten_largest_path(digraph)
# 		cur_diameter = diameter(digraph)
# 		# print(f"Diameter >= {cur_diameter}")
# 		if best_diameter is None or cur_diameter < best_diameter:
# 			best_diameter = cur_diameter
# 			print(f"{cur_diameter}* ({i}) ", end="", flush=True)
# 			write_barriers(digraph, "autosave-greedy.json")
# 		else:
# 			print(f"{cur_diameter} ", end="", flush=True)
# 	print(f"Best diameter: {best_diameter}")


# def shorten_largest_path(graph):
# 	"""Get a large shortest path (possibly the largest) and reverse a cycle to make a shortcut."""
# 	pair = diameter_pair(graph)
# 	path_ab = nx.shortest_path(graph, pair['a'], pair['b'], weight='weight')
# 	paths = find_paths(graph, path_ab, False)
# 	best_move = {'value': None, 'cycle': []}
# 	for path_ba in paths:
# 		cycle = get_cycle(path_ab, path_ba)
# 		if not cycle:
# 			continue
# 		H = copy.deepcopy(graph)
# 		reverse_cycle(H, cycle)
# 		cur_diam = diameter(H)
# 		if best_move['value'] is None or cur_diam < best_move['value']:
# 			best_move['value'] = cur_diam
# 			best_move['cycle'] = cycle
# 	reverse_cycle(graph, best_move['cycle'])

# def find_paths(G, path, shortest=True):
# 	# Reverse the path in the graph
# 	H = copy.deepcopy(G)
# 	for i in range(len(path) - 1):
# 		u, v = path[i], path[i + 1]
# 		assert H.has_edge(u, v)
# 		H.remove_edge(u, v)
# 		H.add_edge(v, u, weight=1)
# 	if shortest:
# 		path_ba = nx.shortest_path(H, path[-1], path[0], weight='weight')
# 		return path_ba
# 	# Look for paths
# 	paths = []
# 	for i in range(10):
# 		for u, v, d in H.edges(data=True):
# 			d['weight'] = random.uniform(0.1, 1)
# 		path_ba = nx.shortest_path(H, source=path[-1], target=path[0], weight='weight')
# 		paths.append(path_ba)
# 	return paths


# def get_cycle(path_ab, path_ba):
# 	"""Get the cycle defined by a path from a-b and a path b-a."""
# 	# detect if both cycles are the same
# 	if path_ba[::-1] == path_ab:
# 		return []
# 	# one half of the cycle
# 	start_ab = 0
# 	while path_ab[start_ab+1] == path_ba[-(start_ab+1) - 1]:
# 		start_ab += 1
# 	end_ab = len(path_ab) - 1
# 	while path_ab[end_ab-1] == path_ba[len(path_ab) - (end_ab-1) - 1]:
# 		end_ab -= 1
# 	# other half of the cycle
# 	start_ba = 0
# 	while path_ba[start_ba+1] == path_ab[-(start_ba+1) - 1]:
# 		start_ba += 1
# 	end_ba = len(path_ba) - 1
# 	while path_ba[end_ba-1] == path_ab[len(path_ba) - (end_ba-1) - 1]:
# 		end_ba -= 1
# 	# we build the cycle
# 	cycle = path_ab[start_ab:end_ab] + path_ba[start_ba:end_ba]
# 	return cycle

# def reverse_cycle(G, cycle):
# 	for i in range(len(cycle)):
# 		u, v = cycle[i], cycle[(i + 1) % len(cycle)]
# 		assert G.has_edge(u, v)
# 		G.remove_edge(u, v)
# 		G.add_edge(v, u)


if __name__ == "__main__":
	digraph = make_digraph("random-32-32-20.map", "test-barriers.json")
	# digraph = make_digraph("random-32-32-20.map", "improved.json")
	reverse_path(digraph, [989, 1021, 1022, 1023, 991, 990, 989])
	reverse_path(digraph, [976, 977, 1009, 1008, 976])
	reverse_path(digraph, [850, 882, 914, 913, 881])
	reverse_path_between(digraph, 635, 636)
	reverse_path(digraph, [649, 617, 585, 584, 583])
	reverse_path(digraph, [538, 539, 540])
	reverse_path(digraph, [411, 412])
	reverse_path(digraph, [58, 26, 27, 28, 29]); reverse_path_between(digraph, 63, 58)
	reverse_path_between(digraph, 220, 254)
	reverse_path_between(digraph, 50, 48)
	reverse_path_between(digraph, 79, 83)
	reverse_path_between(digraph, 66, 72)
	reverse_path_between(digraph, 462, 469)
	reverse_path_between(digraph, 186, 183)
	reverse_path_between(digraph, 824, 820)
	reverse_path_between(digraph, 756, 819)
	reverse_path_between(digraph, 902, 904)
	reverse_path_between(digraph, 873, 936)
	reverse_path_between(digraph, 335, 341)
	reverse_path_between(digraph, 353, 256)
	reverse_path_between(digraph, 767, 766)
	reverse_path_between(digraph, 883, 819)
	write_barriers(digraph, "improved.json")
	eval_orientation(digraph)

	# simulated_annealing("random-32-32-20.map", 100)
	# greedy_smarter("random-32-32-20.map")