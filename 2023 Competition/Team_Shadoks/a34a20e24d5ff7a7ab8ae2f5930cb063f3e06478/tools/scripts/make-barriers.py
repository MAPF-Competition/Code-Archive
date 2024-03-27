"""
Script for helping me setting the barriers for random.
"""

import json
import random
import itertools

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


def strong_orientation(graph, arcs):
	"""Make a strongly connected digraph from a graph using DFS."""
	G = nx.DiGraph()
	max_node = max(list(graph.nodes))
	depth = [None for _ in range(max_node + 1)]
	root = list(graph.nodes)[0]
	# depth[root] = 0
	stack = [(None, root)]
	while stack:
		prev, u = stack.pop()
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
	write_barriers(G)
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


def reverse_arc(digraph, u, v):
	"""Reverse an arc in a digraph. You are responsible for keeping the digraph strongly connected."""
	assert digraph.has_edge(u, v)
	digraph.remove_edge(u, v)
	digraph.add_edge(v, u)


def reverse_path(digraph, u, v):
	"""
	Reverse a path (the shortest path between two nodes) in a digraph. 
	You are responsible for keeping the digraph strongly connected.
	"""
	path = nx.shortest_path(digraph, source=u, target=v)
	for i in range(len(path) - 1):
		reverse_arc(digraph, path[i], path[i+1])


def post_process(digraph):
	"""
	Make some simple modifications in the digraph to reduce the number of strongly connected components.
	"""
	for u in digraph.nodes():
		if digraph.in_degree(u) == 0:
			v = find_straight_path(digraph, u, True)
			if v is not None:
				reverse_path(digraph, u, v)
			else:
				v = find_short_path(digraph, u, True)
				if v is not None:
					reverse_path(digraph, u, v)
		elif digraph.out_degree(u) == 0:
			v = find_straight_path(digraph, u, False)
			if v is not None:
				reverse_path(digraph, v, u)
			else:
				v = find_short_path(digraph, u, False)
				if v is not None:
					reverse_path(digraph, v, u)


def find_straight_path(digraph, u1, forward=True):
	if forward:
		for u2 in digraph.successors(u1):
			for u3 in digraph.successors(u2):
				if u3-u2 == u2-u1 and digraph.in_degree(u3) > 1:
					return u3
	else:
		for u2 in digraph.predecessors(u1):
			for u3 in digraph.predecessors(u2):
				if u3-u2 == u2-u1 and digraph.out_degree(u3) > 1:
					return u3
	return None

def find_short_path(digraph, u, forward=True):
	if forward:
		for arc in nx.bfs_edges(digraph, source=u):
			if digraph.in_degree(arc[1]) > 1:
				return arc[1]
	else:
		for arc in nx.bfs_edges(digraph, source=u, reverse=True):
			if digraph.out_degree(arc[1]) > 1:
				return arc[1]
	return None


def make_arcs_warehouse():
	"""Make the barriers for the warehouse"""
	fn = "warehouse_large.map"
	map = read_map(fn)
	digraph = nx.DiGraph()
	# Simple pattern
	for x in range(3, 497):
		for y in range(3, 137):
			loc = coords_to_location(x, y, map)
			if y+1 < 137 and map['pixels'][loc] and map['pixels'][loc+map['width']]:
				if x%2 == 0:
					digraph.add_edge(loc, loc+map['width'])
				else:
					digraph.add_edge(loc+map['width'], loc)
			if x+1 < 497 and map['pixels'][loc] and map['pixels'][loc+1]:
				if y%2 == 0:
					digraph.add_edge(loc, loc+1)
				else:
					digraph.add_edge(loc+1, loc)
	# Alternate vertical hallways
	for x in range(11, 491, 8): # south
		u = coords_to_location(x, 136, map)
		v = coords_to_location(x, 7, map)
		reverse_path(digraph, u, v)
	# Two lane highways
	for u, v in [(2003, 2495), (2995, 2503), (67004, 67496), (67996, 67504)]:
		reverse_path(digraph, u, v)
	post_process(digraph)





	# # Thin hallways with alternating directions
	# for y in range(10, 130, 6): # east
	# 	for x in range(7, 491):
	# 		loc = coords_to_location(x, y, map)
	# 		digraph.add_edge(loc, loc+1)
	# for y in range(7, 130, 6): # west
	# 	for x in range(7, 491):
	# 		loc = coords_to_location(x, y, map)
	# 		digraph.add_edge(loc+1, loc)
	# for x in range(11, 491, 8): # south
	# 	for y in range(7, 130):
	# 		loc = coords_to_location(x, y, map)
	# 		digraph.add_edge(loc, loc+500)
	# for x in range(7, 491, 8): # north
	# 	for y in range(7, 130):
	# 		loc = coords_to_location(x, y, map)
	# 		digraph.add_edge(loc+500, loc)
	# # Large corridors
	# for x in range(0, 500):
	# 	for y in range(0, 140):
	# 		if not (7 <= x < 491 and 7 <= y < 130):
	# 			loc = coords_to_location(x, y, map)
	# 			if y+1 < map['height'] and map['pixels'][loc] and map['pixels'][loc+500]:
	# 				if x%2 == 0:
	# 					digraph.add_edge(loc, loc+500)
	# 				else:
	# 					digraph.add_edge(loc+500, loc)
	# 			if x+1 < map['width'] and map['pixels'][loc] and map['pixels'][loc+1]:
	# 				if y%2 == 0:
	# 					digraph.add_edge(loc, loc+1)
	# 				else:
	# 					digraph.add_edge(loc+1, loc)
	# # Reverse some paths in top row
	# for x in range(4, 487, 14):
	# 	for dx in range(0, 3):
	# 		loc = coords_to_location(x + dx, 0, map)
	# 		reverse_arc(digraph, loc, loc+1)
	# reverse_arc(digraph, 494, 494+1)
	# # in left column
	# for y in range(16, 128, 14):
	# 	for dy in range(0, 3):
	# 		loc = coords_to_location(0, y + dy, map)
	# 		reverse_arc(digraph, loc, loc+500)
	# for y in range(128, 135):
	# 	loc = coords_to_location(0, y, map)
	# 	reverse_arc(digraph, loc, loc+500)
	# reverse_arc(digraph, 2000, 2000+500)
	# # in bottom row
	# for x in range(16, 496, 14):
	# 	for dx in range(0, 3):
	# 		loc = coords_to_location(x + dx, 139, map)
	# 		reverse_arc(digraph, loc+1, loc)
	# reverse_arc(digraph, 69504+1, 69504)
	# # in right column
	# for y in range(22, 134, 14):
	# 	for dy in range(0, 3):
	# 		loc = coords_to_location(499, y + dy, map)
	# 		reverse_arc(digraph, loc+500, loc)
	# for y in range(4, 11):
	# 	loc = coords_to_location(499, y, map)
	# 	reverse_arc(digraph, loc+500, loc)
	# reverse_arc(digraph, 67499+500, 67499)

	write_barriers(digraph)
	n_scc = nx.number_strongly_connected_components(digraph)
	if n_scc != 1:
		print(f"Error: the graph is not strongly connected: there are {n_scc} components")
		scc = [f"(v:{list(c)[0]}, size:{len(c)})" for c in sorted(nx.strongly_connected_components(digraph), key=len, reverse=True)]
		print(scc)
		# for c in sorted(nx.strongly_connected_components(digraph), key=len, reverse=True):
			# print(f"There is a component of size {len(c)} with vertex {list(c)[0]}")
	else:
		print("The digraph is strongly connected :D")


def make_arcs_sortation():
	"""Make the barriers for sortation"""
	fn = "sortation_large.map"
	map = read_map(fn)
	digraph = nx.DiGraph()
	# Simple pattern
	for x in range(0, 500):
		for y in range(0, 140):
			loc = coords_to_location(x, y, map)
			if y+1 < map['height'] and map['pixels'][loc] and map['pixels'][loc+500]:
				if x%2 == 0:
					digraph.add_edge(loc, loc+500)
				else:
					digraph.add_edge(loc+500, loc)
			if x+1 < map['width'] and map['pixels'][loc] and map['pixels'][loc+1]:
				if y%2 == 0:
					digraph.add_edge(loc, loc+1)
				else:
					digraph.add_edge(loc+1, loc)
	# Thin hallways with alternating directions
	for y in range(7, 133, 4): # east
		for x in range(5, 493):
			loc = coords_to_location(x, y, map)
			reverse_arc(digraph, loc+1, loc)
	for x in range(7, 493, 4): # south
		for y in range(5, 133):
			loc = coords_to_location(x, y, map)
			reverse_arc(digraph, loc+500, loc)

	# Alternating directions in the rows: 25802 tasks with PlannerLazy, 800 steps, 10k agents planned
	# reverse_arc(digraph, 67501, 67500)
	# reverse_arc(digraph, 69505, 69504)
	# reverse_arc(digraph, 995, 495)
	# reverse_arc(digraph, 2498, 2499)
	# reverse_arc(digraph, 2000, 2001)
	# reverse_arc(digraph, 4, 504)
	# reverse_arc(digraph, 69995, 69495)
	# reverse_arc(digraph, 67999, 67499)

	# Make obstacles in the contour: 25972 tasks with PlannerLazy, 800 steps, 10k agents planned
	# for x in range(4, 495, 2):
	# 	loc = coords_to_location(x, 0, map)
	# 	reverse_arc(digraph, loc, loc+1)
	# 	loc = coords_to_location(x, 139, map)
	# 	reverse_arc(digraph, loc+1, loc)
	# reverse_arc(digraph, 67501, 67500)
	# reverse_arc(digraph, 2498, 2499)
	# reverse_arc(digraph, 2000, 2001)
	# reverse_arc(digraph, 67999, 67499)

	# 2-lane highway: 28021 tasks with PlannerLazy, 800 steps, 10k agents planned
	reverse_path(digraph, 995, 504)
	reverse_path(digraph, 2000, 2499)
	reverse_path(digraph, 69004, 69495)
	reverse_path(digraph, 67999, 67500)
	reverse_arc(digraph, 69505, 69504)
	reverse_arc(digraph, 995, 495)
	reverse_arc(digraph, 994, 995)
	reverse_arc(digraph, 4, 5)
	reverse_arc(digraph, 69995, 69994)

	# 1-lane highway: 26599 tasks with PlannerLazy, 800 steps, 10k agents planned
	# reverse_path(digraph, 995, 504)
	# reverse_path(digraph, 1995, 1504)
	# reverse_path(digraph, 2995, 2504)
	# reverse_path(digraph, 69004, 69495)
	# reverse_path(digraph, 68004, 68495)
	# reverse_path(digraph, 67004, 67495)
	# reverse_arc(digraph, 67501, 67500)
	# reverse_arc(digraph, 69505, 69504)
	# reverse_arc(digraph, 494, 495)
	# reverse_arc(digraph, 2999, 2499)
	# reverse_arc(digraph, 2000, 2500)
	# reverse_arc(digraph, 4, 5)
	# reverse_arc(digraph, 69995, 69994)
	# reverse_arc(digraph, 67999, 67998)

	# Reverse some paths in top row
	# for x in range(4, 495):
	# 	loc = coords_to_location(x, 0, map)
	# 	reverse_arc(digraph, loc, loc+1)
	# for y in range(4, 135):
	# 	loc = coords_to_location(0, y, map)
	# 	reverse_arc(digraph, loc, loc+500)
	# # in bottom row
	# for x in range(4, 495):
	# 	loc = coords_to_location(x, 139, map)
	# 	reverse_arc(digraph, loc+1, loc)
	# # in right column
	# for y in range(4, 135):
	# 	loc = coords_to_location(499, y, map)
	# 	reverse_arc(digraph, loc+500, loc)

	write_barriers(digraph)
	n_scc = nx.number_strongly_connected_components(digraph)
	if n_scc != 1:
		print(f"Error: the graph is not strongly connected: there are {n_scc} components")
		scc = [f"(v:{list(c)[0]}, size:{len(c)})" for c in sorted(nx.strongly_connected_components(digraph), key=len, reverse=True)]
		print(scc)
		# for c in sorted(nx.strongly_connected_components(digraph), key=len, reverse=True):
			# print(f"There is a component of size {len(c)} with vertex {list(c)[0]}")
	else:
		print("The digraph is strongly connected :D")


def make_arcs_game():
	"""Make the barriers for game"""
	fn = "brc202d.map"
	map = read_map(fn)
	digraph = nx.DiGraph()
	# Simple pattern
	for x in range(0, 529):
		for y in range(0, 480):
			loc = coords_to_location(x, y, map)
			if y+1 < map['height'] and map['pixels'][loc] and map['pixels'][loc+530]:
				if x%2 == 0:
					digraph.add_edge(loc, loc+530)
				else:
					digraph.add_edge(loc+530, loc)
			if x+1 < map['width'] and map['pixels'][loc] and map['pixels'][loc+1]:
				if y%2 == 0:
					digraph.add_edge(loc, loc+1)
				else:
					digraph.add_edge(loc+1, loc)
	# Two-lane highway
	reverse_path(digraph, 71839, 71724); reverse_path(digraph, 73314, 73429) 	# A
	reverse_path(digraph, 45334, 75014); reverse_path(digraph, 75015, 43215) 	# B
	for u, v in [(63880, 64410), (63882, 64412), (64417, 63887)]:
		reverse_arc(digraph, u, v)
	reverse_path(digraph, 46588, 46390); reverse_path(digraph, 47981, 48174) 	# C
	for u, v in [(44408, 44407), (45468, 45467)]:
		reverse_arc(digraph, u, v)
	reverse_path(digraph, 954, 74624); reverse_path(digraph, 75687, 957) 	 	# D
	reverse_path(digraph, 73087, 73031); reverse_path(digraph, 74624, 73619) 	# E
	for u, v in [(69880, 69879), (70940, 70939), (72000, 71999), (74649, 74650), (75709, 75710)]:
		reverse_arc(digraph, u, v)
	reverse_path(digraph, 68842, 148342); reverse_path(digraph, 147815, 68845)	# F
	for u, v in [(81030, 81560), (110708, 111238), (110710, 111240)]:
		reverse_arc(digraph, u, v)
	reverse_path(digraph, 122909, 122674); reverse_path(digraph, 124266, 124501) # G
	post_process(digraph)
	for u, v in [(83780, 83779), (33500, 32970), (16304, 16834), (100121, 100651), (28162, 28161), (131035, 131565), (76072, 75542), (69353, 69883)]:
		reverse_arc(digraph, u, v)
	reverse_path(digraph, 14182, 16305)


	# post_process(digraph)
	# Exceptions
	# reverse_path(digraph, 69354, 70414)
	# reverse_path(digraph, 76071, 75013)
	# reverse_path(digraph, 131034, 131565)
	# reverse_path(digraph, 28162, 28691)
	# reverse_path(digraph, 15779, 16835)
	# reverse_path(digraph, 33500, 32970)
	# reverse_path(digraph, 83780, 84309)
	# reverse_path(digraph, 100121, 100651)
	write_barriers(digraph)
	n_scc = nx.number_strongly_connected_components(digraph)
	if n_scc != 1:
		print(f"Error: the graph is not strongly connected: there are {n_scc} components")
		scc = [f"(v: {list(c)[0]}, size: {len(c)})" for c in sorted(nx.strongly_connected_components(digraph), key=len, reverse=True)]
		print(" ".join(scc))
		# for c in sorted(nx.strongly_connected_components(digraph), key=len, reverse=True):
			# print(f"There is a component of size {len(c)} with vertex {list(c)[0]}")
	else:
		print("The digraph is strongly connected :D")


def make_arcs_city():
	"""Make the barriers for city"""
	fn = "Paris_1_256.map"
	map = read_map(fn)
	digraph = nx.DiGraph()
	# Simple pattern
	for x in range(0, 256):
		for y in range(0, 256):
			loc = coords_to_location(x, y, map)
			if y+1 < map['height'] and map['pixels'][loc] and map['pixels'][loc+256]:
				if x%2 == 0:
					digraph.add_edge(loc, loc+256)
				else:
					digraph.add_edge(loc+256, loc)
			if x+1 < map['width'] and map['pixels'][loc] and map['pixels'][loc+1]:
				if y%2 == 0:
					digraph.add_edge(loc, loc+1)
				else:
					digraph.add_edge(loc+1, loc)
	post_process(digraph)
	# Exceptions
	reverse_path(digraph, 5119, 4861)
	reverse_path(digraph, 3055, 3048)
	reverse_path(digraph, 64981, 64721)
	reverse_path(digraph, 58539, 58283)
	reverse_path(digraph, 65336, 65079)
	reverse_path(digraph, 19451, 19196)
	reverse_path(digraph, 6653, 6395)
	reverse_path(digraph, 64980, 64981)
	reverse_path(digraph, 65192, 65447)
	reverse_path(digraph, 5375, 5118)
	write_barriers(digraph)
	n_scc = nx.number_strongly_connected_components(digraph)
	if n_scc != 1:
		print(f"Error: the graph is not strongly connected: there are {n_scc} components")
		scc = [f"(v: {list(c)[0]}, size: {len(c)})" for c in sorted(nx.strongly_connected_components(digraph), key=len, reverse=True)]
		print(" ".join(scc))
		# for c in sorted(nx.strongly_connected_components(digraph), key=len, reverse=True):
			# print(f"There is a component of size {len(c)} with vertex {list(c)[0]}")
	else:
		print("The digraph is strongly connected :D")


def make_arcs_random():
	"""Make the barriers for random"""
	fn = "random-32-32-20.map"
	map = read_map(fn)
	digraph = nx.DiGraph()
	# Simple pattern
	for x in range(0, map['width']):
		for y in range(0, map['height']):
			loc = coords_to_location(x, y, map)
			if y+1 < map['height'] and map['pixels'][loc] and map['pixels'][loc+32]:
				if x%2 == 0:
					digraph.add_edge(loc, loc+32)
				else:
					digraph.add_edge(loc+32, loc)
			if x+1 < map['width'] and map['pixels'][loc] and map['pixels'][loc+1]:
				if y%2 == 0:
					digraph.add_edge(loc, loc+1)
				else:
					digraph.add_edge(loc+1, loc)
	post_process(digraph)
	# Exceptions
	reverse_path(digraph, 1022, 989)
	reverse_path(digraph, 603, 635)
	reverse_path(digraph, 973, 976)
	reverse_path(digraph, 412, 415)
	reverse_path(digraph, 540, 284)
	reverse_path(digraph, 768, 930)
	reverse_path(digraph, 419, 450)
	write_barriers(digraph)
	n_scc = nx.number_strongly_connected_components(digraph)
	if n_scc != 1:
		print(f"Error: the graph is not strongly connected: there are {n_scc} components")
		scc = [f"(v: {list(c)[0]}, size: {len(c)})" for c in sorted(nx.strongly_connected_components(digraph), key=len, reverse=True)]
		print(" ".join(scc))
		# for c in sorted(nx.strongly_connected_components(digraph), key=len, reverse=True):
			# print(f"There is a component of size {len(c)} with vertex {list(c)[0]}")
	else:
		print("The digraph is strongly connected :D")
	eval_orientation(digraph)



def eval_orientation(digraph):
	"""Evaluate a strong orientation"""
	# Adjacent nodes must be at distance <= 3
	distant_neighbors = []
	for e in digraph.edges:
		dist = nx.shortest_path_length(digraph, source=e[1], target=e[0])
		if dist > 3:
			distant_neighbors.append({'edge': e, 'dist': dist})
	distant_neighbors.sort(key=lambda x: x['dist'], reverse=True)
	print("distant neighbors", distant_neighbors)
	# Not two arrows coming from opposite directions
	crushing_nodes = []
	for node in digraph.nodes:
		in_nodes = list(digraph.predecessors(node))
		for n1, n2 in itertools.combinations(in_nodes, 2):
			if abs(node - n1) == abs(node - n2):
				crushing_nodes.append(node)
	crushing_nodes.sort()
	print("confluence nodes: ", crushing_nodes)
	# See the diameter
	print("max distance: ", approx_diameter(digraph))





if __name__ == "__main__":
	make_arcs_warehouse()
	# make_arcs_sortation()
	# make_arcs_city()
	# make_arcs_random()
	# make_arcs_game()