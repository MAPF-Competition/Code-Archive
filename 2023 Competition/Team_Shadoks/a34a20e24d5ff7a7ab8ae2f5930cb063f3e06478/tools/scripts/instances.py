"""
Script for generating new instances
"""

import random

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
	remove_exceptions(map)
	print(f"Removed points: {map['removed']}")
	return map

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

def make_agents(fn, n=1000):
	"""Make n different agents in the largest connected component of the map."""
	map = read_map(fn)
	locations = [i for i, pixel in enumerate(map['pixels']) if pixel]
	random.shuffle(locations)
	name = {
		"random-32-32-20.map": "random",
		"brc202d.map": "brc202d",
		"Paris_1_256.map": "paris",
		"sortation_large.map": "sortation_large",
		"warehouse_large.map": "warehouse_large"
	}
	with open(f"{name[fn]}_{n}.agents", 'w') as f:
			f.write(f"{n}\n")
			for loc in locations[:n]:
				f.write(f"{loc}\n")
	print("Agents done")

def make_tasks(fn, n=1000):
	"""Make n tasks in the largest connected component of the map."""
	map = read_map(fn)
	locations = [i for i, pixel in enumerate(map['pixels']) if pixel]
	name = {
		"random-32-32-20.map": "random",
		"brc202d.map": "brc202d",
		"Paris_1_256.map": "paris",
		"sortation_large.map": "sortation_large",
		"warehouse_large.map": "warehouse_large"
	}
	with open(f"{name[fn]}_custom.tasks", 'w') as f:
			f.write(f"{n}\n")
			for i in range(n):
				f.write(f"{random.choice(locations)}\n")
	print("Tasks done")


if __name__ == "__main__":
	# fn = "random-32-32-20.map"
	fn = "brc202d.map"
	# fn = "Paris_1_256.map"
	# fn = "sortation_large.map"
	# fn = "warehouse_large.map"

	make_agents(fn, 6500)
	make_tasks(fn, 40000)
