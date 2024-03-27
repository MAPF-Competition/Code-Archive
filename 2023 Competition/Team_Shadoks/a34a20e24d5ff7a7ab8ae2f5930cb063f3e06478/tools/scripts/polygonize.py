"""
Transform a map into an SVG path
"""

import json


def read_map(fn):
	"""Read a file in .map format."""
	map = {"width": 0, "height": 0, "pixels": [], "location_at_index": []}
	with open(fn) as f:
		lines = [line.rstrip() for line in f]
	map["height"] = int(lines[1].split()[1])
	map["width"] = int(lines[2].split()[1])
	location = 0
	for line in lines[4:]:
		for char in line:
			if char in ['@', 'T']:
				map["pixels"].append(True) # an obstacle
			else:
				map["pixels"].append(False) # not an obstacle
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
	return [x for x in neighs if not map['pixels'][x]]

def move(map, location, dir):
	p = location_to_coords(map, location)
	if dir == 0:
		return location + 1 if p['x'] + 1 < map["width"] else -1
	if dir == 1:
		return location + map["width"] if p['y'] + 1 < map["height"] else -1
	if dir == 2:
		return location - 1 if 0 <= p['x'] - 1 else -1
	if dir == 3:
		return location - map["width"] if 0 <= p['y'] - 1 else -1


def neighbor(map, location, dir):
	next = move(map, location, dir)
	if next == -1:
		return False
	return map['pixels'][next]

def make_path(map):
	"""Get the polygons"""
	paths = []
	marked = set()
	for start in range(map["width"]*map["height"]):
		if map['pixels'][start] and not neighbor(map, start, 2) and start not in marked:
			# Start search 
			path = []
			p = location_to_coords(map, start)
			path.append(f"M{10*p['x']} {10*p['y']}")
			loc, dir = start, 2
			finished = False
			while not finished:
				if neighbor(map, loc, dir):
					loc = move(map, loc, dir)
					dir = (dir + 1) % 4
				else:
					if dir == 0:
						path.append("v-10")
					elif dir == 1:
						path.append("h10")
					elif dir == 2:
						path.append("v10")
						marked.add(loc)
					else:
						path.append("h-10")
					dir = (dir + 3) % 4
				if loc == start and dir == 2:
					path.append('z')
					finished = True
			# print(path)
			paths.append(" ".join(path))
	print(" ".join(paths))
	return paths
		
				

if __name__ == "__main__":
	# map_fn = "random-32-32-20.map"
	# map_fn = "Paris_1_256.map"
	# map_fn = "brc202d.map"
	# map_fn = "sortation_large.map"
	# map_fn = "warehouse_large.map"
	map_fn = "warehouse_small.map"

	map = read_map(map_fn)
	make_path(map)
	# write_directions(map, dist_fn)
