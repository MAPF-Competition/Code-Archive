"""
Script for helping me setting the barriers for random.
"""

import json
import copy
import math

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

def make_barriers(fn, direction):
	map = read_map(fn)
	barriers = []
	if direction == 0:
		exceptions = [0, 89, 146, 157, 158, 217, 230, 251, 288, 313, 347, 348, 365, 480, 481, 482, 508, 509, 510]
		for y in range(map['height'] // 2):
			for x in range(map['width']):
				loc = coords_to_location(x, y, map)
				next = coords_to_location(x+1, y, map)
				if not map["pixels"][loc] and next is not None and not map["pixels"][next] and loc not in exceptions:
					barriers.append(loc)
	elif direction == 1:
		exceptions = [18, 22, 24, 54, 81, 347, 348, 511, 531, 603, 635, 727, 790, 795, 826, 827, 884, 914, 916, 948, 956, 976, 979, 989]
		for y in range(map['height']):
			for x in range(map['width'] // 2, map['width']):
				loc = coords_to_location(x, y, map)
				next = coords_to_location(x, y+1, map)
				if not map["pixels"][loc] and next is not None and not map["pixels"][next] and loc not in exceptions:
					barriers.append(loc)
	elif direction == 2:
		exceptions = [547,578,594,609,632,637,638,639,654,692,697,722,765,902,957,980,990,991,977,1019]
		for y in range(map['height'] // 2, map['height']):
			for x in range(map['width']):
				loc = coords_to_location(x, y, map)
				next = coords_to_location(x, y+1, map)
				if not map["pixels"][loc] and next is not None and not map["pixels"][next] and loc not in exceptions:
					barriers.append(loc)
	elif direction == 3:
		exceptions = [40, 41, 79, 170, 293, 512, 513, 544, 867, 868, 1002, 1005, 1006]
		for y in range(map['height']):
			for x in range(map['width'] // 2):
				loc = coords_to_location(x, y, map)
				next = coords_to_location(x, y-1, map)
				if not map["pixels"][loc] and next is not None and not map["pixels"][next] and loc not in exceptions:
					barriers.append(loc)
	print(barriers)


if __name__ == "__main__":
	fn = "random-32-32-20.map"
	# fn = "Paris_1_256.map"
	# fn = "brc202d.map"
	# fn = "warehouse_small.map"
	make_barriers(fn, 2)
