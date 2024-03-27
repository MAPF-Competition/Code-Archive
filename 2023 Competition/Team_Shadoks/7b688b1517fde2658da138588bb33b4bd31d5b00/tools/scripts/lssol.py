#!/usr/bin/env python3
"""
Script for printing simple information about several solutions
"""

import json
import sys


def read_solution(fn):
	sol = {}
	sol["fn"] = fn
	sol["map"] = which_map(fn)
	sol["timedate"] = fn.split("-")[-1].split(".")[0]
	with open(fn, 'r') as json_file:
		data = json.load(json_file)
		sol["teamSize"] = data["teamSize"]
		sol["numTaskFinished"] = data["numTaskFinished"]
		sol["makespan"] = data["makespan"]
	return sol

def which_map(fn):
	if "random" in fn:
		return "random   "
	if "sortation" in fn:
		return "sortation"
	if "warehouse" in fn:
		return "warehouse"
	if "paris" in fn:
		return "paris    "
	if "brc202d" in fn:
		return "brc202d  "
	return "unknown"


if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("Usage: python3 lssol.py <filename>")
	else:
		solutions = [read_solution(arg) for arg in sys.argv[1:]]
		solutions.sort(key=lambda x:x['timedate'])
		for sol in solutions:
			print(f"map:{sol['map']} agents:{sol['teamSize']:5d} \tsteps:{sol['makespan']:4d} tasks:{sol['numTaskFinished']:6d} ({sol['fn']})")
