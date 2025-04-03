# -*- coding: UTF-8 -*-
"""Warehouse Generator
Generate warehouse with pickup stations and storages (shelves)
"""

import os
import argparse
from typing import Dict
import copy
import yaml
import numpy as np


class WarehouseMapGenerator:
    """Class for the warehouse generator
    Inputs: width, height, number of pickup stations
    """
    def __init__(self, input_arg):
        print("===== Initialize warehouse generator =====")

        # Load configuration file
        self.config: Dict = {}
        if input_arg.config is not None:  # Load the yaml file
            config_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), input_arg.config)
            with open(config_dir, mode="r", encoding="utf-8") as fin:
                self.config = yaml.load(fin, Loader=yaml.FullLoader)
        else:  #  Load the input arguments
            self.config["mapWidth"] = input_arg.mapWidth
            self.config["mapHeight"] = input_arg.mapHeight
            self.config["output"] = input_arg.output
            self.config["stationNumber"] = input_arg.stationNumber
            self.config["pillarWidth"] = input_arg.pillarWidth
            self.config["operWidth"] = input_arg.operWidth
            self.config["stationConfig"] = input_arg.stationConfig
            self.config["storageSize"] = input_arg.storageSize
            self.config["stationDistance"] = input_arg.stationDistance
            self.config["corridorWidth"] = input_arg.corridorWidth


        assert self.config["mapWidth"] is not None
        assert self.config["mapHeight"] is not None
        assert self.config["output"] is not None
        assert self.config["stationNumber"] is not None
        assert self.config["pillarWidth"] is not None
        assert self.config["operWidth"] is not None
        assert self.config["stationConfig"] is not None
        assert self.config["storageSize"] is not None
        assert self.config["stationDistance"] is not None
        assert self.config["corridorWidth"] is not None


        # Initialize an empty warehouse
        self.warehouse = []
        for _ in range(self.config["mapHeight"]):
            tmp_list = ["." for _ in range(self.config["mapWidth"])]
            self.warehouse.append(tmp_list)
        self.max_raw = self.config["mapHeight"] - 1
        self.max_col = self.config["mapWidth"] - 1


        # Initialize the pillars
        for i in range(self.config["pillarWidth"]):
            for j in range(self.config["pillarWidth"]):
                self.warehouse[i][j] = "@"
                self.warehouse[self.max_raw-i][j] = "@"
                self.warehouse[i][self.max_col-j] = "@"
                self.warehouse[self.max_raw-i][self.max_col-j] = "@"

        # Initialize the station
        self.station = []
        self.cur_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(self.cur_path, self.config["stationConfig"]),
                  mode="r", encoding="utf-8") as fin:
            self.config["stationH"] = int(fin.readline().strip().split(" ")[1])
            self.config["stationW"] = int(fin.readline().strip().split(" ")[1])
            for _line_ in fin.readlines():
                self.station.append(list(_line_.strip()))
            assert len(self.station) == self.config["stationH"]
            assert len(self.station[0]) == self.config["stationW"]

        # Copy for record as block padding guide map
        self.warehouse_record = copy.deepcopy(self.warehouse)
        self.map_row_end = self.config["mapHeight"] - self.config["pillarWidth"]
        self.map_column_end = self.config["mapWidth"] - self.config["pillarWidth"]

        # Simplified version
        self.has_station = {"E": False, "S": False, "W": False, "N": False}


    def print_map(self):
        for i in range(self.config["mapHeight"]):
            print("".join(self.warehouse[i]))
        print("-------------------------------------------")


    def load_stations(self):
        sta_cnt = 0  # Station counter

        # Start from the west side
        cur_raw = self.config["pillarWidth"] + 1
        while (cur_raw + self.config["stationH"] + self.config["pillarWidth"]) < self.config["mapHeight"] \
            and sta_cnt < self.config["stationNumber"]:
            for _w_ in range(self.config["stationW"]):
                for _h_ in range(self.config["stationH"]):
                    self.warehouse[cur_raw+ _h_][_w_] = self.station[_h_][_w_]
            sta_cnt += 1
            self.has_station["W"] = True
            cur_raw += (self.config["stationDistance"] + self.config["stationH"])

        # Move to the south side
        cur_col = self.config["pillarWidth"] + 1
        while (cur_col + self.config["stationH"] + self.config["pillarWidth"]) < self.config["mapWidth"] \
            and sta_cnt < self.config["stationNumber"]:
            for _w_ in range(self.config["stationW"]):
                for _h_ in range(self.config["stationH"]):
                    self.warehouse[self.max_raw-_w_][cur_col+_h_] = self.station[_h_][_w_]
            sta_cnt += 1
            self.has_station["S"] = True
            cur_col += (self.config["stationDistance"] + self.config["stationH"])

        # Move to the east side
        cur_raw = self.max_raw - self.config["pillarWidth"] - 1
        while (cur_raw - self.config["stationH"] - self.config["pillarWidth"]) >= 0 \
            and sta_cnt < self.config["stationNumber"]:
            for _w_ in range(self.config["stationW"]):
                for _h_ in range(self.config["stationH"]):
                    self.warehouse[cur_raw - _h_][self.max_col-_w_] = self.station[_h_][_w_]
            sta_cnt += 1
            self.has_station["E"] = True
            cur_raw -= (self.config["stationDistance"] + self.config["stationH"])

        # Move to the north side
        cur_col = self.max_col - self.config["pillarWidth"] - 1
        while (cur_col - self.config["stationH"] - self.config["pillarWidth"]) >= 0 \
            and sta_cnt < self.config["stationNumber"]:
            for _w_ in range(self.config["stationW"]):
                for _h_ in range(self.config["stationH"]):
                    self.warehouse[_w_][cur_col-_h_] = self.station[_h_][_w_]
            sta_cnt += 1
            self.has_station["N"] = True
            cur_col -= (self.config["stationDistance"] + self.config["stationH"])


    def load_storages(self):
        start_raw = self.config["stationW"] + self.config["operWidth"] \
            if self.has_station["N"] else self.config["pillarWidth"]
        start_col = self.config["stationW"] + self.config["operWidth"] \
            if self.has_station["W"] else self.config["pillarWidth"]
        end_raw = self.config["mapHeight"]-self.config["stationW"]-self.config["operWidth"] \
            if self.has_station["S"] else self.config["mapHeight"] - self.config["pillarWidth"]
        end_col = self.config["mapWidth"]-self.config["stationW"]-self.config["operWidth"] \
            if self.has_station["E"] else self.config["mapWidth"] - self.config["pillarWidth"]

        for i in range(start_raw, end_raw,
                       self.config["storageSize"][1]+self.config["corridorWidth"]):
            for j in range(start_col, end_col,
                           self.config["storageSize"][0]+self.config["corridorWidth"]):
                if i+self.config["storageSize"][1] > end_raw or \
                    j+self.config["storageSize"][0] > end_col:
                    continue
                for ii in range(self.config["storageSize"][1]):
                    self.warehouse[i+ii][j-1] = "S"
                    self.warehouse[i+ii][j+self.config["storageSize"][0]] = "S"
                    for jj in range(self.config["storageSize"][0]):
                        self.warehouse[i+ii][j+jj] = "@"
                        if ii == 0:
                            self.warehouse[i+ii-1][j+jj] = "S"
                        elif ii == self.config["storageSize"][1]-1:
                            self.warehouse[i+ii+1][j+jj] = "S"

                #update map actual end row and column in self
                self.map_row_end = i+self.config["storageSize"][1]+1
                self.map_column_end = j+self.config["storageSize"][0]


    def check_fillin(self,row_start,row_end,column_start,column_end):
        for i in range(row_start,row_end):
            if i >= self.config["mapHeight"] or i < 0:
                return False
            for j in range(column_start,column_end):
                if j >= self.config["mapWidth"] or j < 0:
                    return False
                if self.warehouse_record[i][j] !=  "." and self.warehouse_record[i][j] !=  "S":
                    return False
        return True


    def save_map(self):
        with open(os.path.join(self.cur_path, self.config["output"]),
                  mode="w", encoding="utf-8") as fout:
            fout.write("type octile\n")
            fout.write(f"height {self.config['mapHeight']}\n")
            fout.write(f"width {self.config['mapWidth']}\n")
            fout.write("map\n")
            for i in range(self.config["mapHeight"]):
                fout.write("".join(self.warehouse[i])+"\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Take config.yaml as input!")
    parser.add_argument("--config", type=str, help="path to the configuration file", default=None)
    parser.add_argument("--mapWidth", type=int, default=33, help="map width")
    parser.add_argument("--mapHeight", type=int, default=57, help="map height")
    parser.add_argument("--output", type=str, default="output.map", help="output map file name")
    parser.add_argument("--stationConfig", type=str, default="picking_station.txt", help="station configuration")
    parser.add_argument("--stationNumber", type=int, default=np.inf, help="station number")
    parser.add_argument("--storageSize", type=int, nargs=2, default=[3,2], help="storage size")
    parser.add_argument("--stationDistance", type=int, default=2, help="Distance between stations")
    parser.add_argument("--corridorWidth", type=int, default=1, help="corridor width")
    parser.add_argument("--pillarWidth", type=int, default=4, help="pillar width")
    parser.add_argument("--operWidth", type=int, default=5, help="operation area width")

    args = parser.parse_args()

    generator = WarehouseMapGenerator(input_arg=args)
    generator.load_stations()
    generator.load_storages()
    generator.save_map()
