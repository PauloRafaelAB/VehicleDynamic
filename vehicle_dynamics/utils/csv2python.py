# -*- coding: utf-8 -*-
"""
Created on Fri Jul  7 09:36:25 2023

@author: albertonbloemer
"""
from collections import namedtuple
import csv
import numpy as np
import os
from vehicle_dynamics.utils.SimulationData import SimulationData
import pickle

# Specify the path to your CSV file
data = {}

path = "../../exampledata/Step_steer_14_09/"

files_in_dir = os.listdir(path)

all_data_names = ""


for i, file_in_dir in enumerate(files_in_dir): 
    print(file_in_dir)
    with open(path + file_in_dir, 'r') as file:
        filename = file_in_dir.split(".csv")[0]
        if i == 0:
            reader = csv.reader(file)
            for j, row in enumerate(reader):
                data[j] = SimulationData(files_in_dir)
                setattr(data[j], filename, float(row[1]))
        else: 
            reader = csv.reader(file)
            for j, row in enumerate(reader):
                setattr(data[j], filename, float(row[1]))

with open(path + "SimulationData.pickle", "wb+") as handle:

    pickle.dump(data, handle)
