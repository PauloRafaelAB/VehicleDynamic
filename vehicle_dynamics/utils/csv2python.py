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


#csv_file = 'C:/Users/albertonbloemer/Documents/T3.6_1/src_cm4sl/data_sim/2_acc_brake/brake_pedal.csv'
path = "../../exampledata/2_acc_brake/"

files_in_dir = os.listdir(path)

all_data_names = ""


for i, file_in_dir in enumerate(files_in_dir): 
    with open(path + file_in_dir, 'r') as file:
        filename = file_in_dir.split(".csv")[0]
        if i == 0:
            reader = csv.reader(file)
            for row in reader:
                data[row[0]] = SimulationData(files_in_dir)
                setattr(data[row[0]], filename, float(row[1]))
        else: 
            reader = csv.reader(file)
            for row in reader:
                setattr(data[row[0]], filename, float(row[1]))

with open(path + "SimulationData.pickle", "wb+") as handle:

    pickle.dump(data, handle)
