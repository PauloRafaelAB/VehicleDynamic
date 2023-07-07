# -*- coding: utf-8 -*-
"""
Created on Fri Jul  7 09:36:25 2023

@author: albertonbloemer
"""
from collections import namedtuple
import csv
import numpy as np
import os 

# Specify the path to your CSV file
data = {}



#csv_file = 'C:/Users/albertonbloemer/Documents/T3.6_1/src_cm4sl/data_sim/2_acc_brake/brake_pedal.csv'
path = "C:/Users/albertonbloemer/Documents/T3.6_1/src_cm4sl/data_sim/2_acc_brake/"

files_in_dir = os.listdir(path)

all_data_names =""
    
class SimulationData:
    def __init__(self, files_in_dir):
        for string in files_in_dir:
            string = string.split(".csv")[0]
            setattr(self, string, 0)
    def __repr__(self):
        return " ".join([str(getattr(self, i)) for i in [x for x in dir(self) if not x.startswith('__')]])


for i, file_in_dir in enumerate(files_in_dir): 
    with open(path + file_in_dir, 'r') as file:
        filename = file_in_dir.split(".csv")[0]
        if i==0:
            reader = csv.reader(file)
            for row in reader:
                data[row[0]] = SimulationData(files_in_dir)
                setattr(data[row[0]], filename, float(row[1]))
        else: 
            reader = csv.reader(file)
            for row in reader:
                setattr(data[row[0]], filename, float(row[1]))
        

#print(data)

# Print the array
print(data)
