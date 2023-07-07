# -*- coding: utf-8 -*-
"""
Created on Fri Jul  7 11:13:29 2023

@author: albertonbloemer
"""


class SimulationData:
    def __init__(self, files_in_dir):
        for string in files_in_dir:
            string = string.split(".csv")[0]
            setattr(self, string, 0)
    def __repr__(self):
        return " ".join([str(getattr(self, i)) for i in [x for x in dir(self) if not x.startswith('__')]])
