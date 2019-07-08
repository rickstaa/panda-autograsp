#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" Script to view npy arrays in the spyder IDE

Created on Mon Jul  8 10:21:26 2019

@author: ricks
"""

## Imports ##
import numpy as np
from tkinter.filedialog import askopenfilename

## Ask for .npy file  ##
filename = askopenfilename()

## Get .npy array ##
data = np.load(filename)