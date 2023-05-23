# -*- coding: utf-8 -*-
"""
Created on Mon May 22 19:51:29 2023

@author: User123
"""

from model import ConvNet
import numpy as np
import torch
import matplotlib.pyplot as plt

model = ConvNet(2)
I = np.random.rand(3,64,64)
I.dtype=np.float32
I = torch.tensor(I)

Y = model(I)

