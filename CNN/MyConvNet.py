# -*- coding: utf-8 -*-
"""
Created on Fri May 19 12:11:53 2023

@author: User123
"""

# Load in relevant libraries, and alias where appropriate
import torch.nn as nn

## CNN Model ======================================

# Creating a CNN class
class MyConvNet(nn.Module):
	#  Determine what layers and their order in CNN object
    def __init__(self, num_classes):
        super(MyConvNet, self).__init__()
        # Convolutional layer uses a kernel of size [kernel_size x kernel_size]
        # Also assumes that the input image is a tensor of size [in_channels x H x W]
        # And output data is also a tensor of size [out_channels x H-kernel_size+1 x W-kernel_size+1]
        # 64 x 64 x 3
        self.conv_layer1 = nn.Conv2d(in_channels=3, out_channels=32, kernel_size=3)
        # 64 - 3 + 1 x 64 - 3 + 1 x 32 -> 62x62x32 
        self.conv_layer2 = nn.Conv2d(in_channels=32, out_channels=32, kernel_size=3)
        #  60 x 60 x 32
        # MaxPool is a layer that reduces every groups of [kernel_size x kernel_size] pixels
        # Into a single value, for instance if the input matrix X is
        #       [1  2   3   4   5 ]
        #       [6  7   8   9   10]
        # X =   [11 12  13  14  15],  and the MaxPool layer is a kernel of size 2 x 2
        #       [16 17  18  19  20]   every 2 x 2 submatrices of "X" will be reduced to
        #       [21 22  23  24  25]   a single value and will be the maximum element of such group
        #
        # For example, the first 2 x 2 group of pixels are the elements
        #   [1  2], After applying the MaxPool layer, the resulting value will be
        #   [6  7]  the maximum element of [1,2,6,7] = 7
        #
        # Horizontally, the next group of 2 x 2 pixels will be
        #   [2  3], that is, shifting in one position to the right
        #   [7  8]  and the output of this group will be "8"
        #
        # Vertically, if we are in the first group, the next group wiill be
        #   [6  7]  whose ouput value will be "12"
        #   [11 12]
        #
        # The stride refers to the number of shifts uses to change
        # to the next group, in the previous example it was 1
        # Finally if the input dimensions are N x H x W, the output
        # will be N x ceil((H - kernel_size) / stride) + 1 x ceil((W - kernel_size) / stride) + 1
        self.max_pool1 = nn.MaxPool2d(kernel_size = 2, stride = 2)
        #  (60-2)/2 + 1 x (60-2)/2 + 1 x 32 -> 30 x 30 x 32
        
        self.conv_layer3 = nn.Conv2d(in_channels=32, out_channels=64, kernel_size=3)
        # 30 - 3 + 1 -> 28 x 28 x 64
        self.conv_layer4 = nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3)
        # 26 x 26 x 64
        self.max_pool2 = nn.MaxPool2d(kernel_size = 2, stride = 2)
        # (26-2)/2 + 1 -> 13 x 13 x 64
        # Linear layers are modeled as a simple linear transformation
        #   y = Ax + b
        # The first parameter specifies the input dimension, while the 
        # second argument specifies the output dimension
        self.fc1 = nn.Linear(1600, 128)
        # 13*13*64 = 10816
        # ReLU is a non-linear transformation, e.g.
        # y = sin(y)
        self.relu1 = nn.ReLU()
        
        # The output layer takes the previous output dimension
        # (in this case 128 from the self.fc1 layer) and the output
        # dimension must be the number of desired classes
        self.fc2 = nn.Linear(128, num_classes)
        # 
    
    
    # Progresses data across layers    
    # This defines the data flow of the input until it reachs
    # the output. It follows the architecture defined in the 
    # "__init__" function
    def forward(self, x):
        out = self.conv_layer1(x)
        out = self.conv_layer2(out)
        out = self.max_pool1(out)
        
        out = self.conv_layer3(out)
        out = self.conv_layer4(out)
        out = self.max_pool2(out)
                
        out = out.reshape(out.size(0), -1)
        
        out = self.fc1(out)
        out = self.relu1(out)
        out = self.fc2(out)
        
        return out