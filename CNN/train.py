# -*- coding: utf-8 -*-
"""
Created on Fri May 19 12:12:48 2023

@author: User123
"""

# Load in relevant libraries, and alias where appropriate
import torch
import torch.nn as nn
import torchvision
import torchvision.transforms as transforms
from MyConvNet import MyConvNet
import matplotlib.pyplot as plt

## DATASET LOADING ======================================
# A Neural Network needs to learn somehow the parameters of each layer
# To learn it will be feeded with labelled data (images) and its 
# parameters will be updated somehow according to this training data
# Also we need to evaluate how well performs on unseen data, so we also
# include "testing" data
def dataset_load(batch_size=32):
    # Use transforms.compose method to reformat images for modeling,
    # and save to variable all_transforms for later use
    all_transforms = transforms.Compose([transforms.Resize((32,32)),
                                         transforms.ToTensor(),
                                         transforms.Normalize(mean=[0.4914, 0.4822, 0.4465],
                                                              std=[0.2023, 0.1994, 0.2010])
                                         ])
    # Create Training dataset
    train_dataset = torchvision.datasets.CIFAR10(root = './data',
                                                 train = True,
                                                 transform = all_transforms,
                                                 download = True)

    # Create Testing dataset
    test_dataset = torchvision.datasets.CIFAR10(root = './data',
                                                train = False,
                                                transform = all_transforms,
                                                download=True)

    # Instantiate loader objects to facilitate processing
    train_loader = torch.utils.data.DataLoader(dataset = train_dataset,
                                               batch_size = batch_size,
                                               shuffle = True)


    test_loader = torch.utils.data.DataLoader(dataset = test_dataset,
                                               batch_size = batch_size,
                                               shuffle = True)
    return train_loader,test_loader
    





## TRAINING PROCEDURE FUNCTION =====================================
def main():
    # Define relevant variables for the ML task
    batch_size = 64         # How many samples we use to train our network in single pass
    num_classes = 10        # Desired number of classes
    learning_rate = 0.001   # How fast the neural network learn
    num_epochs = 20 # How many times the training will be repeated
    
    # Device will determine whether to run the training on GPU or CPU.
    # If your PC has a GPU the processing time required to train can be
    # significantly reduced
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    # Instanciate our model (with initial random values)
    model = MyConvNet(num_classes)

    # Set Loss function with criterion
    # The loss function is a measure of how well the CNN classify
    # data. The Cross entropy loss is a common used loss function
    # (https://ml-cheatsheet.readthedocs.io/en/latest/loss_functions.html)
    criterion = nn.CrossEntropyLoss()
    
    # Set optimizer of Stochastic Gradient Descent
    # This is the algorithm used for learning the parameters of each
    # layer on our CNN
    optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate, weight_decay = 0.005, momentum = 0.9)  
    
    # Load training/testing dataset
    train_dataset,test_dataset = dataset_load(batch_size)
    
    # Number of batches of batch_size samples per iteration
    total_step = len(train_dataset)
    
    
    #TRAINING PROCEDURE
    # We use the pre-defined number of epochs to determine how many iterations to train the network on
    # We train our CNN (update the parameters of each layer) iteratively using
    # num_epochs iterations
    loss_function_per_iter = []
    for epoch in range(num_epochs):
    	#Load in the data in batches using the train_loader object
        for i, (images, labels) in enumerate(train_dataset):  
            # Move tensors to the configured device (GPU or CPU)
            images = images.to(device)  # Samples
            labels = labels.to(device)  # Associated labels
            
            # Forward pass
            outputs = model(images)
            
            # Measure how well the CNN classify the training samples
            loss = criterion(outputs, labels)
            
            # Backward and optimize (updates the parameters of our CNN in 
            # dependence of the current loss value)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
        loss_function_per_iter.append(loss.item())
        print('Epoch [{}/{}], Loss: {:.4f}'.format(epoch+1, num_epochs, loss.item()))
    
    # If the code reach here, it means the CNN was trained successfully
    # Plot the loss function over iterations
    plt.plot(loss_function_per_iter)
    plt.xlabel("Iteration")
    plt.ylabel("Loss function")
    plt.show()
    
    # After the training is finished, we can evaluate the 
    # performance of the CNN using a metric called ACCURACY
    # That measures from the testing samples, how many were
    # classified correctly, for example if in the testing dataset
    # we have 100 samples, and after classify each sample we determine
    # that only 70 were classified correctly, the accuracy will be
    #
    # ACCURACY(%) = 100% * 70/100 = 70%
    #
    # Therefore, our model has an accuracy of 70%
    with torch.no_grad():
        correct = 0
        total = 0
        for images, labels in train_dataset:
            images = images.to(device)
            labels = labels.to(device)
            outputs = model(images)
            _, predicted = torch.max(outputs.data, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()
        
        print('Accuracy of the network on the {} train images: {} %'.format(50000, 100 * correct / total))