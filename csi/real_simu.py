# -*- coding: utf-8 -*-
"""
Created on Mon Nov 11 17:21:24 2019

@author: Weiyu_Lee
"""

import numpy as np
import os
import pandas as pd 
import scipy
import matplotlib.pyplot as plt

BUF_SIZE = 100
SUBCARRIER = 114

def _remove_nan(data):
    
    data = data.dropna(axis=0, how='any')
    
    return data

def _remove_inf(data):
    
    data = data.replace([np.inf, -np.inf], np.nan)
    
    return data

def _remove_zero(data):

    data = data.replace(0, np.nan)
    
    return data

def load_data(dataset_folder_path, label):
    """
    Load the region data
    """

    with open(os.path.join(dataset_folder_path, 'amp.csv'), newline='') as csvfile:            
        curr_data = pd.read_csv(csvfile, header=None, error_bad_lines=False)
        curr_data = _remove_inf(curr_data)
        curr_data = _remove_zero(curr_data)
        curr_data = _remove_nan(curr_data)
        curr_label = np.ones((curr_data.shape[0], 1), dtype=int) * label

    print("[{}]: {}".format(dataset_folder_path, curr_label.shape[0]))
            
    return curr_data.values, curr_label

def plot_cluster_cent(DUT_obj_dict, pos, title):
    
    plt.figure()
    
    for idx, p in enumerate(pos): 
        
        plt.plot(DUT_obj_dict[idx].cent_data, label=p)   

    plt.xlabel("Subcarrier")    
    plt.ylabel("Amp.")    
    plt.title(title)    
    plt.legend()    
    

class Classification(object):

    def __init__(self, train_data, label):

        self.cent_data = self._get_centroid(train_data)
        self.data_buffer = train_data
        self.buf_idx = 0
        self.label = label        
    
    def _get_centroid(self, input_data):
        
        cent_data = np.mean(input_data, axis=0)
    
        return cent_data
            
    def update_cent_data(self, input_data):
               
        self.data_buffer[self.buf_idx, :] = input_data
        self.cent_data = self._get_centroid(self.data_buffer)
        
        self.buf_idx = self.buf_idx + 1
        if self.buf_idx >= BUF_SIZE:
            self.buf_idx = 0
            
    def prediction(self):
        
        return self.label        


def test():
    #==============================================================================
    folder_name = "T:/data/wei/dataset/WIFI/csi_data/long_time_case/case4"
    pos = ["DUT_A", "DUT_B"]

    data = {}
    label = {}
    sample_num = 10000000
    for idx, p in enumerate(pos):
        
        data[idx], label[idx] = load_data(os.path.join(folder_name, p), idx)
        
        if sample_num > data[idx].shape[0]:
            sample_num = data[idx].shape[0]

    # Initial Obj and Cluster Centroid Positions
    DUT_obj_dict = {}
    for idx, p in enumerate(pos):
        
        DUT_obj_dict[idx] = Classification(data[idx][0:BUF_SIZE, :], idx)

    plot_cluster_cent(DUT_obj_dict, pos, "Initial Cluster Centroid")

    print("Start Clustering...")

    dist = np.zeros(len(pos), float)
    pred = np.zeros((len(pos), sample_num-BUF_SIZE), int)
    for s_idx in range(BUF_SIZE+1, sample_num):
        
        # Get data from each DUT. 
        for idx, p in enumerate(pos):
            
            curr_data = np.mean(data[idx][s_idx-10:s_idx+1, :], axis=0)
            
            # Calculate the distance between each DUT's centroid.
            for p_idx, p in enumerate(pos):    
            
                dist[p_idx] = scipy.spatial.distance.correlation(DUT_obj_dict[p_idx].cent_data, curr_data)
        
            # Find the neaest centroid
            min_idx = np.argmin(dist)
            
            # Update the centroid
            DUT_obj_dict[min_idx].update_cent_data(curr_data)
            
            # Make prediction. 
            pred[idx, s_idx-BUF_SIZE] = DUT_obj_dict[min_idx].prediction()

    plot_cluster_cent(DUT_obj_dict, pos, "Final Cluster Centroid")

    # Performance review
    error = 0
    for idx, p in enumerate(pos):
        
        error = error + sum(pred[idx, :] != idx)
        
    print("Error sample: {}".format(error))
    print("Total sample: {}".format(sample_num))
    print("Acc.: {}".format(1 - error/sample_num))
        
        
