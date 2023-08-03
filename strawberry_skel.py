#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This script provides a demo for extracting the skeleton structure from the plant point cloud.
"""
import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
from joblib import load
import matplotlib.pyplot as plt
import visualize as vis
import somSkeleton as som
import skeleton as skel
import utils
import os

from data_loader import filter_xyz, parse_to_xyz


def skeltonise(sample):
    selected_cats = ["petiole"] 
    # pcds = filter_xyz(sample, selected_cats, filter_instances=True)
    pcds = []
    pcd_names = []
    files = os.listdir(f"Data/young/{sample}/")
    for file in files:   
        
        if file[-4:] != ".ply" or os.path.isdir(f"Data/young/{sample}/" + file):
            continue        
         
        pcds.append(o3d.io.read_point_cloud(f"Data/young/{sample}/" + file))
        pcd_names.append(file)
   
    print
    for i, pcd in enumerate(pcds):     
        data = np.array(pcd.points)
        cwise_skeleton_nodes = som.getSkeleton([data])
        graph = som.getGraph(cwise_skeleton_nodes, data)

        # convert this graph to skeleton class
        S = utils.convert_to_skeleton_class(cwise_skeleton_nodes, graph)

        edges = S.edges
        nodes = S._XYZ    
        line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
                nodes), lines=o3d.utility.Vector2iVector(edges))

        # o3d.visualization.draw_geometries([pcd, line_set])        
        o3d.io.write_line_set(f"Results/som/{sample}/{pcd_names[i]}", line_set, write_ascii=True)
            

if __name__ == "__main__":

    samples = ["1","2","3","4","5","6","7","8","9","10"]
    for sample in samples:       
        if not os.path.exists("Results/som"):
            os.mkdir("Results/som")

        if not os.path.exists(f"Results/som/{sample}"):
            os.mkdir(f"Results/som/{sample}")
     
        if not os.path.exists(f"Results/som/{sample}"):
            os.mkdir(f"Results/som/{sample}")

        skeltonise(sample)
      
