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

def skeletonise(path):    
    pcds = []
    pcd_names = []
    files = sorted(os.listdir(data_dir))    

    for file in files:   
        
        if file[-4:] != ".ply" or os.path.isdir(f"{path}/" + file):
            continue        
         
        pcds.append(o3d.io.read_point_cloud(f"{path}/" + file))     
        pcd_names.append(file)    
  
    
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
         
        print(f"Results/{pcd_names[i]}") 
        o3d.io.write_line_set(f"Results/{pcd_names[i][:-4]}.ply", line_set, write_ascii=True)
            

if __name__ == "__main__":

    data_dir = "/home/katherine/Documents/Berry4D/petiole_instances"      
    
    if not os.path.exists("Results"):            
        os.mkdir("Results")    

    skeletonise(data_dir)
      
