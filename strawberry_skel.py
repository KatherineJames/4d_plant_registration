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

from data_loader import filter_xyz


def experiment_1_semantic():
    selected_cats = ["petiole"]
    pcd = filter_xyz("demo", selected_cats)
        
    pcd.paint_uniform_color([1, 0.706, 0])
    data = np.array(pcd.points)

    # compute skel
    cwise_skeleton_nodes = som.getSkeleton([data])
    graph = som.getGraph(cwise_skeleton_nodes, data)

    # convert this graph to skeleton class
    S = utils.convert_to_skeleton_class(cwise_skeleton_nodes, graph)

    edges = S.edges
    nodes = S._XYZ 
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
            nodes), lines=o3d.utility.Vector2iVector(edges))

    o3d.visualization.draw_geometries([pcd, line_set])
    # o3d.io.write_line_set(
    #         f"results_seedling4/skel_files/p2_cyl.ply", line_set, write_ascii=True)


def experiment_2_instance():
    selected_cats = ["petiole"] 
    pcds = filter_xyz("demo", selected_cats, filter_instances=True)
    for pcd in pcds:     
        data = np.array(pcd.points)
        cwise_skeleton_nodes = som.getSkeleton([data])
        graph = som.getGraph(cwise_skeleton_nodes, data)

        # convert this graph to skeleton class
        S = utils.convert_to_skeleton_class(cwise_skeleton_nodes, graph)

        edges = S.edges
        nodes = S._XYZ    
        line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
                nodes), lines=o3d.utility.Vector2iVector(edges))

        o3d.visualization.draw_geometries([pcd, line_set])

if __name__ == "__main__":

    # Case 1: Semantic segmentation
    experiment_1_semantic()  
    
    # Case 2: Instance segmentation 
    experiment_2_instance()