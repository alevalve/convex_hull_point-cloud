import numpy as np
import open3d as o3d
import time 
from sklearn.neighbors import KDTree
import csv

np.random.seed(42)

class ChamferDistance:
    def __init__(self, source, clean, max_points):
        self.source_points = source
        self.clean_points = clean
        self.max_points = max_points

    
    def chamfer(self, source_points, clean_points, max_points):
        source_points = np.asarray(self.source_points.points)
        clean_points = np.asarray(self.clean_points.points)

        if source_points.shape[0] > self.max_points:
            indices = np.random.choice(source_points.shape[0], self.max_points, replace=False)
            source_points = source_points[indices]
        
        if clean_points.shape[0] > self.max_points:
            indices = np.random.choice(clean_points.shape[0], self.max_points, replace=False)
            clean_points = clean_points[indices]
        
        source_tree = KDTree(source_points)
        clean_tree = KDTree(clean_points)


        # Calculate source-clean
        distances_s_c, _= clean_tree.query(source_points, k=1)
        source_clean_distances = distances_s_c.flatten()

        # Calcualte clean-source
        distances_c_s, _ = source_tree.query(clean_points, k=1)
        clean_source_distances = distances_c_s.flatten()

        # Obtain mean of both 
        chamfer_s_c = np.mean(source_clean_distances)
        chamfer_c_s = np.mean(clean_source_distances)

        # Obtain chamfer distance 
        chamfer = (chamfer_s_c + chamfer_c_s) / 2

        return {
            'raw_to_clean': chamfer_s_c,
            'clean_to_raw': chamfer_c_s,
            'chamfer': chamfer
        }


    def forward(self):
        chamfer_distance = self.chamfer(self.source_points, self.clean_points, self.max_points)
        return chamfer_distance


class NormalConsistency:
    def __init__(self, source, clean, max_points):
        self.source = source
        self.clean = clean
        self.max_points = max_points

    
    def normal(self, source, clean, max_points):

        if not source.has_normals():
            self.source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        if not clean.has_normals():
            self.clean.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        # Normalize normals
        self.source.normalize_normals()
        self.clean.normalize_normals()

        # Convert in numpy arrays
        source_points = np.asarray(self.source.points)
        clean_points = np.asarray(self.clean.points)
        source_normals = np.asarray(self.source.normals)
        clean_normals = np.asarray(self.clean.normals)

        if source_points.shape[0] > self.max_points:
            indices = np.random.choice(source_points.shape[0], self.max_points, replace=False)
            source_points = source_points[indices]
            source_normals = source_normals[indices]
        
        if clean_points.shape[0] > self.max_points:
            indices = np.random.choice(clean_points.shape[0], self.max_points, replace=False)
            clean_points = clean_points[indices]
            clean_normals = clean_normals[indices]
        

        # Build KDtrees
        source_tree = KDTree(source_points)
        clean_tree = KDTree(clean_points)

        # Find nearest neighbors s-c
        _, indices_s_c = clean_tree.query(source_points, k=1)
        indices_s_c = indices_s_c.flatten()

        # Find nearest neighbors c-s
        _, indices_c_s = source_tree.query(clean_points, k=1)
        indices_c_s = indices_c_s.flatten()

        # Calculate normal consistency 
        corresponding_clean_normals = clean_normals[indices_s_c]
        dot_product_s_c = np.sum(clean_normals * corresponding_clean_normals, axis=1)

        # Use absolute values if normals are not consistent

        abs_dot_s_c = np.abs(dot_product_s_c)
        normal_consistency_s_c = np.mean(abs_dot_s_c)

        # Calculate normal consitency: clean to source
        corresponding_source_normals = source_normals[indices_c_s]
        dot_product_c_s = np.sum(source_normals * corresponding_source_normals, axis=1)
        abs_dot_c_s = np.abs(dot_product_c_s)
        normal_consistency_c_s = np.mean(abs_dot_c_s)

        # Normal Consistency
        normal = (normal_consistency_s_c + normal_consistency_c_s) / 2

        return {
        'raw_to_clean': normal_consistency_s_c,
        'clean_to_raw': normal_consistency_c_s,
        'average': normal
        }

    
    def forward(self):
        normal_consistency = self.normal(self.source, self.clean, self.max_points)
        return normal_consistency

    



        


        


        
        
