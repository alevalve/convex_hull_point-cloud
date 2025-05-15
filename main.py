import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull
from hull_method import HullRemoval
from metrics import ChamferDistance, NormalConsistency
import csv
import argparse

# Process point clouds

def point_cloud(raw):

    raw_pc = o3d.io.read_point_cloud(raw)
    hull_remover = HullRemoval(raw_pc)
    _, _, clean_pc  = hull_remover.forward()

    o3d.io_write_point_cloud("convex_pc.ply", clean_pc)
    
    return raw_pc, clean_pc

def evaluation(raw_pc, clean_pc, max_points):

    CD = ChamferDistance(raw_pc, clean_pc, max_points)
    chamfer_distance = CD.forward()
    NC = NormalConsistency(raw_pc, clean_pc, max_points)
    normal_consistency = NC.forward()

    return chamfer_distance, normal_consistency

def write_csv(raw_pc, clean_pc, max_points):

    chamfer, normal = evaluation(raw_pc, clean_pc, max_points)

    columns = ['original_size', 'hull_size', 'chamfer_distance', 'normal_consistency']

    with open('point_cloud_results.csv', 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=columns)
        writer.writeheader()

        row_dict = {}
        
        row_dict['normal_consistency'] = normal['average']
        row_dict['chamfer_distance'] =  chamfer['chamfer']
        row_dict['original_size'] = len(raw_pc.points)
        row_dict['hull_size'] = len(clean_pc.points)

        writer.writerow(row_dict)
 


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate point cloud metrics.")
    parser.add_argument('--input_path', type=str, required=True, help='Path to input point cloud')
    parser.add_argument('--max_points', type=int, default=10000, help='Maximum number of points to evaluate')

    args = parser.parse_args()

    raw_pc, clean_pc = point_cloud(args.input_path)
    write_csv(raw_pc, clean_pc, args.max_points)






    


    