import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

file_path = "data/calibration_30000_samples/acc3_raw.csv"
data = pd.read_csv(file_path, header=None).values


target_points = 3000

def downsample_points(points, target_size):
    if len(points) <= target_size:
        return points 

    tree = cKDTree(points)
    distances, _ = tree.query(points, k=10) 
    density_scores = np.mean(distances, axis=1)

    sorted_indices = np.argsort(-density_scores)
    points =  points[sorted_indices[target_points*3:]]

    tree = cKDTree(points)
    distances, _ = tree.query(points, k=10)
    density_scores = np.mean(distances, axis=1)

    sorted_indices = np.argsort(-density_scores)
    return points[sorted_indices[:target_points]]

filtered_data = downsample_points(data, target_points)

output_file = "data/calibration_30000_samples/acc3_filtered.csv"
pd.DataFrame(filtered_data).to_csv(output_file, index=False, header=False)
