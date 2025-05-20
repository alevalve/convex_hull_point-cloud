  # Convex-Guided Outlier Removal for 3D Point Clouds

  ![Pipeline_page-0001](https://github.com/user-attachments/assets/941c6b56-fff9-415e-8bae-5caa95404502)

  The Convex-Guided Outlier Removal method estimates a convex hull around the input point cloud and removes points that lie too close to or outside the boundary. 
  This is particularly useful for eliminating surface noise or spurious outliers from Structure-from-Motion (SfM) outputs.

  This repository provides a pipeline for **outlier removal** and **quality evaluation** of 3D point clouds using convex hull–based filtering. It includes:

  - **Convex hull–based filtering** to remove boundary outliers
  - **Chamfer Distance** and **Normal Consistency** evaluation metrics

  ---

  ## 📁 Project Structure

- main.py # Entry point: loads, filters, evaluates, and saves results 

- hull_method.py # HullRemoval class for convex hull filtering

- metrics.py # ChamferDistance and NormalConsistency metrics

## Usage

### 1. Install Requirements

Ensure you have Python 3.8+ and install dependencies:

```bash
pip install open3d numpy scikit-learn open-3d scipy
```

### 2 Run Hull filtered

```bash
python main.py --input_path path/to/point_cloud.ply --max_points 10000
```

This will:
- Load the raw point cloud
- Apply convex filtering
- Save the filtered point cloud to convex_pc.ply
- Compute Chamfer Distance and Normal Consistency
- Save numerical results

### 3 Evaluation Metrics

a) Chamfer Distance
Computes average closest-point distances between the raw and filtered point clouds in both directions using a KD-tree.

b) Normal Consistency
Measures cosine similarity between normals at nearest neighbors in both directions (raw → clean and clean → raw). Normals are estimated if not present.


## Author:
Alexander Valverde
Feel free to reach out for academic or research collaboration.





