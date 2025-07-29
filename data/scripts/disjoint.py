import numpy as np
from scipy.cluster.hierarchy import DisjointSet
from scipy.spatial import KDTree
from tqdm import tqdm

def build_disjoint_sets(points, threshold):
    """
    Build disjoint sets from a list of points using the Manhattan distance metric and a threshold.
    Utilizes KDTree for efficient neighbor searching.
    
    Parameters:
    - points (list of tuples): List of points, each tuple represents a point (e.g., [(x1, y1), (x2, y2), ...]).
    
    Returns:
    - disjoint_sets (dict): A dictionary where keys are component labels and values are lists of points in each set.
    """
    # Initialize DisjointSet
    ds = DisjointSet(range(len(points)))
    
    # Convert points to a NumPy array for KDTree
    points_array = np.array(points)
    
    # Build KDTree for efficient neighbor searching
    data_tree = KDTree(points_array)
    query_tree = KDTree(points_array)
    query_results = query_tree.query_ball_tree(data_tree, threshold)
    
    for i, neighbors in enumerate(query_results):
        for j in neighbors:
            if i < j:
                # Only merge in one direction to avoid redundant checks
                ds.merge(i, j)
    
    return ds

def add_to_disjoint_sets(ds, existing_points, new_points, threshold):
    """
    Efficiently add new points to an existing DisjointSet structure.
    Optimized by assuming all points in the same set are already adjacent,
    and only checking new points against existing sets once.
    
    Parameters:
    - ds (DisjointSet): Existing DisjointSet object
    - existing_points (list of tuples): List of points used to create the DisjointSet
    - new_points (list of tuples): List of new points to add
    - threshold (float): Distance threshold for considering points as neighbors
    
    Returns:
    - ds (DisjointSet): The updated DisjointSet object with new points added
    """
    
    n_existing = len(existing_points)
    n_new = len(new_points)
    
    # Extend the DisjointSet to include new points
    for i in range(n_existing, n_existing + n_new):
        ds.add(i)
    
    # Convert to numpy arrays
    existing_points_array = np.array(existing_points)
    new_points_array = np.array(new_points)
    
    # Create KDTree for existing points
    existing_tree = KDTree(existing_points_array)
    
    # Process each new point
    for i, new_point in enumerate(tqdm(new_points, desc="Adding new points")):
        new_idx = n_existing + i
        
        # Find existing points within threshold
        neighbors = existing_tree.query_ball_point(new_point, threshold)
        
        # If there are neighbors, merge with the first one
        # (Since all points in a set are assumed adjacent, we only need to merge with one representative)
        if neighbors:
            ds.merge(new_idx, neighbors[0])
    
    # Create KDTree for new points
    new_tree = KDTree(new_points_array)
    
    # Check adjacency between new points
    for i in range(n_new):
        new_idx_i = n_existing + i
        
        # Find neighboring new points within threshold
        neighbors = new_tree.query_ball_point(new_points_array[i], threshold)
        
        # Merge with higher index neighbors only (to avoid redundant checks)
        for j in neighbors:
            if j > i:  # Only consider higher indices
                new_idx_j = n_existing + j
                ds.merge(new_idx_i, new_idx_j)
    
    return ds