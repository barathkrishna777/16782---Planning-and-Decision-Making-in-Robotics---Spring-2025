import pandas as pd
import numpy as np

# Load the results.csv file
results_file = "results.csv"
results_df = pd.read_csv(results_file)

# Load the vertices.txt file
vertices_file = "vertices.txt"
with open(vertices_file, "r") as f:
    vertices = [int(line.strip()) for line in f.readlines()]

# Extract unique counts
num_planners = len(results_df["planner"].unique())
num_problems = len(results_df["problemIndex"].unique())
num_repeats = len(vertices) // (num_planners * num_problems)

# Ensure the number of vertices matches expectations
expected_entries = num_planners * num_problems * num_repeats
if len(vertices) != expected_entries:
    raise ValueError(f"Mismatch in expected vertices count: expected {expected_entries}, got {len(vertices)}")

# Reshape vertices into (num_planners, num_problems, num_repeats)
vertices = np.array(vertices).reshape((num_planners, num_problems, num_repeats))

# Compute mean and std for vertices
mean_vertices = vertices.mean(axis=2).flatten()
std_vertices = vertices.std(axis=2).flatten()

# Add to dataframe
results_df["meanVertices"] = mean_vertices
results_df["stdVertices"] = std_vertices

# Save updated results
updated_results_file = "updated_results.csv"
results_df.to_csv(updated_results_file, index=False)

print(f"Updated results saved to {updated_results_file}")
