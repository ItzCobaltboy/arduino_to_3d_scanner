import numpy as np
import pandas as pd
import trimesh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
import itertools

def visualize_data(df):
    # Create 3D scatter plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    scatter = ax.scatter(df['x'], 
                        df['y'], 
                        df['z'],
                        c=df['z'],  # Color by z-coordinate
                        cmap='viridis',
                        alpha=0.6)
    
    plt.colorbar(scatter)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Point Cloud Visualization')
    
    plt.show()

def convert_to_stl(df, output_file='output.stl'):
    # Convert DataFrame to numpy array
    points = df[['x', 'y', 'z']].values
    
    # Create convex hull
    hull = ConvexHull(points)
    
    # Create mesh using trimesh
    mesh = trimesh.Trimesh(vertices=points, faces=hull.simplices)
    
    # Save as STL
    mesh.export(output_file)
    print(f"STL file saved as: {output_file}")
    
    return mesh

if __name__ == "__main__":

    df = pd.read_csv('point_cloud.csv')
    
    # Display first few rows of the data
    print("\nFirst few rows of the point cloud data:")
    print(df.head())
    
    # Show basic statistics
    print("\nData statistics:")
    print(df.describe())
    
    # Visualize the point cloud
    print("\nDisplaying visualization...")
    visualize_data(df)
    
    # Convert to STL
    print("\nConverting to STL...")
    try:
        mesh = convert_to_stl(df, 'point_cloud.stl')
        print("Conversion successful!")
        
        # Save the point cloud data to CSV
        # df.to_csv('point_cloud_2.csv', index=False)
        print("Point cloud data saved to CSV")
        
        # Display mesh information
        print("\nMesh information:")
        print(f"Number of vertices: {len(mesh.vertices)}")
        print(f"Number of faces: {len(mesh.faces)}")
        
    except Exception as e:
        print(f"Error during conversion: {str(e)}")
