# Import required libraries
import cv2  # OpenCV for image loading
import numpy as np  # Numerical operations
import matplotlib.pyplot as plt  # Plotting
import pandas as pd  # Saving to CSV
import networkx as nx  # Graph-based path generation
from matplotlib.lines import Line2D  # Custom legend items
from skimage.morphology import medial_axis, remove_small_objects, remove_small_holes, binary_closing  # Morphological tools
from scipy.ndimage import binary_erosion, binary_dilation  # Image preprocessing

# Compute curvature (κ) based on 3 points
def compute_curvature(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    # Numerator of curvature formula
    num = abs((x2 - x1)*(y3 - y2) - (x3 - x2)*(y2 - y1))
    # Denominator with Euclidean distances
    denom = np.sqrt(((x2 - x1)**2 + (y2 - y1)**2) * ((x3 - x2)**2 + (y3 - y2)**2))
    if denom == 0:
        return 0
    return num / denom

# Assign throttle value based on curvature
def assign_throttle(k):
    if k < 0.1:
        return 100  # High speed for straight sections
    elif k < 0.5:
        return 60   # Medium speed for moderate turns
    else:
        return 30   # Low speed for sharp curves

# Main function to process the map and extract middleline and speed points
def extract_middleline(pgm_path, resolution=0.05, origin=(-12.824999, -12.824999), spacing=0.2, curvature_window=3, save=True):
    # Load grayscale map
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not read image from: {pgm_path}")
   
    # Threshold to binary: only keep nearly white (track) pixels
    binary = img >= 250

    # Morphological cleaning to close gaps and smooth
    binary = binary_closing(binary, footprint=np.ones((3, 3)))
    binary = binary_erosion(binary, iterations=1)
    binary = binary_dilation(binary, iterations=1)
    binary = remove_small_objects(binary, min_size=100)
    binary = remove_small_holes(binary, area_threshold=100)

    # Plot and save the binary mask
    plt.figure(figsize=(6, 6))
    plt.imshow(binary, cmap='gray')
    plt.title("Step 1: Binary Track Mask")
    plt.axis('off')
    plt.savefig("step1_binary_track.png", dpi=200)
    plt.show()

    # Skeletonize the binary mask using medial axis (thinning to centerline)
    skeleton, _ = medial_axis(binary, return_distance=True)
   
    # Plot and save skeleton view
    plt.figure(figsize=(6, 6))
    plt.imshow(skeleton, cmap='gray')
    plt.title("Step 2: Medial Axis Skeleton")
    plt.axis('off')
    plt.savefig("step2_skeleton.png", dpi=200)
    plt.show()

    # Build a graph from the skeleton pixels
    def build_graph(skel):
        G = nx.Graph()
        h, w = skel.shape
        for y in range(h):
            for x in range(w):
                if skel[y, x]:
                    # Add 8-connected neighbors
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dy == 0 and dx == 0:
                                continue
                            ny, nx_ = y + dy, x + dx
                            if 0 <= ny < h and 0 <= nx_ < w and skel[ny, nx_]:
                                G.add_edge((y, x), (ny, nx_))
        return G

    # Build the graph and extract the main connected component
    G = build_graph(skeleton)
    if len(G.nodes) == 0:
        return [], []

    largest_cc = max(nx.connected_components(G), key=len)
    G_largest = G.subgraph(largest_cc).copy()

    # Do DFS to walk through the skeleton path
    start_node = list(G_largest.nodes)[0]
    dfs_path = list(nx.dfs_preorder_nodes(G_largest, source=start_node))
    sorted_path = np.array(dfs_path)

    # Close the loop by interpolating back to the start
    if len(sorted_path) > 2:
        start = sorted_path[0]
        end = sorted_path[-1]
        interp = np.linspace(end, start, num=int(np.linalg.norm(start - end)), dtype=int)
        sorted_path = np.vstack([sorted_path, interp])

    # Convert skeleton coordinates to world (metric) coordinates
    origin_x, origin_y = origin
    img_height = img.shape[0]
    world_coords = np.array([
        [origin_x + col * resolution, origin_y + (img_height - row) * resolution]
        for row, col in sorted_path
    ])

    # world_coords = np.array([
    #     [origin_x + col * resolution, origin_y + row * resolution]
    #     for row, col in sorted_path
    # ])

    # Downsample to maintain uniform spacing between waypoints
    spaced_coords = [world_coords[0]]
    last = world_coords[0]
    for pt in world_coords[1:]:
        if np.linalg.norm(pt - last) >= spacing:
            spaced_coords.append(pt)
            last = pt
    spaced_coords = np.array(spaced_coords)

    # Compute curvature and throttle for each waypoint
    throttle_values = []
    for i in range(len(spaced_coords)):
        idx_prev = (i - curvature_window) % len(spaced_coords)
        idx_next = (i + curvature_window) % len(spaced_coords)
        p1 = spaced_coords[idx_prev]
        p2 = spaced_coords[i]
        p3 = spaced_coords[idx_next]
        k = compute_curvature(p1, p2, p3)
        throttle = assign_throttle(k)
        throttle_values.append(throttle)

    # Assign a color to each waypoint based on its throttle value
    throttle_colors = []
    for t in throttle_values:
        if t == 100:
            throttle_colors.append('red')
        elif t == 60:
            throttle_colors.append('yellow')
        else:
            throttle_colors.append('green')

    # Convert spaced world coordinates back to pixel coordinates for plotting
    pix_coords = np.array([
        [(x - origin_x) / resolution, (y - origin_y) / resolution]
        for x, y in spaced_coords
    ])

    # Create the final plot
    plt.figure(figsize=(6, 6))
    plt.imshow(img, cmap='gray')

    # Draw black lines between each waypoint
    for i in range(len(pix_coords)):
        pt1 = pix_coords[i]
        pt2 = pix_coords[(i + 1) % len(pix_coords)]
        plt.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], 'black', linewidth=0.5)

    # Plot each waypoint with its corresponding throttle color
    plt.scatter(pix_coords[:, 0], pix_coords[:, 1], c=throttle_colors, s=30)

    # Create a custom legend for the throttle colors
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', label='100% Throttle (Fast)', markerfacecolor='red', markersize=10),
        Line2D([0], [0], marker='o', color='w', label='60% Throttle (Medium)', markerfacecolor='yellow', markersize=10),
        Line2D([0], [0], marker='o', color='w', label='30% Throttle (Slow)', markerfacecolor='green', markersize=10)
    ]
    plt.legend(handles=legend_elements, loc='lower right')

    # Save and display the final map
    plt.title("Step 3: Colored Waypoints with Throttle Legend")
    plt.axis('off')
    plt.savefig("step3_middleline_throttle_legend.png", dpi=200)
    plt.show()

    # Save the results
    if save:
        # Save coordinates and throttle values to CSV
        df = pd.DataFrame(spaced_coords, columns=["x", "y"])
        df["throttle"] = throttle_values
        df.to_csv("middleline.csv", index=False)

        # Create a binary map showing just the waypoint positions
        binary_out = np.zeros_like(img, dtype=np.uint8)
        for row, col in np.round(pix_coords).astype(int):
            if 0 <= row < binary_out.shape[0] and 0 <= col < binary_out.shape[1]:
                binary_out[int(row), int(col)] = 255
        cv2.imwrite("middleline_binary.pgm", binary_out)

    # Return the raw path and waypoints
    return sorted_path, spaced_coords

# Run the function on a given .png or .pgm map
extract_middleline(
    r"/home/f1tenth/F1tenthcpp/f1tenth_cpp_ws/maps/test1.pgm",  # Path to map image
    resolution=0.05,                         # Meters per pixel
    origin=(-1.97, -1.64),         # Origin from map.yaml
    spacing=0.6,                            # Waypoint spacing (meters)
    curvature_window=3,                      # Curvature sampling window
    save=True                                # Whether to save outputs
)
