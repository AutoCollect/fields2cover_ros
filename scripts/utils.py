import numpy as np
import math
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString

# Load your data
def load_data(file_path):
    with open(file_path, 'r') as file:
        points = [line.strip().split(' ') for line in file]  # Adjust split based on your data format
        points = [(float(point[0]), float(point[1])) for point in points]
    return points

def calculate_point_on_line(start_point, end_point, distance):
    # Calculate the direction vector of the line
    direction = [end_point[0] - start_point[0], end_point[1] - start_point[1]]
    
    # Normalize the direction vector
    norm = math.sqrt(direction[0]**2 + direction[1]**2)
    direction_normalized = [direction[0] / norm, direction[1] / norm]
    
    # Calculate the point at the specified distance along the line
    point_at_distance = [start_point[0] + direction_normalized[0] * distance,
                         start_point[1] + direction_normalized[1] * distance]
    return point_at_distance

def calculate_distance(start_point, end_point):
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]
    return np.sqrt(dx**2 + dy**2)

def calculate_heading(x1, y1, x2, y2):
    return np.arctan2(y2 - y1, x2 - x1)

def heading_to_quaternion(theta):
    w = round(np.cos(theta / 2), 5)
    x = 0
    y = 0
    z = round(np.sin(theta / 2), 5)
    return (x, y, z, w)

def interpolate_path(path_coords, step_size):
    """Interpolates a path with given step size.
    
    Args:
        path_coords (list of tuples): The original path as a list of (x, y) tuples.
        step_size (float): The desired distance between consecutive points in the path.
        
    Returns:
        list of tuples: The interpolated path as a list of (x, y) tuples.
    """
    interpolated_path = []
    for i in range(len(path_coords) - 1):
        start_point = path_coords[i]
        end_point = path_coords[i + 1]
        
        # Calculate the distance between the points
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        # Normalize the direction vector
        direction = (dx / distance, dy / distance)
        
        if distance == 0:
            continue
        
        # Calculate how many intermediate steps are needed
        num_steps = int(np.floor(distance / step_size))
        
        # Generate the intermediate points
        for step in range(num_steps + 1):  # +1 to include the end point of each segment
            interpolated_x = start_point[0] + direction[0] * step * step_size
            interpolated_y = start_point[1] + direction[1] * step * step_size
            interpolated_path.append((interpolated_x, interpolated_y))
    
    # Ensure the last point of the original path is included
    if len(path_coords) > 0:
        interpolated_path.append(path_coords[-1])
    
    return interpolated_path


QUADRUPLE_SIZE: int = 4


def num_segments(point_chain: tuple) -> int:
    # There is 1 segment per 4 points, so we must subtract 3 from the number of points  
    return len(point_chain) - (QUADRUPLE_SIZE - 1)


def flatten(list_of_lists) -> list:
    # E.g. mapping [[1, 2], [3], [4, 5]] to  [1, 2, 3, 4, 5] 
    return [elem for lst in list_of_lists for elem in lst]


def catmull_rom_spline(
    P0: tuple,
    P1: tuple,
    P2: tuple,
    P3: tuple,
    num_points: int,
    alpha: float = 1.0,
):
    """
    Compute the points in the spline segment
    :param P0, P1, P2, and P3: The (x,y) point pairs that define the Catmull-Rom spline
    :param num_points: The number of points to include in the resulting curve segment
    :param alpha: 0.5 for the centripetal spline, 0.0 for the uniform spline, 1.0 for the chordal spline.
    :return: The points
    """

    # Calculate t0 to t4. Then only calculate points between P1 and P2.
    # Reshape linspace so that we can multiply by the points P0 to P3
    # and get a point for each value of t.
    def tj(ti: float, pi: tuple, pj: tuple) -> float:
        xi, yi = pi
        xj, yj = pj
        dx, dy = xj - xi, yj - yi
        l = (dx ** 2 + dy ** 2) ** 0.5
        return ti + l ** alpha

    t0: float = 0.0
    t1: float = tj(t0, P0, P1)
    t2: float = tj(t1, P1, P2)
    t3: float = tj(t2, P2, P3)
    t = np.linspace(t1, t2, num_points).reshape(num_points, 1)

    A1 = (t1 - t) / (t1 - t0) * P0 + (t - t0) / (t1 - t0) * P1
    A2 = (t2 - t) / (t2 - t1) * P1 + (t - t1) / (t2 - t1) * P2
    A3 = (t3 - t) / (t3 - t2) * P2 + (t - t2) / (t3 - t2) * P3
    B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2
    B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3
    points = (t2 - t) / (t2 - t1) * B1 + (t - t1) / (t2 - t1) * B2
    return points


def catmull_rom_chain(points: tuple, num_points: int) -> list:
    """
    Calculate Catmull-Rom for a sequence of initial points and return the combined curve.
    :param points: Base points from which the quadruples for the algorithm are taken
    :param num_points: The number of points to include in each curve segment
    :return: The chain of all points (points of all segments)
    """
    point_quadruples = (  # Prepare function inputs
        (points[idx_segment_start + d] for d in range(QUADRUPLE_SIZE))
        for idx_segment_start in range(num_segments(points))
    )
    all_splines = (catmull_rom_spline(*pq, num_points) for pq in point_quadruples)
    return flatten(all_splines)




def interpolate_path_large(path_coords, step_size):
    """Interpolates a path with given step size.
    
    Args:
        path_coords (list of tuples): The original path as a list of (x, y) tuples.
        step_size (float): The desired distance between consecutive points in the path.
        
    Returns:
        list of tuples: The interpolated path as a list of (x, y) tuples.
    """
    interpolated_path = []
    for i in range(len(path_coords) - 1):
        start_point = path_coords[i]
        end_point = path_coords[i + 1]
        
        # Calculate the distance between the points
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        # Check for a valid distance before dividing
        if distance == 0 or np.isnan(distance) or step_size == 0:
            print("Warning: Skipping interpolation due to zero or undefined distance.")
            continue
        
        # Normalize the direction vector
        direction = (dx / distance, dy / distance)
        
        # Calculate how many intermediate steps are needed
        num_steps = max(int(np.floor(distance / step_size)), 1)  # Ensure at least 1 step
        
        # Generate the intermediate points
        for step in range(1, num_steps + 1):  # Start from 1 to avoid duplicating start_point
            interpolated_x = start_point[0] + direction[0] * step * step_size
            interpolated_y = start_point[1] + direction[1] * step * step_size
            interpolated_path.append((interpolated_x, interpolated_y))
    
    # Ensure the last point of the original path is included
    if len(path_coords) > 0:
        interpolated_path.append(path_coords[-1])
    
    return interpolated_path



# Load your data
def load_data(file_path):
    with open(file_path, 'r') as file:
        points = [line.strip().split(' ') for line in file]  # Adjust split based on your data format
        points = [(float(point[0]), float(point[1])) for point in points]
    return points

# Simplify the polygon
def simplify_polygon(points, epsilon=1.0):
    polygon = Polygon(points)
    simplified_polygon = polygon.simplify(epsilon, preserve_topology=True)
    return simplified_polygon

# Plotting function for visualization
def plot_polygons(original, simplified):
    fig, ax = plt.subplots()
    x, y = original.exterior.xy
    ax.plot(x, y, color='blue', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2, label='Original')

    x, y = simplified.exterior.xy
    ax.plot(x, y, color='red', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2, label='Simplified')

    ax.set_title('Polygon Simplification Parameter=1.0')
    ax.set_aspect('equal', adjustable='box')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.show()