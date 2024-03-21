import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from matplotlib import pyplot as plt
from scipy.spatial import cKDTree
import numpy as np
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union
import matplotlib.pyplot as plt
import numpy as np
import math
import os
from utils import num_segments, catmull_rom_chain, load_data, calculate_point_on_line, calculate_distance, interpolate_path, calculate_heading, heading_to_quaternion, simplify_polygon, plot_polygons, load_data

import fields2cover as f2c



file_path = '/home/chen/auto_collect/src/autocollect_ws/fields2cover_ros/scripts/global_odom.txt'

points = load_data(file_path)

# points = [(0,0),(50,0),(50,50),(0,50),[0,0]]

original_polygon = Polygon(points)
simplified_polygon = simplify_polygon(points, epsilon=0.8)  

plot_polygons(original_polygon, simplified_polygon)

simplified_coords = np.array(simplified_polygon.exterior.coords)

############
# first ring
inward_offset_distance = -1.0
# distance between end of ring and start of ring
distance_away = -2*inward_offset_distance  # meters
# step size interpolation
step_size = 0.01
# turn radians
turn_radius = 0.8
############

original_polygon = Polygon(simplified_coords)

first_ring_polygon = original_polygon.buffer(inward_offset_distance)
first_ring_polygon_coords = first_ring_polygon.exterior.coords
second_ring_polygon = original_polygon.buffer(inward_offset_distance * 3)

x1, y1 = second_ring_polygon.exterior.coords[0]
# orientation_radians = math.atan2(y1 - 0, x1 - 0) + 1.57
# orientation_radians = math.atan2(y1 - 0, x1 - 0) + 1.57
orientation_radians = 0

second_ring_polygon_coords = second_ring_polygon.exterior.coords

first_ring_polygon_coords = interpolate_path(first_ring_polygon_coords, distance_away-0.1)
second_ring_polygon_coords = interpolate_path(second_ring_polygon_coords, distance_away-0.1)

start_point_first_ring = first_ring_polygon_coords[0]
end_point_first_ring = first_ring_polygon_coords[-2]

start_point_second_ring = second_ring_polygon_coords[0]
end_point_second_ring = second_ring_polygon_coords[-2]

first_point_away = calculate_point_on_line(start_point_first_ring, end_point_first_ring, distance_away)
print(f"Point 2 meters away on the fourth line: {first_point_away}")

second_point_away = calculate_point_on_line(start_point_second_ring, end_point_second_ring, distance_away)
print(f"Point 2 meters away on the fourth line: {second_point_away}")


for i, coord in enumerate(first_ring_polygon_coords):
    if calculate_distance(first_ring_polygon_coords[0], first_ring_polygon_coords[-i])>distance_away:
        new_path_coords = first_ring_polygon_coords[:-i]
        break

new_path_coords.append(first_point_away)

for i, coord in enumerate(second_ring_polygon_coords):
    if calculate_distance(second_ring_polygon_coords[0], second_ring_polygon_coords[-i])>distance_away:
        new_path_coords += second_ring_polygon_coords[:-i]
        break

new_path_coords.append(second_point_away)
file_path = '/home/chen/auto_collect/src/autocollect_ws/fields2cover_ros/scripts/main.txt'
if os.path.exists(file_path):
    os.remove(file_path)
    print("File deleted successfully.")
else:
    print("The file does not exist.")

NUM_POINTS: int = 100

new_path2 = LineString(new_path_coords)

first_point = new_path_coords[0]
last_point = new_path_coords[-1]

chain_points_1: list = catmull_rom_chain(new_path_coords, NUM_POINTS)
assert len(chain_points_1) == num_segments(new_path_coords) * NUM_POINTS

interpolated_path_coords_1 = np.concatenate(([np.array(first_point)], chain_points_1, [np.array(last_point)]))


points = []
for point in list(second_ring_polygon.exterior.coords):
    points.append(f2c.Point(point[0], point[1]))

field = f2c.Cells(f2c.Cell(f2c.LinearRing(f2c.VectorPoint(points))))
cells = field
robot = f2c.Robot(1, 1)
optim = f2c.OptimizationParams
const_hl = f2c.HG_Const_gen()
no_hl = const_hl.generateHeadlands(cells, -inward_offset_distance)
bf = f2c.SG_BruteForce()

n_swath = f2c.OBJ_NSwath()
swaths = bf.generateBestSwaths(n_swath, robot.op_width, no_hl.getGeometry(0))

snake_sorter = f2c.RP_Boustrophedon()

swaths = snake_sorter.genSortedSwaths(swaths)

robot.setMinRadius(turn_radius)
path_planner = f2c.PP_PathPlanning()
dubins = f2c.PP_DubinsCurves()
path = path_planner.searchBestPath(robot, swaths, dubins)

for state in path.states:
    interpolated_path_coords_1 = np.concatenate((interpolated_path_coords_1, [np.array([state.point.getX(), state.point.getY()])]))

interpolated_path_coords_1 = interpolate_path(interpolated_path_coords_1, step_size)

new_path_1 = LineString(interpolated_path_coords_1)

# Add Quaternions
quaternions = []
for i in range(len(interpolated_path_coords_1) - 1):
    point1 = interpolated_path_coords_1[i]
    point2 = interpolated_path_coords_1[i + 1]
    theta = calculate_heading(point1[0], point1[1], point2[0], point2[1])
    quaternion = heading_to_quaternion(theta)
    quaternions.append(quaternion)
    if not math.isnan(quaternion[3]):
        with open('/home/chen/auto_collect/src/autocollect_ws/fields2cover_ros/scripts/main.txt', 'a') as file:
            file.write(f'{round(point1[0], 5)} {round(point1[1], 5)} 0.0 {quaternion[0]} {quaternion[1]} {quaternion[2]} {quaternion[3]}\n')

import matplotlib.pyplot as plt

fig, ax = plt.subplots()
x, y = new_path_1.xy
ax.plot(x, y, 'k-', label='interpolate 1 meter first')
ax.set_aspect('equal', adjustable='box')


plt.legend()
plt.show()
