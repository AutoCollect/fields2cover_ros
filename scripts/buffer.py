from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union
import matplotlib.pyplot as plt
import numpy as np
import math
import os
# from catmull_rom import num_segments, flatten, catmull_rom_spline, catmull_rom_chain
from utils import num_segments, catmull_rom_chain, load_data, calculate_point_on_line, calculate_distance, interpolate_path_large, calculate_heading, heading_to_quaternion, simplify_polygon

import fields2cover as f2c
from ccma import CCMA

ccma = CCMA(w_ma=10, w_cc=10)

source_file_path = '/home/aucobot-p4/demo_ws/data/global_odom.txt'
file_path = "/home/aucobot-p4/demo_ws/data/test2.txt"

############
# buffer
buffer_distance = -1.0
# first ring
inward_offset_distance = -1
# distance between end of ring and start of ring
distance_away = -2*inward_offset_distance  # meters
# step size interpolation
step_size = 0.01
############


##### source data
points = load_data(source_file_path)
original_polygon = Polygon(points)

# original_polygon = Polygon([(0,0),(50,0),(50,50),(0,50),[0,0]])

##### remove noise
original_polygon = Polygon(points)
original_polygon = simplify_polygon(points, epsilon=0.8)  
##### generate two polygons
original_polygon = original_polygon.buffer(buffer_distance)
first_ring_polygon = original_polygon.buffer(inward_offset_distance)
second_ring_polygon = original_polygon.buffer(inward_offset_distance * 3)


##### link two polygons together

start_point_first_ring = first_ring_polygon.exterior.coords[0]
end_point_first_ring = first_ring_polygon.exterior.coords[-2]

start_point_second_ring = second_ring_polygon.exterior.coords[0]
end_point_second_ring = second_ring_polygon.exterior.coords[-2]

# Calculate the point
first_point_away = calculate_point_on_line(start_point_first_ring, end_point_first_ring, distance_away)
print(f"Point 2 meters away on the fourth line: {first_point_away}")

second_point_away = calculate_point_on_line(start_point_second_ring, end_point_second_ring, distance_away)
print(f"Point 2 meters away on the fourth line: {second_point_away}")

first_ring_coords = list(first_ring_polygon.exterior.coords)
second_ring_coords = list(second_ring_polygon.exterior.coords)

first_ring_coords = interpolate_path_large(first_ring_coords, 2.0)
second_ring_coords = interpolate_path_large(second_ring_coords, 2.0)

for i, coord in enumerate(first_ring_coords):
    if calculate_distance(first_ring_coords[0], first_ring_coords[-i])>distance_away:
        new_path_coords = first_ring_coords[:-i]
        break

new_path_coords.append(first_point_away)  # Move to the start of the second ring

for i, coord in enumerate(second_ring_coords):
    if calculate_distance(second_ring_coords[0], second_ring_coords[-i])>distance_away:
        new_path_coords += second_ring_coords[:-i]
        break

new_path_coords.append(second_point_away)  # Close the loop by returning to the start of the second ring




# Create a LineString from the new path
new_path2 = LineString(new_path_coords)
interpolated_path_coords_1 = new_path_coords

points = []
for point in list(second_ring_polygon.exterior.coords):
    points.append(f2c.Point(point[0], point[1]))

field = f2c.Cells(f2c.Cell(f2c.LinearRing(f2c.VectorPoint(points))))
cells = field
robot = f2c.Robot(1.0, 2.0)
const_hl = f2c.HG_Const_gen()
no_hl = const_hl.generateHeadlands(cells, -inward_offset_distance)
bf = f2c.SG_BruteForce()


# swaths = bf.generateSwaths(0.0, robot.op_width, no_hl.getGeometry(0))
n_swath = f2c.OBJ_NSwath()
swaths = bf.generateBestSwaths(n_swath, robot.op_width, no_hl.getGeometry(0))


snake_sorter = f2c.RP_Boustrophedon()

swaths = snake_sorter.genSortedSwaths(swaths)

robot.setMinRadius(2.0)
path_planner = f2c.PP_PathPlanning()
dubins = f2c.PP_DubinsCurves()
path = path_planner.searchBestPath(robot, swaths, dubins)

for state in path.states:
    interpolated_path_coords_1 = np.concatenate((interpolated_path_coords_1, [np.array([state.point.getX(), state.point.getY()])]))

interpolated_path_coords_1 = interpolate_path_large(interpolated_path_coords_1, 0.5)

smoothed_points = ccma.filter(np.array(interpolated_path_coords_1))

interpolated_path_coords_1 = interpolate_path_large(smoothed_points, step_size)

new_path_1 = LineString(interpolated_path_coords_1)

if os.path.exists(file_path):
    os.remove(file_path)
    print("File deleted successfully.")
else:
    print("The file does not exist.")

# Add Quaternions
quaternions = []
for i in range(len(interpolated_path_coords_1) - 1):
    point1 = interpolated_path_coords_1[i]
    point2 = interpolated_path_coords_1[i + 1]
    theta = calculate_heading(point1[0], point1[1], point2[0], point2[1])
    quaternion = heading_to_quaternion(theta)
    quaternions.append(quaternion)
    if not math.isnan(quaternion[3]):
        with open(file_path, 'a') as file:
            file.write(f'{round(point1[0], 5)} {round(point1[1], 5)} 0.0 {quaternion[0]} {quaternion[1]} {quaternion[2]} {quaternion[3]}\n')

# new_path_1 = LineString(smoothed_points)

# For visualization purposes, let's plot the original polygon, the two rings, and the new path
# import matplotlib.pyplot as plt

# fig, ax = plt.subplots()
# x, y = new_path2.xy
# ax.plot(x, y, 'r-', label='Original Polygon')
# x, y = new_path_1.xy
# ax.plot(x, y, 'k-', label='Polygon with smooth corners')
# ax.scatter(x, y)
# # Set aspect ratio to equal to avoid distortion
# ax.set_aspect('equal', adjustable='box')


# plt.legend()
# plt.show()


import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

fig, ax = plt.subplots()
x, y = new_path2.xy
ax.plot(x, y, 'r-', label='interpolate 1 meter first')
x, y = new_path_1.xy
ax.plot(x, y, 'k-', label='interpolate 1 meter first')
ax.set_aspect('equal', adjustable='box')


plt.legend()
plt.savefig('global_planner.png', dpi=300)