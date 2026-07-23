import numpy as np
from rps.robotarium import Robotarium
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt
import random

r = Robotarium(
    number_of_robots=1,
    show_figure=True,
    initial_conditions=np.array([
        [-1.0], #x cordinate
        [0.0], #y cordinate
        [0.0] #unit circle orientation
    ])
)



rows=2*100 #offset 1 into coordinate plane
cols = 3*100 #offset 1.5 
cell = 1/100

known_map = np.zeros((rows,cols), dtype=int) #map robots make
acc_map = np.zeros((rows,cols), dtype=int) # 0 means unkown, 1 is obstacle, 2 is cleared

# to go from robotarium coordinates to rows and columns:
    # x = np.floor(100*x)+150
    #y = 100-np.floor(100*y)

def x_to_col(x):
    x = int(np.floor(100*x)+150)
    return x
def y_to_row(y):
    y = int(100-np.floor(100*y))
    return y

def make_obs(num):
    for i in range(num):
        obs_height = random.randint(1,30)
        obs_legnth = random.randint(1,30)

        obs_cords = [obs_height,obs_legnth,random.randint(0,rows-obs_height),random.randint(0,cols-obs_legnth)] #height, legnth, starting row (y), starting col (x)

        check_obs = acc_map[obs_cords[2]:obs_cords[2]+obs_cords[0],obs_cords[3]:obs_cords[3]+obs_cords[1]]
        if np.all(check_obs==0):
            acc_map[obs_cords[2]:obs_cords[2]+obs_cords[0],obs_cords[3]:obs_cords[3]+obs_cords[1]] = 1
            rect = patches.Rectangle(
            (obs_cords[3] * cell - 1.5, 1- (obs_cords[2] + obs_cords[0])*cell),
            obs_cords[1]*cell,           # width
            obs_cords[0]*cell,           # height
            color='black'
            )
            r._axes_handle.add_patch(rect)

r_vision = 50 #radius of vision circle
r_avoid = 40 # cells

fov = np.pi/3
def obs_detect(poses,acc_map,robot_map):
    robot_x = int(np.floor(poses[0,0]*100)+150)
    robot_y = int(100-np.floor(poses[1,0]*100))
    robot_theta = poses[2,0]

    start_angle = robot_theta - fov/2
    end_angle = robot_theta + fov/2
    angles = np.linspace(start_angle,end_angle,60)

    for angle in angles:
        for step in range(r_vision):
            x = step*np.cos(angle)+robot_x
            y = step*np.sin(angle)+robot_y
            x = int(np.round(x))
            y = int(np.round(y))
            if x > 299 or x < 0 or y > 199 or y<0:
                break
            if acc_map[y,x] == 1:
                robot_map[y,x] = 1
                break
            robot_map[y,x] = 2
    return robot_map

def draw_sensor(r, old_sector, x, y, theta, radius=r_vision*cell, fov=fov):

    # remove previous sensor cone
    if old_sector is not None:
        old_sector.remove()

    points = [(x, y)]

    start_angle = theta - fov/2
    end_angle = theta + fov/2

    angles = np.linspace(start_angle, end_angle, 60)

    for angle in angles:
        px = x + radius * np.cos(angle)
        py = y + radius * np.sin(angle)

        points.append((px, py))

    sector = Polygon(
        points,
        closed=True,
        color="blue",
        alpha=0.2
    )

    r._axes_handle.add_patch(sector)

    return sector

vel = 0.25
delta_theta = 0.00
time = 1/30
robot_radius = 11 #in cells

def move(poses):
    global known_map
    global r_avoid
    global vel
    global delta_theta
    global robot_radius
    x = poses[0,0]
    y = poses[1,0]
    theta = poses[2,0]
    start_angle = theta - fov/2
    end_angle = theta + fov/2
    angles = np.linspace(start_angle,end_angle,60)
    must_turn = [False,0]
    for angle in angles:
        for radius in range(r_avoid,0,-1):
            future_x = x + radius*np.cos(angle)*cell
            future_y = y + radius*np.sin(angle)*cell
            future_x = x_to_col(future_x)
            future_y = y_to_row(future_y)
            for out_x in range(future_x-robot_radius,future_x+robot_radius+1):
                for out_y in range(future_y-robot_radius,future_y+robot_radius+1):
                    if out_x <0 or out_x>299 or out_y <0 or out_y > 199:
                        must_turn = [True,0.5,angle]
                    if ((out_x-future_x)**2 + (out_y - future_y)**2) <= (robot_radius**2):
                        if known_map[out_y,out_x] == 1:
                            must_turn = [True,radius,angle]
    if must_turn[0]:
        vel = 0.1
        if must_turn[2] > theta:
            delta_theta = -1/must_turn[1]
        else:
            delta_theta = 1/must_turn[1]
    else:
        vel = 0.25
        delta_theta = 0.0

    
blue_cells = set()
def blue_obs():
    global known_map

    for y in range(rows):
        for x in range(cols):

            if known_map[y,x] == 1 and (y,x) not in blue_cells:

                rect1 = patches.Rectangle(
                    (x * cell - 1.5, 1 - (y+1)*cell),
                    cell,
                    cell,
                    color='blue'
                )

                r._axes_handle.add_patch(rect1)

                blue_cells.add((y,x))

#---------------------------------------------
make_obs(9)


sector = None
map_plot = None
for iteration in range(4500):
    if not plt.fignum_exists(r._axes_handle.figure.number):# this is jsut to quit the program when click x
        break

    poses = r.get_poses()
    known_map = obs_detect(poses,acc_map,known_map)
    map_image.set_data(known_map)

    move(poses)

    dxu = np.array([
            [vel], #linear velocity m/s
            [delta_theta] # angular velocity
        ])

    sector = draw_sensor(
    r,
    sector,
    poses[0,0],
    poses[1,0],
    poses[2,0]
)

    r.set_velocities(np.arange(1), dxu)

    blue_obs()

    r.step()
print("done")
