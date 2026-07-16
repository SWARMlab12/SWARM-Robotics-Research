import numpy as np
from rps.robotarium import Robotarium
from matplotlib.patches import Polygon

r = Robotarium(
    number_of_robots=1,
    show_figure=True,
    initial_conditions=np.array([
        [-1.0], #x cordinate
        [0.0], #y cordinate
        [0.0] #unit circle orientation
    ])
)

r1 = 0.5
r_safety = 0.2
fov = np.pi/3

def draw_sensor(r,old_sector, x, y, theta, radius=0.5, fov=np.pi/3):
    if old_sector is not None:
        old_sector.remove()

    points = [(x, y)]

    start_angle = theta - fov/2
    end_angle = theta + fov/2

    angles = np.linspace(start_angle, end_angle, 30)

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

obs = 5+4
obstacles = []
obstacle_line_divisions_x = []
obstacle_line_divisions_y = []
seen_points = set()
seen_x = []
seen_y = []
obstacles.append([-1.5,1,1.5,1])
obstacles.append([-1.5,-1,1.5,-1])
obstacles.append([-1.5,1,-1.5,-1])
obstacles.append([1.5,1,1.5,-1])
for i in range(obs-4):
    x = np.random.uniform(-1.5,1.5)
    y = np.random.uniform(-1,1)
    x1 = np.random.uniform(-1.5,1.5)
    y1 = np.random.uniform(-1,1)
    #r._axes_handle.plot([x, x1],[y, y1],linewidth=1,color = "black")
    obstacles.append([x,y,x1,y1])
for i in obstacles:
    obstacle_line_divisions_x.append(np.linspace(i[0],i[2],50))
    obstacle_line_divisions_y.append(np.linspace(i[1],i[3],50))


def make_obs(r, poses):
        for number in range(obs):
            for number_1 in range(50):
                obs_x_n = obstacle_line_divisions_x[number][number_1]
                obs_y_n = obstacle_line_divisions_y[number][number_1]
                check_1 = (obs_x_n-poses[0,0])**2 + (obs_y_n-poses[1,0])**2
                check_2 = np.atan2(obs_y_n-poses[1,0],obs_x_n-poses[0,0])
                check_1 = check_1 <= (r1**2)
                check_2 = (poses[2,0]-(fov/2)) <= check_2 and check_2 <= (poses[2,0]+(fov/2))
                if check_1 and check_2:
                    if (obs_x_n,obs_y_n) not in seen_points:
                        seen_points.add((obs_x_n,obs_y_n))
                        seen_x.append(obs_x_n)
                        seen_y.append(obs_y_n)



vel = 0.25
delta_theta = 0.0

def avoid_obs(poses, seen_points):
    global vel
    global delta_theta
    x_pos = r_safety*np.cos(poses[2,0])+poses[0,0]
    y_pos = r_safety*np.sin(poses[2,0])+poses[1,0]
    must_move = 0
    for i in seen_points:
        obs_x = i[0]
        obs_y=i[1]
        check_3 = (obs_x-x_pos)**2 + (obs_y-y_pos)**2
        check_4 = np.atan2(obs_y-poses[1,0],obs_x-poses[0,0])
        check_3 = check_3 <= (r_safety**2)
        check_4 = (poses[2,0]-(np.pi/12)) <= check_4 and check_4 <= (poses[2,0]+(np.pi/12))
        if check_3 and check_4:
            must_move = 1
    if must_move:
        if vel >0:
            vel-=0.025
        if vel <0:
            vel = 0
        if delta_theta > -0.35:
            delta_theta -= 0.1
        if delta_theta < -0.35:
            delta_theta = -0.35
    else:
        if vel < 0.25:
            vel +=0.015
        if vel > 25:
            vel = 0.25
        if delta_theta <0:
            delta_theta += 0.1
        if delta_theta > 0:
            delta_theta = 0.0  



sector = None
map_plot = None

for iteration in range(4500):
    poses = r.get_poses()

    sector = draw_sensor(
    r,
    sector,
    poses[0,0],
    poses[1,0],
    poses[2,0],r1,fov
    )

    if map_plot is not None:
        map_plot.remove()
    make_obs(r, poses)
    map_plot = r._axes_handle.scatter(
    seen_x,
    seen_y,
    color="red",
    s=5
    )

    avoid_obs(poses, seen_points)

    dxu = np.array([
        [vel], #linear velocity m/s
        [delta_theta] # angular velocity
    ])

    r.set_velocities(np.arange(1), dxu)
    r.step()