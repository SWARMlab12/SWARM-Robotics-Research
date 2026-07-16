import numpy as np
from rps.robotarium import Robotarium


r = Robotarium(
    number_of_robots=1,
    show_figure=True,
    initial_conditions=np.array([
        [-1.0], #x cordinate
        [0.0], #y cordinate
        [0.0] #unit circle orientation
    ])
)
goal_x = np.random.uniform(-1.5,1.5)
goal_y = np.random.uniform(-1,1)
r._axes_handle.plot(
    [-1.0, goal_x],
    [0, goal_y],
    linewidth=3
)

pi=np.pi
vel = 0.25
acc_error = 0.05

for i in range(900):

    poses = r.get_poses()
    x = goal_x - poses[0,0]
    y=goal_y-poses[1,0]
    theta = np.atan2(y,x)
    delta_theta = theta - poses[2,0]
    while delta_theta < -pi:
        delta_theta = delta_theta+ 2*pi
    while delta_theta > pi:
        delta_theta = delta_theta - 2*pi
    
    dist = (x**2 + y**2)**0.5
    if dist > acc_error:
        if delta_theta == 0:
            vel = 0.25
        else:
            vel = 0.15
    else:
        vel = 0
    dxu = np.array([
        [vel], #linear velocity m/s
        [delta_theta] # angular velocity
    ])

    r.set_velocities(np.arange(1), dxu)
    
    r.step()