from rps.robotarium import Robotarium
import numpy as np


# Create two robots
initial_conditions = np.array([
    [0.0, 0.5, 0.0],
    [0.0, -0.5, 0.0]
]).T


# Start simulation
r = Robotarium(
    number_of_robots=2,
    show_figure=True,
    initial_conditions=initial_conditions
)

print("Robotarium simulation started!")

# Simulation loop
for i in range(100):
    poses = r.get_poses()

    # No movement commands yet
    velocities = np.zeros((2, 2))

    r.set_velocities(
        np.arange(2),
        velocities
    )

    r.step()



print("Simulation finished!")