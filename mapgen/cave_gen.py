import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from random import random

def simple_noise(t, offset=0):
    return np.sin(t * 0.3 + offset) + 0.5 * np.sin(t * 0.7 + offset)

def generate_tunnel_path(start_pos, direction, length=100, noise_strength=0.1):
    path = [np.array(start_pos)]
    current_pos = np.array(start_pos, dtype=float)
    current_dir = np.array(direction, dtype=float)

    for i in range(length):
        # Add noise to direction for curves
        noise_x = simple_noise(i, offset=0) * noise_strength
        noise_y = simple_noise(i, offset=100) * noise_strength
        noise_z = simple_noise(i, offset=200) * noise_strength

        # Update direction with noise
        current_dir[0] += noise_x
        current_dir[1] += noise_y
        current_dir[2] += noise_z

        # Normalize to prevent runaway
        current_dir = current_dir / np.linalg.norm(current_dir)

        # Step forward
        current_pos += current_dir * 0.5
        path.append(current_pos.copy())

    return np.array(path)

def create_cave_network(num_branching=5):
    tunnels = []
    length = 120

    # Main tunnel
    main_tunnel = generate_tunnel_path(
        start_pos=[0, 0, 0],
        direction=[1, 0, 0],
        length=length,
        noise_strength=0.15
    )
    tunnels.append(main_tunnel)
    cur_tunnel = main_tunnel

    length *= 0.8
    threshold = 0.1
    max_tunnel = 5
    count = 0
    for _ in range(num_branching):
        for i in range(len(cur_tunnel)):
            if threshold < random():
                continue

            if max_tunnel == count:
                break
            branch_start = cur_tunnel[i]
            direction=[random()*random(), random()*random(), random()*random()]
            branch = generate_tunnel_path(
                   start_pos=branch_start,
                   direction=direction,
                   length=int(length),
                   noise_strength=0.10
            )
            tunnels.append(branch)
            count += 1

        threshold *= 0.7
        length *= 0.7
        count = 0

    return tunnels

def main():
    tunnels = create_cave_network()

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot each tunnel
    colors = ['brown', 'orange', 'goldenrod', 'chocolate']
    linewidths = [4, 3, 3, 3]

    for i, tunnel in enumerate(tunnels):
        color = colors[i % len(colors)]
        linewidth = linewidths[i % len(linewidths)]

        # Plot entire tunnel as one continuous line
        ax.plot(tunnel[:, 0], tunnel[:, 1], tunnel[:, 2],
                color=color, linewidth=linewidth, alpha=0.8)

    # Mark entrance
    ax.scatter([0], [0], [0], color='green', s=100, label='Entrance')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z (Depth)')
    ax.set_title('Cave Network')
    ax.invert_zaxis()  # Show depth going down

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
