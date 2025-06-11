# AI-based-Robot-Navigation-Simulation
This project is a Python simulation of robot path planning in a 2D grid environment using the **A\*** algorithm. It demonstrates how an autonomous robot can find the shortest path from a start point to a goal while avoiding obstacles.

## Features

- 2D grid-based environment with configurable size.
- Randomly placed obstacles to simulate real-world constraints.
- Implementation of the A* search algorithm for efficient pathfinding.
- Visualization of the grid, obstacles, start & goal points, and the calculated path using `matplotlib`.
- Suitable for learning basic AI search algorithms applied in robotics without physical hardware.

## How it works

1. Define the grid size, start, and goal positions.
2. Randomly place obstacles on the grid.
3. Run the A* pathfinding algorithm to find the shortest path from start to goal.
4. Visualize the environment and the path in a graphical window.
![image](https://github.com/user-attachments/assets/e20286fe-da8b-4381-bfda-afa04d8368c0)

## Requirements

- Python 3.6+
- matplotlib
- numpy

You can install the required packages with:

```bash
pip install matplotlib numpy
