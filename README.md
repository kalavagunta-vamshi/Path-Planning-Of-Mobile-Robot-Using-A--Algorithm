# A* Path Planning Implementation

## Overview
The A* (pronounced "A-star") algorithm is a path planning and graph traversal algorithm widely used in robotics. It is especially useful for finding the shortest path between two nodes. This repository contains implementations of the A* algorithm in both 2-D and 3-D environments, utilizing Python and ROS Noetic for simulation in Gazebo.

## Dependencies
- `numpy`
- `heapq`
- `matplotlib`
- `time`
- `math`

## Instructions

### Part 1: 2D Implementation
1. Run the script with Python 3:
   ```bash
   python3 part_1.py
   ```
2. Input the following details:
   - Clearance of robot (in meters): `0.1`
   - Left and right RPMs: `10 20`
   - Start coordinates (in meters): `-0.25 -0.5`
   - Robot start angle (degrees): `0`
   - Goal coordinates (in meters): `5 -0.5`

### Part 2: 3D Implementation
1. Place the `Part02` package in the source folder of your catkin workspace.
2. Launch the robot simulation:
   ```bash
   roslaunch Part02 bot.launch
   ```
3. In a new terminal, run the script:
   ```bash
   python3 part_2.py
   ```

#### Test Cases
- **Test Case 1:**
  - Clearance: `0.1`, RPMs: `30 20`, Start: `0 0`, Angle: `0`, Goal: `5.3 -0.5`
- **Test Case 2:**
  - Clearance: `0.1`, RPMs: `30 20`, Start: `0 0`, Angle: `0`, Goal: `5 0`

## Visualization Instructions
To visualize the exploration and pathfinding animation, uncomment line 188:
```python
#plt.pause(0.000000001)
```

## Demo Videos
- [Part 1 Demo](https://drive.google.com/file/d/10kRCP2tK4UXXdxRaQT-3ReDn1fS_8_lY/view?usp=share_link)
- [Part 2 Demo 1](https://drive.google.com/file/d/1AJL7qBV6aPueJ8R7CJWWdDk0XUdpG3E4/view?usp=share_link)
- [Part 2 Demo 2](https://drive.google.com/file/d/1XJ6IKED7USyVjtfyQhxSP8GRFA6pl9dP/view?usp=share_link)

## Contributors
- **Surya Chappidi** - [Profile](https://github.com/Suryachappidi)
- **Vamshi Kalavagunta** - [Profile](https://github.com/kalavagunta-vamshi)

