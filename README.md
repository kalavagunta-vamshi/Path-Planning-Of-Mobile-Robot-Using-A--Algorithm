# A-star Algorithm

##### Surya Chappidi (UID:119398166)(Directory ID: Chappidi)
##### Vamshi Kalavagunta (UID: 119126332)(Directory ID: vamshik) 

### Github Link: https://github.com/Suryachappidi/proj3_astar

### Demo videos
part1: https://drive.google.com/file/d/10kRCP2tK4UXXdxRaQT-3ReDn1fS_8_lY/view?usp=share_link

part2 demo1: https://drive.google.com/file/d/1AJL7qBV6aPueJ8R7CJWWdDk0XUdpG3E4/view?usp=share_link

part2 demo2: https://drive.google.com/file/d/1XJ6IKED7USyVjtfyQhxSP8GRFA6pl9dP/view?usp=share_link

### Dependencies 
* numpy
* heapq
* matplotlib
* Time
* Math

### Instructions part 1
* Run python3 part_1.py

Input:

1. Enter the clearance of robot(in meter): 0.1 
2. Enter the left and right RPMs: 10 20
3. Enter the start coordinates(in meter): -0.25 -0.5
4. Enter the robot start angle: 0
5. Enter the goal coordinates(in meter): 5 -0.5

### Instructions part 2
* Place the Part02 package in source folder of your catkin_workspace
* open terminal and go to launch folder location in Part02 and run roslaunch bot.launch
* in a new terminal go to src located in Part02 and run python3 part_2.py

Test case1:

1. Enter the clearance of robot(in meter): 0.1
2. Enter the left and right RPMs: 30 20
3. Enter the start coordinates(in meter): 0 0
4. Enter the robot start angle: 0
5. Enter the goal coordinates(in meter): 5.3 -0.5

Test case2:

1. Enter the clearance of robot(in meter): 0.1
2. Enter the left and right RPMs: 30 20
3. Enter the start coordinates(in meter): 0 0
4. Enter the robot start angle: 0
5. Enter the goal coordinates(in meter): 5 0

### Visualisation Instruction:
* uncomment line 188 (i.e #plt.pause(0.000000001)) to visualise animation of exploration and optimal path travel.